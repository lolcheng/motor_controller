#include "encos_core/transaction_manager.hpp"

#include <algorithm>

namespace encos {
namespace {

using State = TransactionState;

constexpr auto max_clock_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::duration::max());

bool addressed_write(ReplyKind kind) {
  return kind == ReplyKind::set_id || kind == ReplyKind::zero;
}

bool overlaps(const ReplyKey& a, const ReplyKey& b) {
  if (a.id != b.id) {
    return false;
  }
  if (addressed_write(a.kind) && addressed_write(b.kind)) {
    if (a.motor == b.motor) {
      return true;  // 00 failure is shared by zero and set-ID.
    }
    return a.kind == ReplyKind::set_id && b.kind == ReplyKind::set_id && a.new_id == b.new_id;
  }
  return a.kind == b.kind && a.code == b.code;
}

bool matches(const ReplyKey& key, const Feedback& feedback) {
  switch (key.kind) {
    case ReplyKind::telemetry: {
      const auto* value = std::get_if<Telemetry>(&feedback);
      return value && value->type == key.code;
    }
    case ReplyKind::query: {
      const auto* value = std::get_if<QueryReply>(&feedback);
      return value && static_cast<std::uint8_t>(value->code) == key.code;
    }
    case ReplyKind::config: {
      const auto* status = std::get_if<ConfigStatus>(&feedback);
      const auto* echo = std::get_if<ConfigEcho>(&feedback);
      return (status && static_cast<std::uint8_t>(status->code) == key.code) ||
             (echo && static_cast<std::uint8_t>(echo->code) == key.code);
    }
    case ReplyKind::kt_group: {
      const auto* value = std::get_if<KtReply>(&feedback);
      return value && value->group == key.code;
    }
    case ReplyKind::brake: {
      const auto* value = std::get_if<BrakeReply>(&feedback);
      return value && (value->error ? 6 : 0xb2) == key.code;
    }
    case ReplyKind::discover:
    case ReplyKind::reset_id:
    case ReplyKind::zero:
    case ReplyKind::set_id: {
      const auto* value = std::get_if<ManagementReply>(&feedback);
      if (!value) {
        return false;
      }
      switch (value->kind) {
        case ManagementKind::discover: return key.kind == ReplyKind::discover;
        case ManagementKind::reset_id: return key.kind == ReplyKind::reset_id;
        case ManagementKind::zero: return key.kind == ReplyKind::zero && value->id == key.motor;
        case ManagementKind::set_id: return key.kind == ReplyKind::set_id && value->id == key.new_id;
        case ManagementKind::write_failed: return addressed_write(key.kind) && value->id == key.motor;
      }
      return false;
    }
    case ReplyKind::none: return false;
  }
  return false;
}

bool rejected(const Feedback& feedback, const CanFrame& response, const CanFrame& command) {
  if (const auto* value = std::get_if<Telemetry>(&feedback)) {
    return value->error != MotorError::none;
  }
  if (const auto* value = std::get_if<QueryReply>(&feedback)) {
    return value->error != MotorError::none;
  }
  if (const auto* value = std::get_if<ConfigStatus>(&feedback)) {
    return !value->success || value->error != MotorError::none;
  }
  if (std::holds_alternative<ConfigEcho>(feedback)) {
    return response.length != command.length + 1 ||
           !std::equal(command.data.begin() + 2, command.data.begin() + command.length,
                       response.data.begin() + 3);
  }
  if (const auto* value = std::get_if<ManagementReply>(&feedback)) {
    return !value->success;
  }
  if (const auto* value = std::get_if<BrakeReply>(&feedback)) {
    const auto target = static_cast<BrakeState>(command.data[command.length - 1]);
    return value->state != target || (value->error && value->error != MotorError::none);
  }
  if (std::holds_alternative<KtReply>(feedback) && command.length == 8) {
    return !std::equal(command.data.begin() + 2, command.data.begin() + 8, response.data.begin() + 2);
  }
  return false;
}

}  // namespace

TransactionManager::TransactionManager(std::optional<std::chrono::milliseconds> lifetime, ReplyReuse reuse)
    : reply_lifetime_(lifetime), reuse_(reuse) {
  if (lifetime && (*lifetime <= std::chrono::milliseconds::zero() || *lifetime > max_clock_ms)) {
    throw std::invalid_argument("reply lifetime must be positive");
  }
}

Begin TransactionManager::begin(const Encoded& command, TimePoint sent, TimePoint deadline) {
  if (pending_) {
    return Begin::busy;
  }
  if (command.frame.length < 2 || command.frame.length > 8 || command.reply.id != command.frame.id ||
      command.frame.id == 0 || command.frame.id > management_id || command.frame.rtr ||
      command.frame.error || command.frame.extended ||
      (!matches_wire_format(command.frame, WireFormat::classic) &&
       !matches_wire_format(command.frame, WireFormat::fd_brs))) {
    return Begin::invalid_command;
  }
  if (deadline <= sent || (last_send_ && sent < *last_send_) ||
      (reply_lifetime_ && (sent > TimePoint::max() - *reply_lifetime_ ||
                           deadline > sent + *reply_lifetime_)) ||
      command.next_send_delay < std::chrono::milliseconds::zero() ||
      command.next_send_delay > max_clock_ms ||
      sent > TimePoint::max() - command.next_send_delay) {
    return Begin::invalid_time;
  }
  if (next_send_ && sent < *next_send_) {
    return Begin::wait_after_write;
  }
  retired_.erase(std::remove_if(retired_.begin(), retired_.end(), [sent](const Retired& entry) {
    return entry.expires && sent > *entry.expires;
  }), retired_.end());
  if (command.reply.kind != ReplyKind::none &&
      std::any_of(retired_.begin(), retired_.end(), [&](const Retired& entry) {
        return overlaps(entry.key, command.reply);
      })) {
    return Begin::quarantined;
  }
  last_send_ = sent;
  next_send_ = sent + command.next_send_delay;
  if (command.reply.kind != ReplyKind::none) {
    pending_ = Pending{command, sent, deadline};
  }
  return Begin::accepted;
}

TransactionEvent TransactionManager::receive(const Decoded& decoded, TimePoint now) {
  if (!pending_) {
    return {};
  }
  if (now >= pending_->deadline) {
    return finish(State::timed_out);
  }
  const auto& sample = decoded.raw;
  const auto& command = pending_->command;
  if (sample.origin != Origin::remote || sample.stamp <= pending_->sent || sample.stamp > now ||
      sample.frame.id != command.reply.id || sample.frame.kind != command.frame.kind ||
      sample.frame.brs != command.frame.brs || !matches(command.reply, decoded.feedback)) {
    return {};
  }
  if (rejected(decoded.feedback, sample.frame, command.frame)) {
    return finish(State::rejected, decoded);
  }
  const auto* query = std::get_if<QueryReply>(&decoded.feedback);
  const auto* part = query ? std::get_if<UuidPart>(&query->value) : nullptr;
  if (!part) {
    return finish(State::completed, decoded);
  }
  if (part->index < 1 || part->index > pending_->uuid.size()) {
    return {};
  }
  auto& slot = pending_->uuid[part->index - 1];
  if (slot) {
    if (*slot != part->bytes) {
      return finish(State::rejected, decoded);
    }
    return {State::duplicate, decoded, {}};
  }
  slot = part->bytes;
  std::array<std::uint8_t, 12> uuid{};
  for (std::size_t i = 0; i < pending_->uuid.size(); ++i) {
    if (!pending_->uuid[i]) {
      return {State::partial, decoded, {}};
    }
    std::copy(pending_->uuid[i]->begin(), pending_->uuid[i]->end(), uuid.begin() + 4 * i);
  }
  return finish(State::completed, decoded, uuid);
}

TransactionEvent TransactionManager::poll(TimePoint now) {
  return pending_ && now >= pending_->deadline ? finish(State::timed_out) : TransactionEvent{};
}

TransactionEvent TransactionManager::cancel() {
  return pending_ ? finish(State::cancelled) : TransactionEvent{};
}

TransactionEvent TransactionManager::finish(State result, std::optional<Decoded> decoded,
                                            std::optional<std::array<std::uint8_t, 12>> uuid) {
  const auto delay = pending_->command.next_send_delay;
  if (decoded && delay > std::chrono::milliseconds::zero()) {
    // The peer can start its quiet interval after receiving and processing the write.
    next_send_ = std::min(decoded->raw.stamp, TimePoint::max() - delay) + delay;
  }
  std::optional<TimePoint> expires;
  if (reply_lifetime_) {
    expires = pending_->sent + *reply_lifetime_;
  }
  // A dedicated fault-free emulator can promise exactly one telemetry reply.
  // Any failed transaction remains subject to the original quarantine.
  if (reuse_ != ReplyReuse::single_reply_simulation || result != State::completed ||
      pending_->command.reply.kind != ReplyKind::telemetry) {
    retired_.push_back({pending_->command.reply, expires});
  }
  pending_.reset();
  return {result, decoded, uuid};
}

}  // namespace encos
