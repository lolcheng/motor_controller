#include "encos_driver/control_worker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace encos::driver {
namespace {
using namespace std::chrono_literals;
using S = TransactionState;
using M = msg::MotorCommand;
bool moving(SafetyState state) { return state == SafetyState::armed || state == SafetyState::running; }

std::optional<QueryCode> readback_query(ConfigCode code) {
  switch (code) {
    case ConfigCode::kt: return QueryCode::kt;
    case ConfigCode::kp_range: return QueryCode::kp_range;
    case ConfigCode::kd_range: return QueryCode::kd_range;
    case ConfigCode::position_range: return QueryCode::position_range;
    case ConfigCode::velocity_range: return QueryCode::velocity_range;
    case ConfigCode::torque_range: return QueryCode::torque_range;
    case ConfigCode::current_range: return QueryCode::current_range;
    case ConfigCode::timeout: return QueryCode::timeout;
    case ConfigCode::current_pi: return QueryCode::current_pi;
    case ConfigCode::velocity_pi: return QueryCode::velocity_pi;
    case ConfigCode::position_pd: return QueryCode::position_pd;
    case ConfigCode::kt_segment: return QueryCode::kt_segment;
    case ConfigCode::zero_offset: return QueryCode::zero_offset;
    default: return {};
  }
}
}  // namespace

ControlWorker::ControlWorker(RuntimeConfig config)
    : config_(std::move(config)), period_(1'000'000'000 / config_.frequency), guard_(config_.limits),
      protocol_(config_.format),
      bus_(std::make_unique<SocketCanTransport>(SocketCanOptions{config_.interface, config_.format, {}, true})),
      id_(config_.id), latest_(state_message(id_)) {
  guard_.connect();
  runner_ = std::thread([this] { run(); });
}

ControlWorker::~ControlWorker() {
  shutdown_.store(true);
  if (runner_.joinable()) runner_.join();
}

bool ControlWorker::submit_job(Job job) {
  const std::lock_guard<std::mutex> hold(inbox_);
  if (job_ || shutdown_.load()) return false;
  job_ = std::move(job);
  return true;
}

void ControlWorker::submit_target(Target target) {
  const std::lock_guard<std::mutex> hold(inbox_);
  ++received_commands_;
  if (target_) ++overwritten_commands_;
  target_ = std::move(target);
}

bool ControlWorker::submit_goal(Goal goal) {
  const std::lock_guard<std::mutex> hold(inbox_);
  if (goal_ || shutdown_.load()) return false;
  goal_ = std::move(goal);
  return true;
}

void ControlWorker::enter(std::optional<ReceivedFrame> frame) {
  if (!frame) return;
  ++counts_.rx;
  if (frame->socket_dropped) counts_.socket_drops = *frame->socket_dropped;
  if (config_.raw && !raw_.push({false, raw_message(frame->frame, frame->stamp, &*frame)})) ++counts_.overflow;
}

bool ControlWorker::send_frame(const Encoded& packet, TimePoint before, SendStatus& status) {
  status = bus_->send(packet.frame);
  if (status != SendStatus::accepted) return false;
  ++counts_.tx;
  if (packet.reply.kind != ReplyKind::none) ++counts_.expected;
  if (config_.raw && !raw_.push({true, raw_message(packet.frame, before)})) ++counts_.overflow;
  return true;
}

TransactionEvent ControlWorker::transact(const Encoded& packet, bool priming) {
  // Management operations execute only when no motion is active.
  const auto admission_limit = Clock::now() + 650ms;
  TimePoint deadline;
  while (!shutdown_.load() && !stop_.load()) {
    const auto before = Clock::now();
    if (before >= admission_limit) return {S::timed_out, {}, {}};
    deadline = before + 75ms;
    const auto accepted = transactions_.begin(packet, before, deadline);
    if (accepted == Begin::wait_after_write || accepted == Begin::quarantined) {
      std::this_thread::sleep_for(1ms);
      continue;
    }
    if (accepted != Begin::accepted) return {S::cancelled, {}, {}};
    SendStatus status;
    if (!send_frame(packet, before, status)) return transactions_.cancel();
    if (!transactions_.pending()) return {S::completed, {}, {}};
    break;
  }
  while (transactions_.pending() && !shutdown_.load() && !stop_.load()) {
    const auto time = Clock::now();
    if (time >= deadline) {
      ++counts_.timeouts;
      return transactions_.poll(time);
    }
    auto rx = bus_->receive_for(std::min(Clock::duration(3ms), deadline - time));
    if (rx.status == ReceiveStatus::disconnected || rx.status == ReceiveStatus::io_error) return transactions_.cancel();
    if (!rx.sample) continue;
    enter(rx.sample);
    auto decoded = protocol_.decode(*rx.sample, profile_.protocol);
    if (rx.sample->frame.error) return transactions_.cancel();
    if (const auto* issue = std::get_if<CodecIssue>(&decoded.feedback)) {
      (void)issue;
      ++counts_.invalid;
    }
    if (priming && rx.sample->origin == Origin::remote && rx.sample->frame.id == id_ && rx.sample->stamp <= Clock::now()) {
      if (const auto* f = std::get_if<Telemetry>(&decoded.feedback); f && f->type == 1) prime_ = rx.sample;
    }
    auto event = transactions_.receive(decoded, Clock::now());
    if (event.state == S::completed) ++counts_.matched;
    if (event.reply && event.state == S::rejected) {
      const auto* reply = std::get_if<QueryReply>(&event.reply->feedback);
      if (reply && reply->error != MotorError::none) {
        latest_.error_valid = true;
        latest_.motor_error = static_cast<std::uint8_t>(reply->error);
        guard_.motor_fault();
      }
    }
    if (event.state == S::timed_out) ++counts_.timeouts;
    if (event.state == S::ignored) ++counts_.ignored;
    if (!transactions_.pending()) return event;
  }
  return transactions_.cancel();
}

JobResult ControlWorker::feedback_result(const TransactionEvent& event) {
  JobResult result;
  result.success = event.state == S::completed;
  result.acknowledged = event.reply.has_value();
  result.detail = result.success ? "ok" : (event.state == S::timed_out ? "reply timeout or signature quarantine"
                                         : (result.acknowledged ? "motor rejected request" : "transaction cancelled"));
  if (!event.reply) return result;
  const auto& response = *event.reply;
  result.raw = raw_message(response.raw.frame, response.raw.stamp, &response.raw);
  if (const auto* data = std::get_if<ManagementReply>(&response.feedback)) result.id = data->id.value_or(0);
  if (const auto* data = std::get_if<QueryReply>(&response.feedback)) result.value = typed_value(data->value);
  if (const auto* data = std::get_if<ConfigEcho>(&response.feedback)) result.value = typed_value(data->value);
  if (const auto* data = std::get_if<KtReply>(&response.feedback)) {
    result.value.kind = msg::ParameterValue::KT_GROUP;
    result.value.group = data->group.value_or(0);
    result.value.kt_raw_header = data->raw_header;
    result.value.values = data->values;
  }
  if (event.uuid) {
    result.value.kind = msg::ParameterValue::UUID;
    result.value.bytes = *event.uuid;
  }
  return result;
}

bool ControlWorker::fresh_prime(TimePoint now) const {
  return prime_ && prime_->stamp <= now && now - prime_->stamp < config_.limits.feedback_age - 25ms;
}

bool ControlWorker::refresh_prime() {
  prime_.reset();
  const auto result = transact(protocol_.query(id_, QueryCode::version), true);
  if (result.state != S::completed || !fresh_prime(Clock::now()) || !profile_.ready()) return false;
  if (!guard_.set_verified_motor(profile_, prime_->stamp)) return false;
  guard_.observe(*prime_, Clock::now());
  const auto decoded = protocol_.decode(*prime_, profile_.protocol);
  if (const auto* f = std::get_if<Telemetry>(&decoded.feedback)) {
    latest_ = state_message(id_, f, &*prime_);
    full_feedback_ = prime_->stamp;
  }
  return guard_.state() == SafetyState::unarmed || guard_.state() == SafetyState::read_only;
}

bool ControlWorker::verify() {
  profile_ = {};
  prime_.reset();
  const auto discovered = feedback_result(transact(protocol_.discover()));
  if (!discovered.success || discovered.id != id_) return false;
  const auto query = [&](QueryCode code) {
    const auto result = transact(protocol_.query(id_, code));
    if (result.state != S::completed || !result.reply) throw std::runtime_error("profile query failed");
    return std::get<QueryReply>(result.reply->feedback).value;
  };
  const auto range = [&](QueryCode code) {
    const auto value = std::get<Pair>(query(code));
    return Range{value.first, value.second};
  };
  VerifiedMotor verified;
  verified.id = id_;
  verified.format = config_.format;
  verified.brake_present = true;  // The isolated ENCOS simulator exposes both brake commands.
  verified.stop_strategy_confirmed = true;  // P1 simulated watchdog, not a hardware claim.
  try {
    verified.timeout_ms = static_cast<std::uint16_t>(std::get<Scalar>(query(QueryCode::timeout)).value);
    verified.brake_state = static_cast<BrakeState>(std::get<Enumeration>(query(QueryCode::brake)).raw);
    verified.protocol.kt_nm_per_a = std::get<Scalar>(query(QueryCode::kt)).value;
    verified.protocol.query35_raw = std::get<Enumeration>(query(QueryCode::kt_calibration)).raw;
    verified.protocol.mixed = MixedRanges{range(QueryCode::kp_range), range(QueryCode::kd_range),
        range(QueryCode::position_range), range(QueryCode::velocity_range), range(QueryCode::torque_range)};
    verified.protocol.feedback1 = Feedback1Ranges{verified.protocol.mixed->position_rad,
        verified.protocol.mixed->velocity_rad_s, range(QueryCode::current_range)};
    const auto version = transact(protocol_.query(id_, QueryCode::version), true);
    if (version.state != S::completed || !version.reply) return false;
    verified.version = std::get<Version>(std::get<QueryReply>(version.reply->feedback).value);
  } catch (const std::exception&) {
    return false;
  }
  if (!verified.ready() || !fresh_prime(Clock::now())) return false;
  profile_ = verified;
  if (!guard_.set_verified_motor(profile_, prime_->stamp)) return false;
  guard_.observe(*prime_, Clock::now());
  const auto decoded = protocol_.decode(*prime_, profile_.protocol);
  if (const auto* f = std::get_if<Telemetry>(&decoded.feedback)) {
    latest_ = state_message(id_, f, &*prime_);
    full_feedback_ = prime_->stamp;
  }
  return guard_.state() == SafetyState::unarmed || guard_.state() == SafetyState::read_only;
}

bool ControlWorker::confirm_virtual_stop() {
  const auto response = transact(protocol_.query(id_, QueryCode::velocity));
  if (!response.reply) return false;
  const auto* q = std::get_if<QueryReply>(&response.reply->feedback);
  const auto* speed = q ? std::get_if<Scalar>(&q->value) : nullptr;
  return speed && std::abs(speed->value) < 1e-6 && guard_.confirm_stop(true);
}

JobResult ControlWorker::operate(const Job& job) {
  JobResult result;
  if (job.kind == JobKind::reset) {
    if (!job.confirm || (guard_.state() != SafetyState::fault && guard_.state() != SafetyState::stopping)) {
      result.detail = "reset requires operator confirmation and a fault";
      return result;
    }
    if (stop_after_ && Clock::now() < *stop_after_) {
      result.detail = "waiting for simulator watchdog";
      return result;
    }
    if (!connected_) {
      try {
        bus_ = std::make_unique<SocketCanTransport>(SocketCanOptions{config_.interface, config_.format, {}, true});
        connected_ = true;
        guard_.connect();
      } catch (const std::system_error&) {
        result.detail = "vcan is unavailable";
        return result;
      }
    }
    if (guard_.state() == SafetyState::stopping && !confirm_virtual_stop()) {
      result.detail = "simulated stop not verified";
      return result;
    }
    if (guard_.fault() == SafetyFault::motor) {
      const auto status = transact(protocol_.query(id_, QueryCode::velocity));
      if (status.state != S::completed) {
        result.detail = "reported motor fault is still present";
        return result;
      }
    }
    result.success = guard_.reset_fault(true);
    stop_after_.reset();
    profile_ = {};
    latest_ = state_message(id_);
    detail_ = result.detail = result.success ? "reset; new authorization required" : "reset failed";
    return result;
  }
  if (moving(guard_.state()) || guard_.state() == SafetyState::stopping ||
      guard_.state() == SafetyState::fault || authority_ == Authority::topic || authority_ == Authority::action) {
    result.detail = "management is unavailable during motion, authorization, or a latched fault";
    return result;
  }
  if (!connected_) { result.detail = "vcan unavailable"; return result; }
  if (job.kind == JobKind::arm) {
    const auto requested = static_cast<Authority>(job.code);
    if (!job.confirm || (requested != Authority::read_only && requested != Authority::topic &&
                         requested != Authority::action) ||
        (requested != Authority::read_only && job.owner == 0)) {
      result.detail = "invalid explicit authorization";
      return result;
    }
    if (requested == Authority::read_only) {
      result.success = guard_.state() == SafetyState::read_only || guard_.authorize_read_only(true);
    } else {
      result.success = verify();
    }
    if (result.success) {
      authority_ = requested;
      owner_ = job.owner;
      detail_ = "software authorized; waiting for a new target";
      result.detail = detail_;
    } else {
      result.detail = "simulator profile or fresh bootstrap feedback not verified";
      detail_ = result.detail;
    }
    return result;
  }
  try {
    if (job.kind == JobKind::discover) return feedback_result(transact(protocol_.discover()));
    if (job.kind == JobKind::query) {
      return feedback_result(transact(protocol_.query(id_, static_cast<QueryCode>(job.code), job.group)));
    }
    if (!config_.allow_writes || !job.confirm) {
      result.detail = "virtual configuration requires startup opt-in and per-operation confirmation";
      return result;
    }
    const auto permission = WritePermission::offline;
    profile_ = {};
    switch (job.kind) {
      case JobKind::configure: {
        const auto code = static_cast<ConfigCode>(job.code);
        if (code == ConfigCode::communication || code == ConfigCode::acceleration) {
          result.detail = "blocked by documented protocol ambiguity";
          return result;
        }
        if (code == ConfigCode::timeout && job.value.values[0] < 1) {
          result.detail = "disabling the timeout is prohibited";
          return result;
        }
        const auto packet = protocol_.configure(id_, code, to_config(code, job.value), permission);
        result = feedback_result(transact(packet));
        if (!result.success) return result;
        const auto read = readback_query(code);
        if (!read) {
          result.success = false;
          result.detail = "configuration acknowledged; query enum mapping isolated";
          return result;
        }
        const auto group = code == ConfigCode::kt_segment ? job.value.group : 0;
        const auto verified = feedback_result(transact(protocol_.query(id_, *read, group)));
        const auto length = packet.frame.length - 2;
        result.success = verified.success && verified.raw.length == packet.frame.length &&
            std::equal(packet.frame.data.begin() + 2, packet.frame.data.begin() + 2 + length,
                       verified.raw.data.begin() + 2);
        result.value = verified.value;
        result.detail = result.success ? "echo and readback verified" : "write acknowledged but readback failed";
        return result;
      }
      case JobKind::zero: {
        const auto packet = protocol_.zero(id_, job.option ? std::optional<double>(job.number) : std::nullopt, permission);
        result = feedback_result(transact(packet));
        if (!result.success) return result;
        const auto read = feedback_result(transact(protocol_.query(id_, QueryCode::zero_offset)));
        result.success = read.success && read.raw.length == 4 &&
            read.raw.data[2] == packet.frame.data[4] && read.raw.data[3] == packet.frame.data[5];
        result.detail = result.success ? "zero offset verified" : "zero acknowledged but readback failed";
        return result;
      }
      case JobKind::set_id:
      case JobKind::reset_id: {
        const bool reset = job.kind == JobKind::reset_id;
        const auto packet = reset ? protocol_.reset_id(permission) : protocol_.set_id(id_, job.id, permission);
        result = feedback_result(transact(packet));
        if (!result.success) return result;
        const auto new_id = reset ? 1 : job.id;
        const auto read = feedback_result(transact(protocol_.discover()));
        result.success = read.success && read.id == new_id;
        if (result.success) { id_ = read.id; latest_ = state_message(id_); }
        result.detail = result.success ? "new ID verified" : "ID changed but readback failed";
        return result;
      }
      case JobKind::brake: {
        const auto target = job.option ? BrakeState::released : BrakeState::engaged;
        auto packet = job.secondary ?
            protocol_.control(id_, Drive{DriveMode::brake, job.option ? 1. : 0., Ack::none}) :
            protocol_.brake(id_, target, permission);
        result = feedback_result(transact(packet));
        if (!result.success) return result;
        const auto read = feedback_result(transact(protocol_.query(id_, QueryCode::brake)));
        result.success = read.success && read.value.raw_enum == static_cast<std::uint8_t>(target);
        result.detail = result.success ? "brake state verified" : "brake acknowledged but readback failed";
        if (result.success) {
          latest_.brake_valid = true;
          latest_.brake_released = job.option;
        }
        return result;
      }
      default: break;
    }
  } catch (const CodecError& error) {
    switch (error.issue) {
      case CodecIssue::unsupported: result.detail = "unsupported code or enum"; break;
      case CodecIssue::ambiguous: result.detail = "blocked by documented protocol ambiguity"; break;
      default: result.detail = "invalid request fields"; break;
    }
  }
  return result;
}

void ControlWorker::accept_goal(Goal goal, TimePoint now) {
  if (authority_ != Authority::action || goal.command.owner != owner_ || active_goal_ ||
      guard_.state() == SafetyState::fault || guard_.state() == SafetyState::stopping) {
    if (!actions_.push({goal.token, 0, 0, 0, 0, ActionUpdate::aborted, "action does not own the lease"})) ++counts_.overflow;
    return;
  }
  active_goal_ = RunningGoal{std::move(goal), now, now};
}

void ControlWorker::finish_action(ActionUpdate::Stage stage, const char* detail, TimePoint now) {
  if (!active_goal_) return;
  const auto& active = *active_goal_;
  const double elapsed = std::chrono::duration<double>(now - active.started).count();
  if (!actions_.push({active.goal.token, active.samples, elapsed,
                      std::clamp(elapsed / active.goal.wave.duration, 0., 1.), active.target, stage, detail})) ++counts_.overflow;
  active_goal_.reset();
}

void ControlWorker::stopping(const char* detail, bool intentional, TimePoint now) {
  authority_ = Authority::none;
  owner_ = 0;
  expiry_.reset();
  current_target_.reset();
  const auto fault = guard_.fault();
  intentional_stop_ = intentional && (fault == SafetyFault::none || fault == SafetyFault::stopped);
  if (!intentional || intentional_stop_) detail_ = detail;
  if (pending_) motion_reply(transactions_.cancel(), now);
  if (moving(guard_.state())) guard_.stop();
  if (guard_.state() == SafetyState::stopping && !stop_after_) {
    stop_after_ = now + Milliseconds(profile_.timeout_ms) + 10ms;
    stop_checked_ = false;
  }
  if (!stop_after_) profile_ = {};
  if (active_goal_) finish_action(ActionUpdate::aborted, detail, now);
}

void ControlWorker::motion(Target target, TimePoint scheduled, TimePoint now) {
  if (target.command.owner != owner_ || target.command.motor_id != id_ ||
      (authority_ != Authority::topic && authority_ != Authority::action)) {
    ++counts_.dropped;
    return;
  }
  if (!target.time_valid || target.expires <= now) {
    ++counts_.expired;
    return;
  }
  if (!moving(guard_.state())) {
    if (!fresh_prime(now) && !refresh_prime()) {
      guard_.bus_fault();
      stopping("fresh simulator feedback unavailable", false, Clock::now());
      return;
    }
    now = Clock::now();
    if (target.expires <= now) {
      ++counts_.expired;
      return;
    }
    if (!guard_.arm(owner_, true, now)) {
      guard_.bus_fault();
      stopping("arming refused by safety checks", false, now);
      return;
    }
  }
  std::uint8_t applied = target.command.feedback;
  if (target.command.mode == M::MIXED) applied = 1;
  if (target.command.mode != M::ELECTROMAGNETIC_BRAKE &&
      (!full_feedback_ || now - *full_feedback_ >= Milliseconds(config_.safety_feedback_ms))) applied = 1;
  std::optional<Encoded> packet;
  try {
    packet = guard_.submit(owner_, to_control(target.command, applied), now, now);
  } catch (const CodecError&) {
    guard_.invalid_command();
  }
  if (!packet) {
    stopping("command rejected by safety checks", false, now);
    return;
  }
  expiry_ = target.expires;
  detail_ = "virtual motion active";
  msg::ControlSample sample;
  sample.command = std::move(target.command);
  sample.applied_ack = applied;
  sample.command_generated_mono_ns = mono(target.generated);
  sample.command_received_mono_ns = mono(target.received);
  sample.ros_command_received = target.ros_received;
  sample.ros_command_received_valid = target.ros_received_valid;
  sample.scheduled_mono_ns = mono(scheduled);
  sample.cycle_started_mono_ns = mono(now);
  sample.previous_cycle_interval_ns = last_cycle_ ? mono(now) - mono(*last_cycle_) : 0;
  sample.skipped_cycles = counts_.skipped;
  sample.requested_reply = packet->reply.kind != ReplyKind::none;
  const auto before = Clock::now();
  if (before >= target.expires) {
    ++counts_.expired;
    guard_.command_expired();
    stopping("source command expired before CAN send", false, before);
    return;
  }
  if (stop_.load() || (active_goal_ && cancel_.load() == active_goal_->goal.token)) {
    return;
  }
  sample.sequence = ++sequence_;
  last_cycle_ = now;
  sample.tx_call_before_mono_ns = mono(before);
  const auto deadline = before + 20ms;
  const auto begin = transactions_.begin(*packet, before, deadline);
  SendStatus status = SendStatus::io_error;
  const bool sent = begin == Begin::accepted && send_frame(*packet, before, status);
  sample.send_status = static_cast<std::uint8_t>(status);
  sample.tx_call_after_mono_ns = mono(Clock::now());
  pending_ = std::move(sample);
  if (!sent) {
    motion_reply(transactions_.cancel(), Clock::now());
    guard_.bus_fault();
    stopping("send failed or reply signature quarantined", false, Clock::now());
    return;
  }
  if (active_goal_) ++active_goal_->samples;
  if (!transactions_.pending()) {
    motion_reply({S::completed, {}, {}}, Clock::now());
    return;
  }
  expected_reply_ = deadline;
}

void ControlWorker::motion_reply(TransactionEvent event, TimePoint now) {
  if (!pending_) return;
  auto& sample = *pending_;
  sample.transaction_state = static_cast<std::uint8_t>(event.state);
  sample.matched = event.state == S::completed && event.reply.has_value();
  sample.rx_parsed_mono_ns = event.reply ? mono(now) : 0;
  sample.feedback = state_message(id_);
  if (event.reply) {
    const auto& feedback = *event.reply;
    if (const auto* telem = std::get_if<Telemetry>(&feedback.feedback)) {
      guard_.observe(feedback.raw, now);
      sample.feedback = state_message(id_, telem, &feedback.raw);
      latest_ = sample.feedback;
      if (telem->type == 1) full_feedback_ = feedback.raw.stamp;
    } else if (const auto* brake = std::get_if<BrakeReply>(&feedback.feedback)) {
      sample.feedback = state_message(id_, nullptr, &feedback.raw);
      sample.feedback.feedback_type = 6;
      sample.feedback.brake_valid = true;
      sample.feedback.brake_released = brake->state == BrakeState::released;
      sample.feedback.error_valid = brake->error.has_value();
      sample.feedback.motor_error = static_cast<std::uint8_t>(brake->error.value_or(MotorError::none));
      latest_ = sample.feedback;
      if (brake->error && *brake->error != MotorError::none) guard_.observe(feedback.raw, now);
    }
  }
  sample.feedback.connected = connected_;
  sample.feedback.profile_verified = profile_.ready();
  sample.feedback.safety_state = static_cast<std::uint8_t>(guard_.state());
  sample.feedback.authority = static_cast<std::uint8_t>(authority_);
  sample.feedback.owner = owner_;
  if (sample.matched) ++counts_.matched;
  if (event.state == S::timed_out) ++counts_.timeouts;
  if (mono(now) - sample.scheduled_mono_ns > period_.count()) ++counts_.overruns;
  if (!samples_.push(std::move(sample))) ++counts_.overflow;
  pending_.reset();
  expected_reply_.reset();
}

void ControlWorker::consume(const ReceivedFrame& frame, TimePoint now) {
  enter(frame);
  const auto decode = protocol_.decode(frame, profile_.protocol);
  if (const auto* issue = std::get_if<CodecIssue>(&decode.feedback)) {
    ++counts_.invalid;
    if (*issue == CodecIssue::nonfinite && moving(guard_.state()) &&
        frame.frame.id == id_ && frame.origin == Origin::remote) {
      guard_.observe(frame, now);
      stopping("non-finite feedback", false, now);
      return;
    }
  }
  if (frame.frame.error) {
    guard_.bus_fault();
    stopping("SocketCAN error frame", false, now);
    return;
  }
  if (!pending_) {
    ++counts_.ignored;
    return;
  }
  auto event = transactions_.receive(decode, now);
  if (event.state == S::ignored) ++counts_.ignored;
  if (!transactions_.pending()) {
    const auto state = event.state;
    const auto brake = event.reply && std::holds_alternative<BrakeReply>(event.reply->feedback);
    motion_reply(std::move(event), now);
    if (state != S::completed || !moving(guard_.state())) {
      if (moving(guard_.state())) guard_.bus_fault();
      stopping("feedback timeout, rejected response or motor fault", false, now);
    } else if (brake) {
      stopping("brake command completed; new authorization required", true, now);
    }
  }
}

void ControlWorker::publish_snapshot(TimePoint now) {
  auto state = latest_;
  if (state.rx_mono_ns && mono(now) - state.rx_mono_ns >
      std::chrono::duration_cast<std::chrono::nanoseconds>(config_.limits.feedback_age).count()) {
    const auto brake_valid = state.brake_valid;
    const auto brake_released = state.brake_released;
    const auto error_valid = state.error_valid;
    const auto motor_error = state.motor_error;
    state = state_message(id_);
    state.brake_valid = brake_valid;
    state.brake_released = brake_released;
    if (guard_.state() == SafetyState::stopping || guard_.state() == SafetyState::fault) {
      state.error_valid = error_valid;
      state.motor_error = motor_error;
    }
  }
  if (!state.brake_valid && profile_.brake_state) {
    state.brake_valid = true;
    state.brake_released = profile_.brake_state == BrakeState::released;
  }
  state.connected = connected_;
  state.profile_verified = profile_.ready();
  state.owner = owner_;
  state.authority = static_cast<std::uint8_t>(authority_);
  state.safety_state = static_cast<std::uint8_t>(guard_.state());
  counts_.command_received = received_commands_.load();
  auto counts = counts_;
  counts.dropped += overwritten_commands_.load() + foreign_commands_.load();
  counts.expired += expired_commands_.load();
  if (!status_.push({profile_message(profile_), state, counts, now, guard_.fault(), detail_})) ++counts_.overflow;
}

void ControlWorker::cycle(TimePoint scheduled, TimePoint now) {
  if (stop_.exchange(false)) stopping("stop requested; virtual watchdog pending", true, now);
  if (active_goal_ && cancel_.load() == active_goal_->goal.token) {
    finish_action(ActionUpdate::cancelled, "cancelled; motion sending stopped", now);
    stopping("action cancelled; virtual watchdog pending", true, now);
  }
  if (expected_reply_ && now >= *expected_reply_) {
    motion_reply(transactions_.poll(now), now);
    guard_.bus_fault();
    stopping("control reply deadline elapsed", false, now);
  }
  if (expiry_ && now >= *expiry_) guard_.command_expired();
  if (moving(guard_.state())) guard_.tick(now);
  if (guard_.state() == SafetyState::stopping && !stop_after_) stopping("safety watchdog latched", false, now);

  if (stop_after_ && now >= *stop_after_ && !stop_checked_ && connected_) {
    stop_checked_ = true;
    if (confirm_virtual_stop()) {
      if (intentional_stop_) guard_.reset_fault(true);
      profile_ = {};
      prime_.reset();
      stop_after_.reset();
      detail_ = intentional_stop_ ? "virtual stop confirmed; new authorization required" : "virtual stop confirmed; fault latched";
    } else {
      detail_ = "virtual stop not verified; manual reset may retry";
    }
  }

  std::optional<Job> job;
  std::optional<Goal> goal;
  std::optional<Target> target;
  {
    const std::unique_lock<std::mutex> lock(inbox_, std::try_to_lock);
    if (lock.owns_lock()) {
      job.swap(job_);
      goal.swap(goal_);
      if (!pending_ || authority_ != Authority::topic) target.swap(target_);
    }
  }
  if (job) {
    auto result = operate(*job);
    if (!job_results_.push(result)) ++counts_.overflow;
    publish_snapshot(Clock::now());
    return;  // Service work never uses the same slot as an active control send.
  }
  if (goal) accept_goal(std::move(*goal), now);
  if (authority_ == Authority::topic) {
    if (target && target->command.owner == owner_ && target->command.motor_id == id_) {
      if (target->time_valid && target->expires > now) {
        current_target_ = std::move(target);
      } else {
        ++counts_.expired;
      }
    } else if (target) {
      ++counts_.dropped;
    }
    if (current_target_ && !pending_) {
      if (current_target_->expires <= now) {
        current_target_.reset();
      } else {
        // Sample and hold the last authorized target only through its original deadline.
        // The packet gets a fresh execution time, while the sample retains its source times.
        motion(*current_target_, scheduled, now);
      }
    }
  } else if (target) {
    ++counts_.dropped;
  }

  if (authority_ == Authority::action && active_goal_) {
    auto& active = *active_goal_;
    const double elapsed = std::chrono::duration<double>(now - active.started).count();
    if (elapsed >= active.goal.wave.duration) {
      finish_action(ActionUpdate::success, "finite trajectory completed", now);
      stopping("action completed; virtual watchdog pending", true, now);
      return;
    }
    if (!pending_) {
      active.target = active.goal.wave.sample(elapsed);
      auto command = active.goal.command;
      if (command.mode == M::POSITION) command.position_rad = active.target;
      else command.velocity_rad_s = active.target;
      if (now - active.updated >= 100ms) {
        if (!actions_.push({active.goal.token, active.samples, elapsed, elapsed / active.goal.wave.duration,
                            active.target})) ++counts_.overflow;
        active.updated = now;
      }
      const auto ttl = config_.limits.command_age.count();
      command.valid_for.sec = static_cast<std::int32_t>(ttl / 1000);
      command.valid_for.nanosec = static_cast<std::uint32_t>((ttl % 1000) * 1000000);
      motion({std::move(command), now, now, now + config_.limits.command_age,
              builtin_interfaces::msg::Time(), false, true}, scheduled, now);
    }
  }
}

void ControlWorker::run() noexcept {
  try {
    auto scheduled = Clock::now();
    while (!shutdown_.load()) {
      auto now = Clock::now();
      if (now >= scheduled) {
        const auto missed = (now - scheduled) / period_;
        counts_.skipped += static_cast<std::uint64_t>(missed);
        scheduled += period_ * missed;
        cycle(scheduled, now);
        scheduled += period_;
        now = Clock::now();
      }
      if (!status_after_ || now >= *status_after_) {
        publish_snapshot(now);
        status_after_ = now + 50ms;
      }
      if (now >= scheduled) continue;
      if (!connected_) {
        std::this_thread::sleep_until(scheduled);
        continue;
      }
      const auto read = bus_->receive_for(std::min(Clock::duration(1ms), scheduled - now));
      if (read.status == ReceiveStatus::disconnected || read.status == ReceiveStatus::io_error) {
        connected_ = false;
        guard_.bus_fault();
        guard_.disconnect();
        stopping("vcan disconnected; explicit reset required", false, Clock::now());
      }
      if (read.sample) consume(*read.sample, Clock::now());
    }
  } catch (...) {
    connected_ = false;
    guard_.bus_fault();
    detail_ = "worker exception; stopped";
    finish_action(ActionUpdate::aborted, detail_, Clock::now());
    publish_snapshot(Clock::now());
    shutdown_.store(true);
  }
}
}  // namespace encos::driver
