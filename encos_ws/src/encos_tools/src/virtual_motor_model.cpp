#include "encos_tools/virtual_motor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace encos::tools {
namespace {

using namespace std::chrono_literals;
using Bytes = std::array<std::uint8_t, 4>;
constexpr std::uint16_t raw_acceleration = 2000;
constexpr std::array<std::uint8_t, 12> uuid{0x45, 0x4e, 0x43, 0x4f, 0x53, 2, 0, 1, 2, 3, 4, 5};

double field(const std::uint8_t* p, Field spec) {
  return (spec.signed_value ? wire::read_i16(p) : wire::read_u16(p)) / spec.scale;
}
double expand(std::uint32_t value, std::uint32_t maximum, Range range) {
  return range.min + (static_cast<double>(value) / maximum) * (range.max - range.min);
}
std::uint32_t compress(double value, std::uint32_t maximum, Range range) {
  const auto unit = (std::clamp(value, range.min, range.max) - range.min) / (range.max - range.min);
  return static_cast<std::uint32_t>(std::lround(unit * maximum));
}
std::uint8_t temperature(double celsius) {
  return static_cast<std::uint8_t>(std::lround(std::clamp(celsius * 2 + 50, 0.0, 255.0)));
}
std::uint16_t signed100(double value) {
  return static_cast<std::uint16_t>(std::lround(std::clamp(value, -327.68, 327.67) * 100));
}

bool same(const CanFrame& a, const CanFrame& b) {
  return a.length == b.length && std::equal(a.data.begin(), a.data.begin() + a.length, b.data.begin());
}

}  // namespace

VirtualMotor::VirtualMotor(ModelOptions options) : options_(options) {
  if (!valid_motor_id(options.id) || !valid_wire_format(options.format) ||
      static_cast<unsigned>(options.error) > 7 || !std::isfinite(options.motor_c)) {
    throw std::invalid_argument("invalid virtual motor settings");
  }
  state_.id = options.id;
  state_.error = options.error;
  state_.motor_c = options.motor_c;
  state_.mos_c = 27;
  const Protocol codec(options.format);
  const auto init = [&](ConfigCode code, const ConfigValue& value) {
    const auto encoded = codec.configure(state_.id, code, value, WritePermission::offline);
    auto& payload = settings_[static_cast<unsigned>(code)];
    std::copy_n(encoded.frame.data.begin() + 2, encoded.frame.length - 2, payload.begin());
  };
  // Synthetic values are an emulator fixture, independent of the motor's datasheet.
  init(ConfigCode::kt, Scalar{2.5});
  init(ConfigCode::kp_range, Pair{0, 500});
  init(ConfigCode::kd_range, Pair{0, 5});
  init(ConfigCode::position_range, Pair{-12.5, 12.5});
  init(ConfigCode::velocity_range, Pair{-18, 18});
  init(ConfigCode::torque_range, Pair{-30, 30});
  init(ConfigCode::current_range, Pair{-90, 90});
  init(ConfigCode::timeout, Scalar{500});
  init(ConfigCode::current_pi, Pair{0.01, 50});
  init(ConfigCode::velocity_pi, Pair{0.006, 0.1});
  init(ConfigCode::position_pd, Pair{0.006, 0.00004});
  init(ConfigCode::kt_calibration, Enumeration{0});
  init(ConfigCode::zero_offset, Scalar{0});
  for (std::uint8_t group = 1; group <= 10; ++group) {
    const auto request = codec.configure(state_.id, ConfigCode::kt_segment,
        KtWrite{group, {2.5, 2.6, 2.7, 2.8}}, WritePermission::offline);
    std::copy_n(request.frame.data.begin() + 2, 6, kt_[group - 1].begin());
  }
  refresh_ranges();
}

CanFrame VirtualMotor::make(std::initializer_list<std::uint8_t> bytes, std::uint16_t id) const {
  CanFrame frame;
  frame.id = id;
  frame.brs = options_.format == WireFormat::fd_brs;
  frame.kind = frame.brs ? FrameKind::fd : FrameKind::classic;
  frame.length = static_cast<std::uint8_t>(bytes.size());
  std::copy(bytes.begin(), bytes.end(), frame.data.begin());
  return frame;
}

void VirtualMotor::refresh_ranges() {
  const auto interval = [&](ConfigCode code) {
    const auto& bytes = settings_[static_cast<unsigned>(code)];
    const auto* spec = parameter(code);
    return Range{field(bytes.data(), spec->first), field(bytes.data() + 2, spec->second)};
  };
  profile_.mixed = MixedRanges{interval(ConfigCode::kp_range), interval(ConfigCode::kd_range),
      interval(ConfigCode::position_range), interval(ConfigCode::velocity_range), interval(ConfigCode::torque_range)};
  profile_.feedback1 = Feedback1Ranges{profile_.mixed->position_rad, profile_.mixed->velocity_rad_s,
                                      interval(ConfigCode::current_range)};
  profile_.kt_nm_per_a = wire::read_u16(settings_[4].data()) / 100.0;
}

void VirtualMotor::stop_motion() noexcept {
  state_.active = false;
  state_.velocity_rad_s = 0;
  state_.phase_current_a = 0;
  last_motion_.reset();
}

void VirtualMotor::set_position(double value) {
  const auto& range = profile_.feedback1->position_rad;
  state_.position_rad = std::clamp(value, range.min, range.max);
}

void VirtualMotor::advance(TimePoint now) {
  if (last_tick_ && now < *last_tick_) {
    return;
  }
  const auto timeout = std::chrono::milliseconds(wire::read_u16(settings_[11].data()));
  const auto expiry = (last_motion_ && timeout.count() > 0) ? std::optional<TimePoint>(*last_motion_ + timeout)
                                                          : std::nullopt;
  if (state_.active && last_tick_) {
    const auto until = expiry ? std::min(*expiry, now) : now;
    const auto dt = std::chrono::duration<double>(until - *last_tick_).count();
    set_position(state_.position_rad + state_.velocity_rad_s * std::max(0.0, dt));
  }
  if (expiry && now >= *expiry) {
    stop_motion();
  }
  last_tick_ = now;
}

std::vector<CanFrame> VirtualMotor::handle(const ReceivedFrame& sample, TimePoint now) {
  advance(now);
  const auto& req = sample.frame;
  if (sample.origin != Origin::remote || req.extended || req.rtr || req.error || req.esi ||
      req.length < 2 || req.length > 8 || !matches_wire_format(req, options_.format) ||
      sample.stamp > now || (quiet_until_ && now < *quiet_until_)) {
    return {};
  }
  if (req.id == management_id) {
    return management(req, now);
  }
  if (req.id != state_.id) {
    return {};
  }
  switch (req.data[0] >> 5) {
    case 6: return configure(req, now);
    case 7: return query(req);
    default: return now - sample.stamp <= 50ms ? control(req, now) : std::vector<CanFrame>{};
  }
}

std::vector<CanFrame> VirtualMotor::management(const CanFrame& req, TimePoint now) {
  const auto* data = req.data.data();
  if (req.length < 4 || data[2] != 0) {
    return {};
  }
  const auto target = wire::read_u16(data);
  const auto hi = [&] { return static_cast<std::uint8_t>(state_.id >> 8); };
  const auto lo = [&] { return static_cast<std::uint8_t>(state_.id); };
  if (req.length == 4 && target == 0xffff && data[3] == 0x82) {
    return {make({0xff, 0xff, 1, hi(), lo()}, management_id)};
  }
  const bool blocked = options_.reject_writes || state_.active || state_.error != MotorError::none;
  if (target == 0x7f7f && req.length == 6 && data[3] == 5 && wire::read_u16(data + 4) == 0x7f7f) {
    if (blocked) {
      return {};
    }
    state_.id = 1;
    quiet_until_ = now + 500ms;
    return {make({0x7f, 0x7f, 1, 5, 0x7f, 0x7f}, management_id)};
  }
  const bool zero = data[3] == 3 && (req.length == 4 || req.length == 6);
  const bool change_id = data[3] == 4 && req.length == 6;
  if (target != state_.id || (!zero && !change_id)) {
    return {};
  }
  if (blocked || (change_id && !valid_motor_id(wire::read_u16(data + 4)))) {
    return {make({hi(), lo(), 1, 0}, management_id)};
  }
  if (zero) {
    zero_offset_count_ = req.length == 6 ? wire::read_i16(data + 4) : 0;
    set_position(zero_offset_count_ / 100.0 * wire::degrees_to_rad);
  } else {
    state_.id = wire::read_u16(data + 4);
  }
  quiet_until_ = now + 500ms;
  return {make({hi(), lo(), 1, data[3]}, management_id)};
}

CanFrame VirtualMotor::telemetry(std::uint8_t type) const {
  auto frame = make({static_cast<std::uint8_t>((type << 5) | static_cast<unsigned>(state_.error)), 0, 0, 0, 0, 0, 0, 0},
                    state_.id);
  auto* data = frame.data.data();
  const auto& ranges = *profile_.feedback1;
  if (type == 1) {
    wire::write_bits(data, 8, 16, compress(state_.position_rad, 65535, ranges.position_rad));
    wire::write_bits(data, 24, 12, compress(state_.velocity_rad_s, 4095, ranges.velocity_rad_s));
    wire::write_bits(data, 36, 12, compress(state_.phase_current_a, 4095, ranges.phase_current_a));
    data[6] = temperature(state_.motor_c);
    data[7] = temperature(state_.mos_c);
  } else {
    const double quantity = type == 2 ? state_.position_rad / wire::degrees_to_rad
                                     : state_.velocity_rad_s / wire::rpm_to_rad_s;
    wire::write_bits(data, 8, 32, wire::float_bits(quantity));
    wire::write_bits(data, 40, 16, signed100(state_.phase_current_a));
    data[7] = temperature(state_.motor_c);
  }
  return frame;
}

CanFrame VirtualMotor::idle_telemetry() const { return telemetry(1); }

std::vector<CanFrame> VirtualMotor::control(const CanFrame& req, TimePoint now) {
  const auto* data = req.data.data();
  const unsigned mode = data[0] >> 5;
  const unsigned sub = (data[0] >> 2) & 7;
  unsigned ack = data[0] & 3;
  const auto brake = [&](std::uint8_t tag, unsigned raw) -> std::vector<CanFrame> {
    if (raw > 1) {
      return {};
    }
    if (state_.error == MotorError::none) {
      state_.brake = static_cast<BrakeState>(raw);
      stop_motion();
    }
    return {make({tag, static_cast<std::uint8_t>(state_.brake)}, state_.id)};
  };
  if (mode == 4 && req.length == 2 && data[0] == 0x80) {
    return brake(0xb2, data[1]);
  }
  if (mode == 3 && req.length == 3 && sub == 5) {
    return brake(static_cast<std::uint8_t>(0xc0 | static_cast<unsigned>(state_.error)), wire::read_u16(data + 1));
  }

  std::optional<double> position;
  double velocity = 0;
  double current = 0;
  const auto& ranges = *profile_.mixed;
  try {
    if (mode == 0 && req.length == 8) {
      ack = 1;
      position = expand(wire::read_bits(data, 24, 16), 65535, ranges.position_rad);
      velocity = expand(wire::read_bits(data, 40, 12), 4095, ranges.velocity_rad_s);
      if (*profile_.kt_nm_per_a > 0) {
        current = expand(wire::read_bits(data, 52, 12), 4095, ranges.torque_nm) / *profile_.kt_nm_per_a;
      }
    } else if (mode == 1 && req.length == 8) {
      ack = data[7] & 3;
      position = wire::float_value(wire::read_bits(data, 3, 32)) * wire::degrees_to_rad;
    } else if (mode == 2 && req.length == 7 && sub == 0) {
      velocity = wire::read_float(data + 1) * wire::rpm_to_rad_s;
    } else if (mode == 3 && req.length == 3 && sub <= 4) {
      const auto raw = wire::read_i16(data + 1);
      if ((sub == 2 && raw != 0) || (sub >= 3 && raw < 0)) {
        return {};
      }
      if (sub == 0) {
        current = raw / 100.0;
      }
      if (sub == 1 && *profile_.kt_nm_per_a > 0) {
        current = raw / 100.0 / *profile_.kt_nm_per_a;
      }
      velocity = current * 0.1;
    } else {
      return {};
    }
  } catch (const CodecError&) {
    return {};
  }

  if (state_.error == MotorError::none && state_.brake == BrakeState::released) {
    if (position) {
      set_position(*position);
    }
    state_.velocity_rad_s = std::clamp(velocity, ranges.velocity_rad_s.min, ranges.velocity_rad_s.max);
    const auto phase = profile_.feedback1->phase_current_a;
    state_.phase_current_a = std::clamp(current, phase.min, phase.max);
    state_.active = state_.velocity_rad_s != 0 || state_.phase_current_a != 0;
    last_motion_ = now;
  }
  return ack ? std::vector<CanFrame>{telemetry(static_cast<std::uint8_t>(ack))} : std::vector<CanFrame>{};
}

CanFrame VirtualMotor::segment(std::uint8_t group) const {
  auto frame = make({0xee, group, 0, 0, 0, 0, 0, 0}, state_.id);
  std::copy(kt_[group - 1].begin(), kt_[group - 1].end(), frame.data.begin() + 2);
  return frame;
}

std::vector<CanFrame> VirtualMotor::configure(const CanFrame& req, TimePoint now) {
  const auto* data = req.data.data();
  const auto code = static_cast<ConfigCode>(data[1]);
  const auto* spec = parameter(code);
  if (!spec || code == ConfigCode::acceleration || req.length != 2 + payload_size(spec->payload)) {
    return {};
  }
  ConfigValue value = Scalar{};
  switch (spec->payload) {
    case Payload::scalar16: value = Scalar{field(data + 2, spec->first)}; break;
    case Payload::pair16: value = Pair{field(data + 2, spec->first), field(data + 4, spec->second)}; break;
    case Payload::enumeration: value = Enumeration{data[2]}; break;
    case Payload::segment: {
      KtWrite write{static_cast<std::uint8_t>(data[0] & 31), {}};
      for (unsigned i = 0; i < write.values.size(); ++i) {
        write.values[i] = wire::read_bits(data, 16 + 12 * i, 12) / 100.0;
      }
      value = write;
      break;
    }
    default: return {};
  }
  try {
    const auto canonical = Protocol(options_.format).configure(state_.id, code, value, WritePermission::offline);
    if (!same(req, canonical.frame)) {
      return {};
    }
  } catch (const CodecError&) {
    return {};
  }
  if (options_.reject_writes || state_.active || state_.error != MotorError::none) {
    return {make({static_cast<std::uint8_t>(0x80 | static_cast<unsigned>(state_.error)), data[1], 0}, state_.id)};
  }
  if (code == ConfigCode::communication) {
    staged_format_ = data[2] == 0 ? WireFormat::classic : WireFormat::fd_brs;
    quiet_until_ = now + 500ms;
    return {};  // Both documented special ACK lengths remain isolated.
  }
  if (code == ConfigCode::kt_segment) {
    if (calibration_written_.value_or(0) != 0) {
      return {};  // EE header is only emitted with the shared mode-zero layout.
    }
    const auto group = std::get<KtWrite>(value).group;
    std::copy_n(data + 2, 6, kt_[group - 1].begin());
    return {segment(group)};
  }
  if (code == ConfigCode::kt_calibration) {
    calibration_written_ = data[2];
  }
  if (code == ConfigCode::zero_offset) {
    zero_offset_count_ = wire::read_i16(data + 2);
  }
  std::copy_n(data + 2, req.length - 2, settings_[data[1]].begin());
  refresh_ranges();
  auto reply = make({0xff, 0xfe, data[1]}, state_.id);
  std::copy_n(data + 2, req.length - 2, reply.data.begin() + 3);
  reply.length = static_cast<std::uint8_t>(req.length + 1);
  return {reply};
}

std::vector<CanFrame> VirtualMotor::query(const CanFrame& req) const {
  if (req.length != 2) {
    return {};
  }
  const auto code = static_cast<QueryCode>(req.data[1]);
  if (!parameter(code)) {
    return {};
  }
  const auto group = static_cast<std::uint8_t>(req.data[0] & 31);
  if (code == QueryCode::kt_segment) {
    return group >= 1 && group <= 10 && calibration_written_.value_or(0) == 0
        ? std::vector<CanFrame>{segment(group)} : std::vector<CanFrame>{};
  }
  if (group != 1) {
    return {};
  }
  const auto header = static_cast<std::uint8_t>(0xa0 | static_cast<unsigned>(state_.error));
  auto reply = make({header, req.data[1]}, state_.id);
  const auto scalar = [&](double physical) {
    reply.length = 6;
    wire::write_bits(reply.data.data(), 16, 32, wire::float_bits(physical));
  };
  const auto words = [&](ConfigCode config) {
    const auto* spec = parameter(config);
    const auto size = payload_size(spec->payload);
    reply.length = static_cast<std::uint8_t>(2 + size);
    std::copy_n(settings_[static_cast<unsigned>(config)].begin(), size, reply.data.begin() + 2);
  };
  switch (code) {
    case QueryCode::position: scalar(state_.position_rad / wire::degrees_to_rad); break;
    case QueryCode::velocity: scalar(state_.velocity_rad_s / wire::rpm_to_rad_s); break;
    case QueryCode::phase_current: scalar(state_.phase_current_a); break;
    case QueryCode::power: scalar(std::abs(state_.phase_current_a * state_.velocity_rad_s * *profile_.kt_nm_per_a)); break;
    case QueryCode::acceleration:
      reply.length = 4;
      wire::write_bits(reply.data.data(), 16, 16, raw_acceleration);
      break;
    case QueryCode::kt: words(ConfigCode::kt); break;
    case QueryCode::kp_range: words(ConfigCode::kp_range); break;
    case QueryCode::kd_range: words(ConfigCode::kd_range); break;
    case QueryCode::position_range: words(ConfigCode::position_range); break;
    case QueryCode::velocity_range: words(ConfigCode::velocity_range); break;
    case QueryCode::torque_range: words(ConfigCode::torque_range); break;
    case QueryCode::current_range: words(ConfigCode::current_range); break;
    case QueryCode::uuid: {
      std::vector<CanFrame> parts;
      for (std::uint8_t i = 0; i < 3; ++i) {
        auto part = make({header, req.data[1], static_cast<std::uint8_t>(i + 1), 0, 0, 0, 0}, state_.id);
        std::copy_n(uuid.begin() + 4 * i, 4, part.data.begin() + 3);
        parts.push_back(part);
      }
      return parts;
    }
    case QueryCode::version: return {make({header, req.data[1], 1, 0, 0, 1, 19, 0}, state_.id)};
    case QueryCode::timeout: words(ConfigCode::timeout); break;
    case QueryCode::current_pi: words(ConfigCode::current_pi); break;
    case QueryCode::velocity_pi: words(ConfigCode::velocity_pi); break;
    case QueryCode::position_pd: words(ConfigCode::position_pd); break;
    case QueryCode::kt_calibration:
      if (calibration_written_) return {};
      return {make({header, req.data[1], 1}, state_.id)};
    case QueryCode::brake: return {make({header, req.data[1], static_cast<std::uint8_t>(state_.brake)}, state_.id)};
    case QueryCode::zero_offset:
      reply.length = 4;
      wire::write_bits(reply.data.data(), 16, 16, static_cast<std::uint16_t>(zero_offset_count_));
      break;
    case QueryCode::encoder: {
      reply.length = 7;
      reply.data[2] = encoder_calibrated_;
      const auto count = signed100(state_.position_rad / wire::degrees_to_rad);
      wire::write_bits(reply.data.data(), 24, 16, count);
      wire::write_bits(reply.data.data(), 40, 16, count);
      break;
    }
    case QueryCode::kt_segment: return {};
  }
  return {reply};
}

void FaultQueue::enqueue(std::vector<CanFrame> frames, TimePoint now) {
  if (frames.empty()) {
    return;
  }
  ++request_count_;
  if (options_.silent || (options_.drop_every && request_count_ % options_.drop_every == 0)) {
    dropped_ += frames.size();
    return;
  }
  const bool uuid = frames.size() == 3 && frames.front().length == 7 && frames.front().data[1] == 29;
  if (uuid && options_.uuid_missing) {
    frames.erase(frames.begin() + 1);
    ++dropped_;
  }
  if (uuid && options_.uuid_reverse) {
    std::reverse(frames.begin(), frames.end());
  }
  const auto due = now + options_.delay;
  for (auto& frame : frames) {
    if (options_.wrong_id) frame.id = frame.id == 0x7fe ? 1 : (frame.id % 0x7fe) + 1;
    if (options_.short_length) frame.length = 1;
    if (options_.unknown_type) frame.data[0] = 0x00;
    for (int i = 0; i < (options_.duplicate ? 2 : 1); ++i) {
      if (queue_.size() == 256) {
        ++dropped_;
      } else {
        queue_.push_back({due, frame});
      }
    }
  }
}

std::optional<CanFrame> FaultQueue::take(TimePoint now) {
  if (queue_.empty() || queue_.front().due > now) {
    return {};
  }
  auto frame = queue_.front().frame;
  queue_.pop_front();
  return frame;
}

std::optional<TimePoint> FaultQueue::next() const {
  return queue_.empty() ? std::nullopt : std::optional<TimePoint>{queue_.front().due};
}

}  // namespace encos::tools
