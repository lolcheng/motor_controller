#include "encos_driver/ros_interfaces.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace encos::driver {
namespace {
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
}

std::int64_t mono(TimePoint time) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

Control to_control(const msg::MotorCommand& c, std::uint8_t applied_ack) {
  for (const auto value : {c.position_rad, c.velocity_rad_s, c.speed_limit_rad_s, c.current_a,
                          c.torque_nm, c.feedforward_nm, c.kp, c.kd, c.current_limit_a}) {
    if (!std::isfinite(value)) throw CodecError(CodecIssue::value, "non-finite command field");
  }
  if (c.feedback > 3 || applied_ack > 3) throw CodecError(CodecIssue::value, "invalid feedback type");
  const auto ack = static_cast<Ack>(applied_ack);
  using M = msg::MotorCommand;
  switch (c.mode) {
    case M::MIXED: return Mixed{c.kp, c.kd, c.position_rad, c.velocity_rad_s, c.feedforward_nm};
    case M::POSITION: return Position{c.position_rad, c.speed_limit_rad_s, c.current_limit_a, ack};
    case M::VELOCITY: return Velocity{c.velocity_rad_s, c.current_limit_a, ack};
    case M::CURRENT: return Drive{DriveMode::current, c.current_a, ack};
    case M::TORQUE: return Drive{DriveMode::torque, c.torque_nm, ack};
    case M::DAMPING: return Drive{DriveMode::damping, 0, ack};
    case M::DISSIPATIVE: return Drive{DriveMode::dissipative, c.current_limit_a, ack};
    case M::REGENERATIVE: return Drive{DriveMode::regenerative, c.current_limit_a, ack};
    case M::ELECTROMAGNETIC_BRAKE: return Drive{DriveMode::brake, c.release_brake ? 1. : 0., ack};
    default: throw CodecError(CodecIssue::unsupported, "unsupported control mode");
  }
}

ConfigValue to_config(ConfigCode code, const msg::ParameterValue& value) {
  const auto* spec = parameter(code);
  if (!spec) throw CodecError(CodecIssue::unsupported, "unsupported configuration code");
  using V = msg::ParameterValue;
  if (spec->payload == Payload::scalar16 && value.kind == V::SCALAR) return Scalar{value.values[0]};
  if (spec->payload == Payload::pair16 && value.kind == V::PAIR) return Pair{value.values[0], value.values[1]};
  if (spec->payload == Payload::enumeration && value.kind == V::ENUMERATION) return Enumeration{value.raw_enum};
  if (spec->payload == Payload::segment && value.kind == V::KT_GROUP) return KtWrite{value.group, value.values};
  throw CodecError(CodecIssue::value, "configuration payload does not match parameter registry");
}

msg::ParameterValue typed_value(const ParameterValue& data) {
  msg::ParameterValue result;
  result.values.fill(nan);
  std::visit([&](const auto& field) {
    using T = std::decay_t<decltype(field)>;
    using V = msg::ParameterValue;
    if constexpr (std::is_same_v<T, Scalar>) {
      result.kind = V::SCALAR; result.values[0] = field.value;
    } else if constexpr (std::is_same_v<T, Pair>) {
      result.kind = V::PAIR; result.values[0] = field.first; result.values[1] = field.second;
    } else if constexpr (std::is_same_v<T, Enumeration>) {
      result.kind = V::ENUMERATION; result.raw_enum = field.raw;
    } else if constexpr (std::is_same_v<T, Raw16>) {
      result.kind = V::RAW16; result.raw_u16 = field.value;
    } else if constexpr (std::is_same_v<T, UuidPart>) {
      result.kind = V::UUID; std::copy(field.bytes.begin(), field.bytes.end(), result.bytes.begin());
    } else if constexpr (std::is_same_v<T, Version>) {
      result.kind = V::VERSION;
      std::copy(field.hardware.begin(), field.hardware.end(), result.bytes.begin());
      std::copy(field.software.begin(), field.software.end(), result.bytes.begin() + 3);
    } else {
      result.kind = V::ENCODER; result.calibrated = field.calibrated;
      result.encoder_counts = {field.raw_count, field.calibrated_count};
      result.values[0] = field.raw_rad.value_or(nan);
      result.values[1] = field.calibrated_rad.value_or(nan);
    }
  }, data);
  return result;
}

msg::RawFrame raw_message(const CanFrame& frame, TimePoint time, const ReceivedFrame* sample) {
  msg::RawFrame result;
  result.id = frame.id;
  result.fd = frame.kind == FrameKind::fd;
  result.brs = frame.brs;
  result.esi = frame.esi;
  result.extended = frame.extended;
  result.rtr = frame.rtr;
  result.error = frame.error;
  result.length = frame.length;
  result.data = frame.data;
  result.host_mono_ns = mono(time);
  if (sample) {
    result.origin = static_cast<std::uint8_t>(sample->origin);
    if (sample->kernel_realtime) {
      result.kernel_realtime_valid = true;
      result.kernel_realtime_ns = sample->kernel_realtime->count();
    }
  }
  return result;
}

msg::MotorState state_message(std::uint16_t id, const Telemetry* f, const ReceivedFrame* sample) {
  msg::MotorState state;
  state.virtual_device = true;
  state.motor_id = id;
  state.position_rad = state.velocity_rad_s = state.phase_current_a = state.estimated_torque_nm =
      state.motor_temperature_c = state.mos_temperature_c = nan;
  if (f) {
    state.feedback_type = f->type;
    state.motor_error = static_cast<std::uint8_t>(f->error);
    state.error_valid = true;
    const std::array<const std::optional<Measurement>*, 6> from{
      &f->position_rad, &f->velocity_rad_s, &f->phase_current_a, &f->estimated_torque_nm, &f->motor_c, &f->mos_c};
    const std::array<double*, 6> to{&state.position_rad, &state.velocity_rad_s, &state.phase_current_a,
                                   &state.estimated_torque_nm, &state.motor_temperature_c, &state.mos_temperature_c};
    for (unsigned i = 0; i < from.size(); ++i) {
      if (const auto& physical = *from[i]) {
        *to[i] = physical->value;
        state.valid[i] = true;
        state.source[i] = static_cast<std::uint8_t>(physical->source) + 1;
        state.measurement_mono_ns[i] = mono(physical->host_rx);
      }
    }
  }
  if (sample) {
    state.rx_mono_ns = mono(sample->stamp);
    state.rx_userspace_mono_ns = sample->userspace_stamp ? mono(*sample->userspace_stamp) : 0;
    if (sample->kernel_realtime) {
      state.kernel_rx_valid = true;
      state.rx_kernel_realtime_ns = sample->kernel_realtime->count();
    }
  }
  return state;
}

msg::MotorProfile profile_message(const VerifiedMotor& motor) {
  msg::MotorProfile result;
  result.motor_id = motor.id;
  result.virtual_device = true;
  result.verified = motor.ready();
  result.fd_brs = motor.format == WireFormat::fd_brs;
  result.timeout_ms = motor.timeout_ms;
  result.brake_present = motor.brake_present.value_or(false);
  result.brake_released = motor.brake_state == BrakeState::released;
  result.kt_nm_per_a = motor.protocol.kt_nm_per_a.value_or(nan);
  result.query35_raw = static_cast<std::uint8_t>(motor.protocol.query35_raw.value_or(0));
  result.ranges.fill(nan);
  if (motor.version) {
    result.hardware_version = motor.version->hardware;
    result.software_version = motor.version->software;
  }
  if (motor.protocol.mixed && motor.protocol.feedback1) {
    const auto& m = *motor.protocol.mixed;
    const auto& f = *motor.protocol.feedback1;
    result.ranges = {m.kp.min, m.kp.max, m.kd.min, m.kd.max, m.position_rad.min, m.position_rad.max,
      m.velocity_rad_s.min, m.velocity_rad_s.max, m.torque_nm.min, m.torque_nm.max, f.phase_current_a.min, f.phase_current_a.max};
  }
  return result;
}
}  // namespace encos::driver
