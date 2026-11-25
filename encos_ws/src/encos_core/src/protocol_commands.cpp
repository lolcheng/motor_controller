#include "encos_core/protocol.hpp"

#include <cmath>
#include <type_traits>

namespace encos {
namespace {

constexpr auto management_delay = std::chrono::milliseconds(500);

void require_motor(std::uint16_t id) {
  if (!valid_motor_id(id)) {
    throw CodecError(CodecIssue::value, "invalid motor id");
  }
}

void require_write(WritePermission permission) {
  if (permission != WritePermission::offline) {
    throw CodecError(CodecIssue::permission, "write serialization requires explicit offline permission");
  }
}

std::uint8_t ack_byte(Ack ack) {
  const auto value = static_cast<std::uint8_t>(ack);
  if (value > 3) {
    throw CodecError(CodecIssue::value, "invalid control ACK");
  }
  return value;
}

std::uint32_t quantize(double value, const Range& range, std::uint32_t max) {
  if (!range.contains(value)) {
    throw CodecError(CodecIssue::value, "mixed control value outside verified range");
  }
  return static_cast<std::uint32_t>((value - range.min) / (range.max - range.min) * max);
}

void set_u16(CanFrame& f, unsigned byte, std::uint16_t value) {
  wire::write_bits(f.data.data(), byte * 8, 16, value);
}

void set_parameter(CanFrame& f, unsigned byte, double value, Field field) {
  set_u16(f, byte, field.signed_value ? wire::fixed(value, field.scale, -32768, 32767)
                                    : wire::fixed(value, field.scale, 0, 65535));
}

}  // namespace

CanFrame Protocol::frame(std::uint16_t id, std::uint8_t length) const {
  if (!valid_wire_format(format_)) {
    throw CodecError(CodecIssue::value, "unsupported transmit frame format");
  }
  const bool fd = format_ == WireFormat::fd_brs;
  CanFrame result;
  result.id = id;
  result.length = length;
  result.kind = fd ? FrameKind::fd : FrameKind::classic;
  result.brs = fd;
  return result;
}

Encoded Protocol::control(std::uint16_t id, const Control& command, const ProtocolProfile& profile) const {
  require_motor(id);
  return std::visit([&](const auto& cmd) -> Encoded {
    using T = std::decay_t<decltype(cmd)>;
    CanFrame f;
    std::uint8_t ack = 1;
    bool brake_reply = false;
    if constexpr (std::is_same_v<T, Mixed>) {
      if (!profile.mixed || !profile.mixed->valid()) {
        throw CodecError(CodecIssue::profile, "mixed control needs verified ranges");
      }
      const auto& ranges = *profile.mixed;
      f = frame(id, 8);
      auto* bytes = f.data.data();
      wire::write_bits(bytes, 3, 12, quantize(cmd.kp, ranges.kp, 4095));
      wire::write_bits(bytes, 15, 9, quantize(cmd.kd, ranges.kd, 511));
      wire::write_bits(bytes, 24, 16, quantize(cmd.position_rad, ranges.position_rad, 65535));
      wire::write_bits(bytes, 40, 12, quantize(cmd.velocity_rad_s, ranges.velocity_rad_s, 4095));
      wire::write_bits(bytes, 52, 12, quantize(cmd.torque_nm, ranges.torque_nm, 4095));
    } else if constexpr (std::is_same_v<T, Position>) {
      ack = ack_byte(cmd.ack);
      f = frame(id, 8);
      auto* bytes = f.data.data();
      wire::write_bits(bytes, 0, 3, 1);
      wire::write_bits(bytes, 3, 32, wire::float_bits(cmd.position_rad / wire::degrees_to_rad));
      wire::write_bits(bytes, 35, 15, wire::fixed(cmd.speed_limit_rad_s, 10 / wire::rpm_to_rad_s, 0, 32767));
      wire::write_bits(bytes, 50, 12, wire::fixed(cmd.current_limit_a, 10, 0, 4095));
      wire::write_bits(bytes, 62, 2, ack);
    } else if constexpr (std::is_same_v<T, Velocity>) {
      ack = ack_byte(cmd.ack);
      f = frame(id, 7);
      f.data[0] = static_cast<std::uint8_t>(0x40 | ack);
      wire::write_bits(f.data.data(), 8, 32, wire::float_bits(cmd.velocity_rad_s / wire::rpm_to_rad_s));
      set_u16(f, 5, wire::fixed(cmd.current_limit_a, 10, 0, 65535));
    } else {
      ack = ack_byte(cmd.ack);
      const auto mode = static_cast<std::uint8_t>(cmd.mode);
      std::uint16_t field = 0;
      switch (cmd.mode) {
        case DriveMode::current:
        case DriveMode::torque:
          field = wire::fixed(cmd.value, 100, -32768, 32767);
          break;
        case DriveMode::dissipative:
        case DriveMode::regenerative:
          field = wire::fixed(cmd.value, 100, 0, 32767);
          break;
        case DriveMode::damping:
          if (cmd.value != 0) {
            throw CodecError(CodecIssue::value, "ignored damping field must be zero");
          }
          break;
        case DriveMode::brake:
          if (cmd.value != 0 && cmd.value != 1) {
            throw CodecError(CodecIssue::value, "invalid electromagnetic brake state");
          }
          field = static_cast<std::uint16_t>(cmd.value);
          brake_reply = true;
          break;
        default:
          throw CodecError(CodecIssue::unsupported, "reserved drive mode");
      }
      f = frame(id, 3);
      f.data[0] = static_cast<std::uint8_t>(0x60 | (mode << 2) | ack);
      set_u16(f, 1, field);
    }
    if (brake_reply) {
      return {f, {ReplyKind::brake, id, 6}};
    }
    return {f, {ack == 0 ? ReplyKind::none : ReplyKind::telemetry, id, ack}};
  }, command);
}

Encoded Protocol::query(std::uint16_t id, QueryCode code, std::uint8_t group) const {
  require_motor(id);
  if (!parameter(code)) {
    throw CodecError(CodecIssue::unsupported, "reserved query code");
  }
  const bool segment = code == QueryCode::kt_segment;
  if ((segment && (group < 1 || group > 10)) || (!segment && group != 0)) {
    throw CodecError(CodecIssue::value, "invalid query group");
  }
  auto f = frame(id, 2);
  // §9.3 shows E1 for the ordinary queries. Only query 36 gives these bits semantics.
  f.data[0] = static_cast<std::uint8_t>(0xe0 | (segment ? group : 1));
  f.data[1] = static_cast<std::uint8_t>(code);
  return {f, {segment ? ReplyKind::kt_group : ReplyKind::query, id, segment ? group : f.data[1]}};
}

Encoded Protocol::configure(std::uint16_t id, ConfigCode code, const ConfigValue& value,
                           WritePermission permission) const {
  require_write(permission);
  require_motor(id);
  const auto* spec = parameter(code);
  if (!spec) {
    throw CodecError(CodecIssue::unsupported, "reserved configuration code");
  }
  if (code == ConfigCode::acceleration) {
    throw CodecError(CodecIssue::ambiguous, "acceleration ACK selector disagrees with vendor example");
  }
  auto f = frame(id, static_cast<std::uint8_t>(2 + payload_size(spec->payload)));
  f.data[0] = code == ConfigCode::communication ? 0xc1 : 0xc0;
  f.data[1] = static_cast<std::uint8_t>(code);
  if (const auto* scalar = std::get_if<Scalar>(&value); scalar && spec->payload == Payload::scalar16) {
    set_parameter(f, 2, scalar->value, spec->first);
  } else if (const auto* pair = std::get_if<Pair>(&value); pair && spec->payload == Payload::pair16) {
    set_parameter(f, 2, pair->first, spec->first);
    set_parameter(f, 4, pair->second, spec->second);
    if (spec->ordered_pair &&
        wire::read_u16(f.data.data() + 2) == wire::read_u16(f.data.data() + 4)) {
      throw CodecError(CodecIssue::value, "range endpoints quantize to the same value");
    }
    if (spec->ordered_pair && pair->first >= pair->second) {
      throw CodecError(CodecIssue::value, "range endpoints out of order");
    }
  } else if (const auto* mode = std::get_if<Enumeration>(&value); mode && spec->payload == Payload::enumeration) {
    const unsigned max = code == ConfigCode::communication ? 1 : 2;
    if (mode->raw > max) {
      throw CodecError(CodecIssue::unsupported, "reserved configuration enumeration");
    }
    f.data[2] = mode->raw;
  } else if (const auto* kt = std::get_if<KtWrite>(&value); kt && spec->payload == Payload::segment) {
    if (kt->group < 1 || kt->group > 10) {
      throw CodecError(CodecIssue::value, "invalid Kt group");
    }
    f.data[0] = static_cast<std::uint8_t>(0xc0 | kt->group);
    for (unsigned i = 0; i < kt->values.size(); ++i) {
      wire::write_bits(f.data.data(), 16 + 12 * i, 12, wire::fixed(kt->values[i], 100, 0, 4095));
    }
    return {f, {ReplyKind::kt_group, id, kt->group}};
  } else {
    throw CodecError(CodecIssue::value, "wrong configuration payload type");
  }
  return {f, {ReplyKind::config, id, f.data[1]},
          code == ConfigCode::communication ? management_delay : std::chrono::milliseconds{}};
}

Encoded Protocol::discover() const {
  auto f = frame(management_id, 4);
  f.data = {0xff, 0xff, 0x00, 0x82};
  return {f, {ReplyKind::discover, management_id}};
}

Encoded Protocol::zero(std::uint16_t id, std::optional<double> offset_rad, WritePermission permission) const {
  require_write(permission);
  require_motor(id);
  auto f = frame(management_id, offset_rad ? 6 : 4);
  set_u16(f, 0, id);
  f.data[3] = 0x03;
  if (offset_rad) {
    set_u16(f, 4, wire::fixed(*offset_rad, 100 / wire::degrees_to_rad, -32768, 32767));
  }
  return {f, {ReplyKind::zero, management_id, 0, id}, management_delay};
}

Encoded Protocol::set_id(std::uint16_t id, std::uint16_t new_id, WritePermission permission) const {
  require_write(permission);
  require_motor(id);
  require_motor(new_id);
  auto f = frame(management_id, 6);
  set_u16(f, 0, id);
  f.data[3] = 0x04;
  set_u16(f, 4, new_id);
  return {f, {ReplyKind::set_id, management_id, 0, id, new_id}, management_delay};
}

Encoded Protocol::reset_id(WritePermission permission) const {
  require_write(permission);
  auto f = frame(management_id, 6);
  f.data = {0x7f, 0x7f, 0x00, 0x05, 0x7f, 0x7f};
  return {f, {ReplyKind::reset_id, management_id}, management_delay};
}

Encoded Protocol::brake(std::uint16_t id, BrakeState state, WritePermission permission) const {
  require_write(permission);
  require_motor(id);
  if (state != BrakeState::engaged && state != BrakeState::released) {
    throw CodecError(CodecIssue::value, "invalid brake state");
  }
  auto f = frame(id, 2);
  f.data = {0x80, static_cast<std::uint8_t>(state)};
  return {f, {ReplyKind::brake, id, 0xb2}};
}

}  // namespace encos
