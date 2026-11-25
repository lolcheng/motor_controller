#include "encos_core/protocol.hpp"

#include <cmath>

namespace encos {
namespace {

void require_length(const CanFrame& f, unsigned expected) {
  if (f.length != expected) {
    throw CodecError(CodecIssue::length, "unexpected feedback length");
  }
}

MotorError motor_error(unsigned value) {
  if (value > 7) {
    throw CodecError(CodecIssue::unsupported, "unknown motor error code");
  }
  return static_cast<MotorError>(value);
}

double physical(std::uint32_t raw, std::uint32_t max, const Range& range) {
  return range.min + (static_cast<double>(raw) / max) * (range.max - range.min);
}

Telemetry telemetry(const ReceivedFrame& sample, std::uint8_t type, const ProtocolProfile& profile) {
  const auto& f = sample.frame;
  require_length(f, 8);
  const auto* bytes = f.data.data();
  Telemetry result;
  result.type = type;
  result.error = motor_error(bytes[0] & 0x1f);
  const auto measured = [&](double value) {
    return Measurement{value, type == 1 ? MeasurementSource::quantized_feedback
                                        : MeasurementSource::float_feedback, sample.stamp};
  };
  result.motor_c = measured((static_cast<int>(bytes[type == 1 ? 6 : 7]) - 50) / 2.0);
  if (type == 1) {
    const auto position = wire::read_u16(bytes + 1);
    const auto velocity = static_cast<std::uint16_t>(wire::read_bits(bytes, 24, 12));
    const auto current = static_cast<std::uint16_t>(wire::read_bits(bytes, 36, 12));
    result.quantized = {{position, velocity, current}};
    result.mos_c = measured((static_cast<int>(bytes[7]) - 50) / 2.0);
    if (profile.feedback1 && profile.feedback1->valid()) {
      result.position_rad = measured(physical(position, 65535, profile.feedback1->position_rad));
      result.velocity_rad_s = measured(physical(velocity, 4095, profile.feedback1->velocity_rad_s));
      result.phase_current_a = measured(physical(current, 4095, profile.feedback1->phase_current_a));
    }
  } else {
    const auto value = wire::read_float(bytes + 1);
    if (type == 2) {
      result.position_rad = measured(value * wire::degrees_to_rad);
    } else {
      result.velocity_rad_s = measured(value * wire::rpm_to_rad_s);
    }
    result.phase_current_a = measured(wire::read_i16(bytes + 5) / 100.0);
  }
  if (result.error == MotorError::none && result.phase_current_a && profile.query35_raw == 1 &&
      profile.kt_nm_per_a && std::isfinite(*profile.kt_nm_per_a) && *profile.kt_nm_per_a > 0) {
    const double estimate = result.phase_current_a->value * *profile.kt_nm_per_a;
    if (std::isfinite(estimate)) {
      result.estimated_torque_nm = Measurement{estimate, MeasurementSource::verified_kt_estimate, sample.stamp};
    }
  }
  return result;
}

double field_value(const std::uint8_t* data, Field field) {
  return (field.signed_value ? wire::read_i16(data) : wire::read_u16(data)) / field.scale;
}

ParameterValue parameter_value(const Parameter& parameter, const std::uint8_t* data,
                               bool encoder_degrees_confirmed = false) {
  switch (parameter.payload) {
    case Payload::float32: return Scalar{wire::read_float(data) / parameter.first.scale};
    case Payload::scalar16: return Scalar{field_value(data, parameter.first)};
    case Payload::pair16: {
      const Pair pair{field_value(data, parameter.first), field_value(data + 2, parameter.second)};
      if (parameter.ordered_pair && pair.first >= pair.second) {
        throw CodecError(CodecIssue::value, "feedback range endpoints out of order");
      }
      return pair;
    }
    case Payload::enumeration: return Enumeration{data[0]};
    case Payload::uuid: {
      if (data[0] < 1 || data[0] > 3) {
        throw CodecError(CodecIssue::value, "invalid UUID fragment index");
      }
      return UuidPart{data[0], {data[1], data[2], data[3], data[4]}};
    }
    case Payload::version: return Version{{data[0], data[1], data[2]}, {data[3], data[4], data[5]}};
    case Payload::encoder: {
      if (data[0] > 1) {
        throw CodecError(CodecIssue::value, "invalid encoder calibration status");
      }
      EncoderAngles result{data[0] != 0, wire::read_i16(data + 1), wire::read_i16(data + 3), {}, {}};
      if (encoder_degrees_confirmed) {
        result.raw_rad = result.raw_count / 100.0 * wire::degrees_to_rad;
        result.calibrated_rad = result.calibrated_count / 100.0 * wire::degrees_to_rad;
      }
      return result;
    }
    case Payload::segment: break;
  }
  throw CodecError(CodecIssue::unsupported, "unsupported ordinary feedback payload");
}

QueryReply query_reply(const CanFrame& f, const ProtocolProfile& profile) {
  const auto code = static_cast<QueryCode>(f.data[1]);
  const auto* spec = parameter(code);
  if (!spec || spec->payload == Payload::segment) {
    throw CodecError(CodecIssue::unsupported, "undocumented ordinary query reply");
  }
  require_length(f, 2 + payload_size(spec->payload));
  const auto err = motor_error(f.data[0] & 0x1f);
  const auto* data = f.data.data() + 2;
  if ((code == QueryCode::kt_calibration && (data[0] < 1 || data[0] > 3)) ||
      (code == QueryCode::brake && data[0] > 1)) {
    throw CodecError(CodecIssue::value, "invalid query enumeration");
  }
  if (code == QueryCode::acceleration) {
    return {code, err, Raw16{wire::read_u16(data)}};
  }
  return {code, err, parameter_value(*spec, data, profile.query39_degrees_confirmed)};
}

ConfigEcho config_echo(const CanFrame& f) {
  if (f.length < 3) {
    throw CodecError(CodecIssue::length, "missing special configuration echo code");
  }
  const auto code = static_cast<ConfigCode>(f.data[2]);
  const auto* spec = parameter(code);
  if (!spec || spec->payload == Payload::segment) {
    throw CodecError(CodecIssue::unsupported, "unknown special configuration echo");
  }
  if (code == ConfigCode::communication) {
    if (f.length != 4 && f.length != 5) {
      throw CodecError(CodecIssue::length, "invalid communication-mode echo length");
    }
    throw CodecError(CodecIssue::ambiguous, "communication-mode echo layout unconfirmed");
  }
  require_length(f, 3 + payload_size(spec->payload));
  if ((code == ConfigCode::kt_calibration && f.data[3] > 2) ||
      (code == ConfigCode::acceleration && wire::read_u16(f.data.data() + 3) > 2000)) {
    throw CodecError(CodecIssue::value, "invalid configuration echo value");
  }
  return {code, parameter_value(*spec, f.data.data() + 3)};
}

KtReply kt_reply(const CanFrame& f) {
  require_length(f, 8);
  const auto header = f.data[1];
  const auto group = static_cast<std::uint8_t>(header & 0x1f);
  KtReply result{header, {}, {}};
  // The common group is usable only when both published interpretations are possible.
  // The mode remains raw even if a group is known.
  if (group >= 1 && group <= 10 && (header >> 4) <= 2) {
    result.group = group;
  }
  for (unsigned i = 0; i < result.values.size(); ++i) {
    result.values[i] = wire::read_bits(f.data.data(), 16 + 12 * i, 12) / 100.0;
  }
  return result;
}

ManagementReply management(const CanFrame& f) {
  if (f.length < 4) {
    throw CodecError(CodecIssue::length, "missing management feedback fields");
  }
  const auto* bytes = f.data.data();
  if (bytes[2] != 1) {
    throw CodecError(CodecIssue::unsupported, "management frame lacks motor reply marker");
  }
  const auto prefix = wire::read_u16(bytes);
  if (prefix == 0xffff) {
    require_length(f, 5);
    const auto id = wire::read_u16(bytes + 3);
    if (!valid_motor_id(id)) {
      throw CodecError(CodecIssue::value, "discovery returned invalid motor id");
    }
    return {ManagementKind::discover, true, id};
  }
  if (prefix == 0x8080 && bytes[3] == 0x80) {
    require_length(f, 4);
    return {ManagementKind::discover, false, {}};
  }
  if (prefix == 0x7f7f && bytes[3] == 0x05) {
    require_length(f, 6);
    if (wire::read_u16(bytes + 4) != 0x7f7f) {
      throw CodecError(CodecIssue::value, "invalid reset reply suffix");
    }
    return {ManagementKind::reset_id, true, 1};
  }
  if (!valid_motor_id(prefix)) {
    throw CodecError(CodecIssue::unsupported, "unknown management reply");
  }
  require_length(f, 4);
  switch (bytes[3]) {
    case 0x00: return {ManagementKind::write_failed, false, prefix};
    case 0x03: return {ManagementKind::zero, true, prefix};
    case 0x04: return {ManagementKind::set_id, true, prefix};
    default: throw CodecError(CodecIssue::unsupported, "unknown management reply code");
  }
}

BrakeState brake_state(unsigned value) {
  if (value > 1) {
    throw CodecError(CodecIssue::value, "invalid brake state");
  }
  return static_cast<BrakeState>(value);
}

Feedback decode_payload(const ReceivedFrame& sample, const ProtocolProfile& profile) {
  const auto& f = sample.frame;
  const auto& bytes = f.data;
  if (f.id == management_id) {
    return management(f);
  }
  if (bytes[0] == 0xff && bytes[1] == 0xfe) {
    return config_echo(f);
  }
  if (bytes[0] == 0xee) {
    return kt_reply(f);
  }
  if (bytes[0] == 0xb2) {
    require_length(f, 2);
    return BrakeReply{brake_state(bytes[1]), {}};
  }
  const auto type = static_cast<std::uint8_t>(bytes[0] >> 5);
  switch (type) {
    case 1:
    case 2:
    case 3: return telemetry(sample, type, profile);
    case 4: {
      require_length(f, 3);
      const auto code = static_cast<ConfigCode>(bytes[1]);
      if (!parameter(code)) {
        return CodecIssue::unsupported;
      }
      if (bytes[2] > 1) {
        return CodecIssue::value;
      }
      return ConfigStatus{code, motor_error(bytes[0] & 0x1f), bytes[2] != 0};
    }
    case 5: return query_reply(f, profile);
    case 6:
      require_length(f, 2);
      return BrakeReply{brake_state(bytes[1]), motor_error(bytes[0] & 0x1f)};
    default: return CodecIssue::unsupported;
  }
}

}  // namespace

Decoded Protocol::decode(const ReceivedFrame& sample, const ProtocolProfile& profile) const {
  const auto& f = sample.frame;
  if (f.id == 0 || f.id > management_id || f.rtr || f.error || f.extended ||
      !matches_wire_format(f, format_)) {
    return {sample, CodecIssue::invalid_frame};
  }
  if (f.length < 2 || f.length > 8) {
    return {sample, CodecIssue::length};
  }
  try {
    return {sample, decode_payload(sample, profile)};
  } catch (const CodecError& error) {
    return {sample, error.issue};
  }
}

}  // namespace encos
