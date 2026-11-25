#pragma once

#include "encos_core/bit_codec.hpp"
#include "encos_core/can_frame.hpp"
#include "encos_core/command.hpp"
#include "encos_core/parameter_registry.hpp"

#include <optional>
#include <variant>

namespace encos {

enum class MotorError : std::uint8_t {
  none, overheat, overcurrent, overvoltage, undervoltage, encoder, brake_overvoltage, driver,
};

enum class MeasurementSource { quantized_feedback, float_feedback, verified_kt_estimate };

struct Measurement {
  double value;
  MeasurementSource source;
  TimePoint host_rx;
};

struct Telemetry {
  std::uint8_t type{};
  MotorError error{};
  std::optional<Measurement> position_rad, velocity_rad_s, phase_current_a;
  std::optional<Measurement> motor_c, mos_c, estimated_torque_nm;
  // Present for type 1 even when the corresponding physical ranges are unavailable.
  std::optional<std::array<std::uint16_t, 3>> quantized;
};

struct Raw16 { std::uint16_t value; };
struct UuidPart { std::uint8_t index; std::array<std::uint8_t, 4> bytes; };
struct Version { std::array<std::uint8_t, 3> hardware, software; };
struct EncoderAngles {
  bool calibrated;
  std::int16_t raw_count, calibrated_count;
  std::optional<double> raw_rad, calibrated_rad;
};
using ParameterValue = std::variant<Scalar, Pair, Enumeration, Raw16, UuidPart, Version, EncoderAngles>;

struct QueryReply { QueryCode code; MotorError error; ParameterValue value; };
struct ConfigStatus { ConfigCode code; MotorError error; bool success; };
struct ConfigEcho { ConfigCode code; ParameterValue value; };
struct KtReply {
  std::uint8_t raw_header;
  std::optional<std::uint8_t> group;
  std::array<double, 4> values;
};
struct BrakeReply { BrakeState state; std::optional<MotorError> error; };
enum class ManagementKind { discover, zero, set_id, reset_id, write_failed };
struct ManagementReply { ManagementKind kind; bool success; std::optional<std::uint16_t> id; };

using Feedback = std::variant<CodecIssue, Telemetry, QueryReply, ConfigStatus, ConfigEcho,
                              KtReply, BrakeReply, ManagementReply>;
struct Decoded { ReceivedFrame raw; Feedback feedback; };

}  // namespace encos
