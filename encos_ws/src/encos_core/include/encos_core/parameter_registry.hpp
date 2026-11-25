#pragma once

#include <cstdint>

namespace encos {

enum class QueryCode : std::uint8_t {
  position = 1, velocity, phase_current, power, acceleration,
  kt = 22, kp_range, kd_range, position_range, velocity_range, torque_range,
  current_range, uuid, version, timeout, current_pi, velocity_pi, position_pd,
  kt_calibration, kt_segment, brake, zero_offset, encoder,
};

enum class ConfigCode : std::uint8_t {
  acceleration = 1, communication = 2,
  kt = 4, kp_range, kd_range, position_range, velocity_range, torque_range,
  current_range, timeout, current_pi, velocity_pi, position_pd, kt_calibration,
  kt_segment, zero_offset,
};

enum class Payload { float32, scalar16, pair16, enumeration, uuid, version, segment, encoder };

struct Field {
  bool signed_value;
  double scale;  // wire units per public value
};

struct Parameter {
  Payload payload;
  Field first;
  Field second;
  bool ordered_pair;
};

const Parameter* parameter(QueryCode code);
const Parameter* parameter(ConfigCode code);
unsigned payload_size(Payload payload);

}  // namespace encos
