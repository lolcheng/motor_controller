#include "encos_core/parameter_registry.hpp"

#include "encos_core/bit_codec.hpp"

namespace encos {
namespace {

constexpr Parameter fp{Payload::float32, {false, 1}, {}, false};
constexpr Parameter degree_fp{Payload::float32, {false, 1 / wire::degrees_to_rad}, {}, false};
constexpr Parameter rpm_fp{Payload::float32, {false, 1 / wire::rpm_to_rad_s}, {}, false};
constexpr Parameter raw16{Payload::scalar16, {false, 1}, {}, false};
constexpr Parameter centi{Payload::scalar16, {false, 100}, {}, false};
constexpr Parameter gains{Payload::pair16, {false, 1}, {false, 1}, true};
constexpr Parameter pos_spd{Payload::pair16, {true, 100}, {true, 100}, true};
constexpr Parameter tor_cur{Payload::pair16, {true, 10}, {true, 10}, true};
constexpr Parameter current_pi{Payload::pair16, {false, 10000}, {false, 10}, false};
constexpr Parameter speed_or_position{Payload::pair16, {false, 100000}, {false, 100000}, false};
constexpr Parameter zero_offset{Payload::scalar16, {true, 100 / wire::degrees_to_rad}, {}, false};
constexpr Parameter enumeration{Payload::enumeration, {}, {}, false};
constexpr Parameter uuid{Payload::uuid, {}, {}, false};
constexpr Parameter version{Payload::version, {}, {}, false};
constexpr Parameter segment{Payload::segment, {}, {}, false};
constexpr Parameter encoder{Payload::encoder, {}, {}, false};

}  // namespace

const Parameter* parameter(QueryCode code) {
  switch (code) {
    case QueryCode::position: return &degree_fp;
    case QueryCode::velocity: return &rpm_fp;
    case QueryCode::phase_current:
    case QueryCode::power: return &fp;
    case QueryCode::acceleration:
    case QueryCode::timeout: return &raw16;
    case QueryCode::kt: return &centi;
    case QueryCode::kp_range:
    case QueryCode::kd_range: return &gains;
    case QueryCode::position_range:
    case QueryCode::velocity_range: return &pos_spd;
    case QueryCode::torque_range:
    case QueryCode::current_range: return &tor_cur;
    case QueryCode::uuid: return &uuid;
    case QueryCode::version: return &version;
    case QueryCode::current_pi: return &current_pi;
    case QueryCode::velocity_pi:
    case QueryCode::position_pd: return &speed_or_position;
    case QueryCode::kt_calibration:
    case QueryCode::brake: return &enumeration;
    case QueryCode::kt_segment: return &segment;
    case QueryCode::zero_offset: return &zero_offset;
    case QueryCode::encoder: return &encoder;
  }
  return nullptr;
}

const Parameter* parameter(ConfigCode code) {
  switch (code) {
    case ConfigCode::acceleration:
    case ConfigCode::kt: return &centi;
    case ConfigCode::communication:
    case ConfigCode::kt_calibration: return &enumeration;
    case ConfigCode::kp_range:
    case ConfigCode::kd_range: return &gains;
    case ConfigCode::position_range:
    case ConfigCode::velocity_range: return &pos_spd;
    case ConfigCode::torque_range:
    case ConfigCode::current_range: return &tor_cur;
    case ConfigCode::timeout: return &raw16;
    case ConfigCode::current_pi: return &current_pi;
    case ConfigCode::velocity_pi:
    case ConfigCode::position_pd: return &speed_or_position;
    case ConfigCode::kt_segment: return &segment;
    case ConfigCode::zero_offset: return &zero_offset;
  }
  return nullptr;
}

unsigned payload_size(Payload payload) {
  switch (payload) {
    case Payload::enumeration: return 1;
    case Payload::scalar16: return 2;
    case Payload::float32:
    case Payload::pair16: return 4;
    case Payload::uuid:
    case Payload::encoder: return 5;
    case Payload::version:
    case Payload::segment: return 6;
  }
  return 0;
}

}  // namespace encos
