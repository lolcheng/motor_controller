#pragma once

#include <array>
#include <cstdint>
#include <variant>

namespace encos {

enum class Ack : std::uint8_t { none, type1, type2, type3 };
enum class DriveMode : std::uint8_t { current, torque, damping, dissipative, regenerative, brake };
enum class BrakeState : std::uint8_t { engaged, released };

struct Mixed {
  double kp{};
  double kd{};
  double position_rad{};
  double velocity_rad_s{};
  double torque_nm{};
};
struct Position {
  double position_rad{};
  double speed_limit_rad_s{};
  double current_limit_a{};
  Ack ack{Ack::none};
};
struct Velocity {
  double velocity_rad_s{};
  double current_limit_a{};
  Ack ack{Ack::none};
};
struct Drive {
  DriveMode mode{};
  // A for current and brake-current thresholds, Nm for torque, 0 or 1 for brake.
  double value{};
  Ack ack{Ack::none};
};
using Control = std::variant<Mixed, Position, Velocity, Drive>;

struct Scalar { double value; };
struct Pair { double first, second; };
struct Enumeration { std::uint8_t raw; };
struct KtWrite { std::uint8_t group; std::array<double, 4> values; };
using ConfigValue = std::variant<Scalar, Pair, Enumeration, KtWrite>;

enum class WritePermission { denied, offline };

}  // namespace encos
