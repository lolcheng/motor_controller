#include "encos_driver/ros_interfaces.hpp"
#include <cmath>
#include <iostream>
#include <limits>

using namespace encos;
using namespace encos::driver;

int main() {
  msg::MotorCommand request;
  request.motor_id = 1;
  request.mode = request.POSITION;
  request.feedback = 2;
  request.position_rad = -90 * wire::degrees_to_rad;
  request.speed_limit_rad_s = 20 * wire::rpm_to_rad_s;
  request.current_limit_a = 5;
  const auto packet = Protocol{}.control(1, to_control(request, 2)).frame;
  const std::array<std::uint8_t, 8> reference{0x38, 0x56, 0x80, 0, 0, 0x32, 0, 0xca};
  if (!std::equal(reference.begin(), reference.end(), packet.data.begin())) return 1;
  request.feedforward_nm = std::numeric_limits<double>::quiet_NaN();
  try {
    to_control(request, 2);
    return 2;
  } catch (const CodecError&) {
  }
  Telemetry info;
  info.type = 2;
  info.position_rad = Measurement{.2, MeasurementSource::float_feedback, TimePoint{}};
  const auto state = state_message(1, &info);
  if (!state.valid[0] || state.valid[1] || !std::isnan(state.velocity_rad_s) ||
      state.source[0] != state.SOURCE_FLOAT || state.error_valid == false) return 3;
  msg::ParameterValue parameter;
  parameter.kind = parameter.PAIR;
  try {
    to_config(ConfigCode::kt, parameter);
    return 4;
  } catch (const CodecError&) {
  }

  std::cout << "ROS mapping, vendor frame, missing values and typed payload PASS\n";
}
