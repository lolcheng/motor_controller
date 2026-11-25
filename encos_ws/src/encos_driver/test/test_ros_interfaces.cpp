#include "encos_driver/ros_interfaces.hpp"
#include "encos_driver/sample_recorder.hpp"

#include <atomic>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>

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

  SampleQueue<int, 17> queue;
  for (int i = 0; i < 16; ++i) if (!queue.push(i)) return 5;
  if (queue.push(16)) return 6;
  for (int i = 0; i < 16; ++i) if (queue.take() != i) return 7;
  if (queue.take()) return 8;
  constexpr int count = 50000;
  std::thread producer([&] {
    for (int i = 0; i < count; ++i) while (!queue.push(i)) std::this_thread::yield();
  });
  bool ordered = true;
  for (int i = 0; i < count; ++i) {
    auto value = queue.take();
    while (!value) {
      std::this_thread::yield();
      value = queue.take();
    }
    ordered = ordered && *value == i;
  }
  producer.join();
  if (!ordered) return 9;
  std::cout << "ROS mapping, vendor frame, missing values, typed payload and 50000 concurrent samples PASS\n";
}
