#pragma once

#include "encos_core/safety_manager.hpp"

#include <string>

namespace rclcpp { class Node; }

namespace encos::driver {
struct RuntimeConfig {
  std::string interface{"vcan0"};
  std::string model{"EC-A10825-P2-24H-B"};
  std::string joint{"encos_joint"};
  WireFormat format{WireFormat::classic};
  std::uint16_t id{1};
  unsigned frequency{400};
  unsigned publish_ms{2};
  unsigned safety_feedback_ms{100};
  bool allow_writes{};
  bool raw{};
  bool joint_states{true};
  bool publish_samples{true};
  bool publish_state{true};
  Limits limits{{-.5, .5}, 1.5, 3, 4, 65, 25, 2, 65, Milliseconds(150),
                Milliseconds(350), Milliseconds(90000)};
};
RuntimeConfig load_config(rclcpp::Node& node);
}  // namespace encos::driver
