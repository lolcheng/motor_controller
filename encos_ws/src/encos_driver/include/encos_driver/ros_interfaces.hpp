#pragma once

#include "encos_core/protocol.hpp"
#include "encos_msgs/msg/motor_command.hpp"
#include "encos_msgs/msg/motor_state.hpp"
#include "encos_msgs/msg/parameter_value.hpp"
#include "encos_msgs/msg/raw_frame.hpp"

namespace encos::driver {
namespace msg = encos_msgs::msg;

std::int64_t mono(TimePoint time);
Control to_control(const msg::MotorCommand& command, std::uint8_t applied_ack);
ConfigValue to_config(ConfigCode code, const msg::ParameterValue& value);
msg::ParameterValue typed_value(const ParameterValue& value);
msg::RawFrame raw_message(const CanFrame& frame, TimePoint time, const ReceivedFrame* sample = nullptr);
msg::MotorState state_message(std::uint16_t id, const Telemetry* feedback = nullptr,
                              const ReceivedFrame* sample = nullptr);
}  // namespace encos::driver
