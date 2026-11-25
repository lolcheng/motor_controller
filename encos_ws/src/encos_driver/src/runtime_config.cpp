#include "encos_driver/runtime_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace encos::driver {
RuntimeConfig load_config(rclcpp::Node& node) {
  RuntimeConfig result;
  const auto declare = [&](const auto& key, const auto& def) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter(key, def, descriptor);
  };
  const auto interval = [&](const char* key, int def, int min, int max) {
    const auto value = declare(key, def);
    if (value < min || value > max) throw std::invalid_argument(std::string("out of range: ") + key);
    return static_cast<unsigned>(value);
  };
  result.interface = declare("can_interface", result.interface);
  result.joint = declare("joint_name", result.joint);
  result.model = declare("motor_model", result.model);
  if (result.model != "EC-A10825-P2-24H-B" || result.joint.empty()) throw std::invalid_argument("invalid model/joint");
  result.format = declare("fd_brs", false) ? WireFormat::fd_brs : WireFormat::classic;
  result.id = static_cast<std::uint16_t>(interval("motor_id", 1, 1, 2046));
  result.frequency = interval("control_hz", 400, 10, 1000);
  result.publish_ms = interval("publish_ms", 2, 1, 20);
  result.safety_feedback_ms = interval("safety_feedback_ms", 100, 20, 200);
  result.allow_writes = declare("allow_virtual_writes", false);
  result.raw = declare("publish_raw", false);
  result.publish_samples = declare("publish_samples", true);
  result.publish_state = declare("publish_state", true);
  result.joint_states = declare("publish_joint_states", true);
  auto& limit = result.limits;
  limit.position_rad.min = declare("limits.position_min_rad", limit.position_rad.min);
  limit.position_rad.max = declare("limits.position_max_rad", limit.position_rad.max);
  limit.velocity_rad_s = declare("limits.velocity_rad_s", limit.velocity_rad_s);
  limit.phase_current_a = declare("limits.phase_current_a", limit.phase_current_a);
  limit.torque_nm = declare("limits.torque_nm", limit.torque_nm);
  limit.motor_c = declare("limits.motor_c", limit.motor_c);
  limit.mos_c = declare("limits.mos_c", *limit.mos_c);
  limit.kp = declare("limits.kp", limit.kp);
  limit.kd = declare("limits.kd", limit.kd);
  limit.command_age = Milliseconds(interval("limits.command_age_ms", 150, 10, 1000));
  limit.feedback_age = Milliseconds(interval("limits.feedback_age_ms", 350, 210, 2000));
  limit.run_duration = Milliseconds(interval("limits.run_ms", 90000, 100, 300000));
  limit.allow_current = limit.allow_torque = declare("allow_virtual_current_torque", false);
  limit.allow_damping = limit.allow_dissipative = limit.allow_regenerative =
      declare("allow_virtual_braking", false);
  limit.allow_brake = declare("allow_virtual_electromagnetic_brake", false);
  if (!limit.valid() || limit.feedback_age <= Milliseconds(result.safety_feedback_ms) +
                                               Milliseconds(1000 / result.frequency)) {
    throw std::invalid_argument("invalid limits or safety feedback interval");
  }
  return result;
}
}  // namespace encos::driver
