#pragma once

#include "encos_driver/control_worker.hpp"
#include "encos_msgs/action/run_experiment.hpp"
#include "encos_msgs/msg/communication_stats.hpp"
#include "encos_msgs/msg/motor_fault.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <deque>
#include <functional>

namespace encos::driver {
class MotorNode : public rclcpp::Node {
 public:
  explicit MotorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MotorNode() override;

 private:
  using Experiment = encos_msgs::action::RunExperiment;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Experiment>;
  void command(const msg::MotorCommand& command, const rclcpp::MessageInfo& info);
  void drain();
  void publish_state(msg::MotorState state, const rclcpp::Time& now);
  void diagnostics(Snapshot snapshot);
  bool valid_goal(const Experiment::Goal& goal) const;
  void action_update(const ActionUpdate& update);

  template<class Service, class MakeJob, class FillResponse>
  void service(const char* name, MakeJob make, FillResponse fill);

  RuntimeConfig config_;
  std::unique_ptr<ControlWorker> worker_;
  Snapshot snapshot_{};
  std::optional<std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>> publisher_gid_;
  std::int64_t last_source_stamp_{};
  std::optional<Snapshot> previous_stats_;
  std::deque<double> rtts_, jitters_;
  std::function<void(const JobResult&)> pending_service_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::shared_ptr<GoalHandle> active_action_;
  bool action_admission_{};
  std::uint64_t action_token_{};
  std::optional<std::pair<SafetyState, SafetyFault>> last_fault_;
  rclcpp::Subscription<msg::MotorCommand>::SharedPtr commands_;
  rclcpp::Publisher<msg::MotorState>::SharedPtr states_;
  rclcpp::Publisher<msg::ControlSample>::SharedPtr samples_;
  rclcpp::Publisher<msg::CommunicationStats>::SharedPtr stats_;
  rclcpp::Publisher<msg::MotorFault>::SharedPtr faults_;
  rclcpp::Publisher<msg::RawFrame>::SharedPtr raw_tx_, raw_rx_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp_action::Server<Experiment>::SharedPtr actions_;
  rclcpp::TimerBase::SharedPtr pump_;
};
}  // namespace encos::driver
