#pragma once

#include "encos_driver/ros_interfaces.hpp"
#include "encos_driver/runtime_config.hpp"
#include "encos_driver/sample_recorder.hpp"
#include "encos_core/socketcan_transport.hpp"
#include "encos_core/transaction_manager.hpp"
#include "encos_experiments/trajectory.hpp"
#include "encos_msgs/msg/control_sample.hpp"

#include <mutex>
#include <thread>

namespace encos::driver {

enum class Authority : std::uint8_t { none, read_only, topic, action };
enum class JobKind { arm, reset, discover, query, configure, zero, set_id, reset_id, brake };

struct Target {
  msg::MotorCommand command;
  TimePoint generated, received, expires;
  builtin_interfaces::msg::Time ros_received;
  bool ros_received_valid{};
  bool time_valid{true};
};

struct Goal {
  std::uint64_t token{};
  msg::MotorCommand command;
  experiments::Trajectory wave;
};

struct ActionUpdate {
  std::uint64_t token{}, samples{};
  double elapsed{}, progress{}, target{};
  enum Stage { running, success, aborted, cancelled } stage{running};
  const char* detail{""};
};

struct Job {
  explicit Job(JobKind operation) : kind(operation) {}
  JobKind kind;
  std::uint8_t code{}, group{};
  std::uint16_t id{};
  std::uint64_t owner{};
  bool confirm{}, option{}, secondary{};
  double number{};
  msg::ParameterValue value = msg::ParameterValue();
};

struct JobResult {
  bool success{}, acknowledged{};
  const char* detail{"request rejected"};
  std::uint16_t id{};
  msg::ParameterValue value = msg::ParameterValue();
  msg::RawFrame raw = msg::RawFrame();
};

struct Counters {
  std::uint64_t tx{}, rx{}, matched{}, expected{}, timeouts{}, invalid{}, ignored{};
  std::uint64_t command_received{}, dropped{}, expired{}, skipped{}, overruns{}, overflow{}, socket_drops{};
};

struct Snapshot {
  msg::MotorProfile profile = msg::MotorProfile();
  msg::MotorState state = msg::MotorState();
  Counters counters;
  TimePoint stamp;
  SafetyFault fault{};
  const char* detail{""};
};

struct RawEvent { bool tx; msg::RawFrame frame; };

class ControlWorker {
 public:
  explicit ControlWorker(RuntimeConfig config);
  ~ControlWorker();
  const RuntimeConfig& config() const { return config_; }
  bool submit_job(Job job);
  void submit_target(Target target);
  bool submit_goal(Goal goal);
  void request_stop() { stop_.store(true); }
  void request_cancel(std::uint64_t token) { cancel_.store(token); }
  void count_rejected() { ++foreign_commands_; }
  void count_expired() { ++received_commands_; ++expired_commands_; }
  auto take_job() { return job_results_.take(); }
  auto take_sample() { return samples_.take(); }
  auto take_status() { return status_.take(); }
  auto take_raw() { return raw_.take(); }
  auto take_action() { return actions_.take(); }

 private:
  struct RunningGoal { Goal goal; TimePoint started; TimePoint updated; std::uint64_t samples{}; double target{}; };

  void run() noexcept;
  void cycle(TimePoint scheduled, TimePoint now);
  void accept_goal(Goal goal, TimePoint now);
  void motion(Target target, TimePoint scheduled, TimePoint now);
  void motion_reply(TransactionEvent event, TimePoint now);
  void consume(const ReceivedFrame& frame, TimePoint now);
  void finish_action(ActionUpdate::Stage stage, const char* detail, TimePoint now);
  void stopping(const char* detail, bool intentional, TimePoint now);
  bool confirm_virtual_stop();
  void publish_snapshot(TimePoint now);
  void enter(std::optional<ReceivedFrame> frame);
  bool send_frame(const Encoded& packet, TimePoint before, SendStatus& status);
  TransactionEvent transact(const Encoded& packet, bool prime = false);
  JobResult operate(const Job& job);
  bool verify();
  bool refresh_prime();
  bool fresh_prime(TimePoint now) const;
  JobResult feedback_result(const TransactionEvent& event);

  const RuntimeConfig config_;
  const std::chrono::nanoseconds period_;
  SafetyManager guard_;
  Protocol protocol_;
  std::unique_ptr<SocketCanTransport> bus_;
  TransactionManager transactions_{std::chrono::milliseconds(120), ReplyReuse::single_reply_simulation};
  std::uint16_t id_;
  bool connected_{true}, intentional_stop_{}, stop_checked_{};
  Authority authority_{Authority::none};
  Owner owner_{};
  VerifiedMotor profile_{};
  std::optional<ReceivedFrame> prime_;
  std::optional<TimePoint> full_feedback_, stop_after_, expiry_, last_cycle_, status_after_, expected_reply_;
  std::optional<RunningGoal> active_goal_;
  std::optional<Target> current_target_;
  std::optional<msg::ControlSample> pending_;
  msg::MotorState latest_;
  Counters counts_;
  const char* detail_{"waiting for explicit authorization"};
  std::uint64_t sequence_{};

  std::mutex inbox_;
  std::optional<Job> job_;
  std::optional<Target> target_;
  std::optional<Goal> goal_;
  std::atomic<std::uint64_t> received_commands_{}, overwritten_commands_{}, foreign_commands_{},
      expired_commands_{}, cancel_{};
  std::atomic<bool> shutdown_{}, stop_{};
  SampleQueue<msg::ControlSample, 4097> samples_;
  SampleQueue<RawEvent, 4097> raw_;
  SampleQueue<Snapshot, 65> status_;
  SampleQueue<JobResult, 3> job_results_;
  SampleQueue<ActionUpdate, 1025> actions_;
  std::thread runner_;
};

}  // namespace encos::driver
