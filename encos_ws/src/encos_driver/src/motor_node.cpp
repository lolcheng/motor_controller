#include "encos_driver/motor_node.hpp"

#include "encos_msgs/srv/arm.hpp"
#include "encos_msgs/srv/discover_motor.hpp"
#include "encos_msgs/srv/get_profile.hpp"
#include "encos_msgs/srv/read_parameter.hpp"
#include "encos_msgs/srv/reset_fault.hpp"
#include "encos_msgs/srv/reset_motor_id.hpp"
#include "encos_msgs/srv/set_brake.hpp"
#include "encos_msgs/srv/set_motor_id.hpp"
#include "encos_msgs/srv/set_zero.hpp"
#include "encos_msgs/srv/stop.hpp"
#include "encos_msgs/srv/write_parameter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace encos::driver {
namespace {
namespace srv = encos_msgs::srv;
using M = msg::MotorCommand;
constexpr auto prefix = "/encos/";
constexpr std::int64_t ns_per_sec = 1'000'000'000;
using namespace std::chrono_literals;

void add_metric(std::deque<double>& samples, double value) {
  if (samples.size() >= 4096) samples.pop_front();
  samples.push_back(value);
}
double percentile(const std::vector<double>& sorted, double ratio) {
  if (sorted.empty()) return std::numeric_limits<double>::quiet_NaN();
  const auto rank = static_cast<std::size_t>(std::ceil(sorted.size() * ratio));
  return sorted[rank - 1];
}
}  // namespace

template<class Service, class MakeJob, class FillResponse>
void MotorNode::service(const char* name, MakeJob make, FillResponse fill) {
  using Request = typename Service::Request;
  using Response = typename Service::Response;
  services_.push_back(create_service<Service>(std::string(prefix) + name,
      [this, make, fill](std::shared_ptr<rclcpp::Service<Service>> handle,
                         std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Request> request) {
    auto reply = [handle, header, fill](const JobResult& result) {
      Response response;
      response.success = result.success;
      response.detail = result.detail;
      fill(response, result);
      handle->send_response(*header, response);
    };
    if (pending_service_) {
      JobResult result;
      result.detail = "another service is pending";
      reply(result);
      return;
    }
    const auto active = snapshot_.state.authority >= static_cast<unsigned>(Authority::topic);
    auto job = make(*request);
    if (active && job.kind != JobKind::reset) {
      JobResult result;
      result.detail = "management unavailable while motion authority is held";
      reply(result);
      return;
    }
    if (!worker_->submit_job(std::move(job))) {
      JobResult result;
      result.detail = "worker unavailable";
      reply(result);
      return;
    }
    pending_service_ = std::move(reply);
  }));
}

MotorNode::MotorNode(const rclcpp::NodeOptions& options)
    : Node("encos_driver", options), config_(load_config(*this)), worker_(std::make_unique<ControlWorker>(config_)) {
  snapshot_.state = state_message(config_.id);
  commands_ = create_subscription<M>("/encos/motor_command", rclcpp::QoS(1).best_effort(),
      [this](M::ConstSharedPtr value, const rclcpp::MessageInfo& info) { command(*value, info); });
  if (config_.publish_state) states_ = create_publisher<msg::MotorState>("/encos/motor_state", rclcpp::QoS(8).best_effort());
  if (config_.publish_samples) samples_ = create_publisher<msg::ControlSample>("/encos/control_sample", rclcpp::QoS(2048));
  stats_ = create_publisher<msg::CommunicationStats>("/encos/communication_stats", rclcpp::QoS(4));
  faults_ = create_publisher<msg::MotorFault>("/encos/fault", rclcpp::QoS(1).transient_local());
  if (config_.joint_states) joints_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(8).best_effort());
  if (config_.raw) {
    raw_tx_ = create_publisher<msg::RawFrame>("/encos/raw_tx", rclcpp::QoS(128).best_effort());
    raw_rx_ = create_publisher<msg::RawFrame>("/encos/raw_rx", rclcpp::QoS(128).best_effort());
  }

  const auto ignore = [](auto&, const auto&) {};
  service<srv::Arm>("arm", [](const auto& r) {
    Job j{JobKind::arm}; j.confirm = r.confirm; j.code = r.authority; j.owner = r.owner; return j;
  }, ignore);
  service<srv::ResetFault>("reset_fault", [](const auto& r) {
    Job j{JobKind::reset}; j.confirm = r.confirm; return j;
  }, ignore);
  service<srv::DiscoverMotor>("discover_motor", [](const auto&) { return Job{JobKind::discover}; },
      [](auto& r, const auto& v) { r.motor_id = v.id; r.raw = v.raw; });
  service<srv::ReadParameter>("read_parameter", [](const auto& r) {
    Job j{JobKind::query}; j.code = r.code; j.group = r.group; return j;
  }, [](auto& r, const auto& v) { r.value = v.value; r.raw = v.raw; });
  service<srv::WriteParameter>("write_parameter", [](const auto& r) {
    Job j{JobKind::configure}; j.code = r.code; j.value = r.value; j.confirm = r.confirm; return j;
  }, [](auto& r, const auto& v) { r.acknowledged = v.acknowledged; r.readback = v.value; r.raw = v.raw; });
  service<srv::SetZero>("set_zero", [](const auto& r) {
    Job j{JobKind::zero}; j.confirm = r.confirm; j.option = r.use_offset; j.number = r.offset_rad; return j;
  }, ignore);
  service<srv::SetMotorId>("set_motor_id", [](const auto& r) {
    Job j{JobKind::set_id}; j.confirm = r.confirm; j.id = r.new_id; return j;
  }, ignore);
  service<srv::ResetMotorId>("reset_motor_id", [](const auto& r) {
    Job j{JobKind::reset_id}; j.confirm = r.confirm; return j;
  }, ignore);
  service<srv::SetBrake>("set_brake", [](const auto& r) {
    Job j{JobKind::brake}; j.confirm = r.confirm; j.option = r.release; j.secondary = r.electromagnetic; return j;
  }, ignore);
  services_.push_back(create_service<srv::GetProfile>("/encos/get_profile",
      [this](std::shared_ptr<srv::GetProfile::Request>, std::shared_ptr<srv::GetProfile::Response> r) {
    r->success = snapshot_.state.connected;
    r->detail = snapshot_.detail;
    r->profile = snapshot_.profile;
    r->state = snapshot_.state;
  }));
  services_.push_back(create_service<srv::Stop>("/encos/stop",
      [this](std::shared_ptr<srv::Stop::Request>, std::shared_ptr<srv::Stop::Response> r) {
    worker_->request_stop();
    r->success = true;
    r->detail = "stop queued; inspect state for virtual watchdog confirmation";
  }));

  actions_ = rclcpp_action::create_server<Experiment>(this, "/encos/run_experiment",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Experiment::Goal> goal) {
    if (active_action_ || action_admission_ || !valid_goal(*goal)) return rclcpp_action::GoalResponse::REJECT;
    action_admission_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }, [this](std::shared_ptr<GoalHandle> handle) {
    if (handle != active_action_) return rclcpp_action::CancelResponse::REJECT;
    worker_->request_cancel(action_token_);
    return rclcpp_action::CancelResponse::ACCEPT;
  }, [this](std::shared_ptr<GoalHandle> handle) {
    action_admission_ = false;
    active_action_ = std::move(handle);
    const auto& g = *active_action_->get_goal();
    M command;
    command.owner = g.owner;
    command.motor_id = snapshot_.state.motor_id;
    command.mode = g.mode;
    command.speed_limit_rad_s = g.speed_limit_rad_s;
    command.current_limit_a = g.current_limit_a;
    command.feedback = g.feedback;
    const auto wave = experiments::Trajectory{static_cast<experiments::Shape>(g.shape), g.duration_s,
          g.offset, g.amplitude, g.start_hz, g.end_hz, g.step_at_s};
    if (!worker_->submit_goal({++action_token_, command, wave})) {
      action_update({action_token_, 0, 0, 0, 0, ActionUpdate::aborted, "worker unavailable"});
    }
  });
  pump_ = create_wall_timer(Milliseconds(config_.publish_ms), [this] { drain(); });
  RCLCPP_INFO(get_logger(), "vcan-only driver on %s, %u Hz", config_.interface.c_str(), config_.frequency);
}

MotorNode::~MotorNode() {
  pump_.reset();
  worker_.reset();
}

bool MotorNode::valid_goal(const Experiment::Goal& g) const {
  if (snapshot_.state.authority != static_cast<unsigned>(Authority::action) ||
      snapshot_.state.owner != g.owner || g.owner == 0 || g.feedback > 3 ||
      (g.mode != M::POSITION && g.mode != M::VELOCITY)) return false;
  const experiments::Trajectory wave{static_cast<experiments::Shape>(g.shape), g.duration_s,
      g.offset, g.amplitude, g.start_hz, g.end_hz, g.step_at_s};
  const auto& limits = config_.limits;
  if (!wave.valid() || g.duration_s > std::chrono::duration<double>(limits.run_duration).count() ||
      !std::isfinite(g.current_limit_a) || g.current_limit_a < 0 || g.current_limit_a > limits.phase_current_a ||
      !std::isfinite(g.speed_limit_rad_s) || g.speed_limit_rad_s < 0 ||
      g.speed_limit_rad_s > limits.velocity_rad_s) return false;
  const double excursion = std::abs(g.amplitude);
  return g.mode == M::POSITION
      ? limits.position_rad.contains(g.offset - excursion) && limits.position_rad.contains(g.offset + excursion) &&
            wave.periodic_slope_bound() <= g.speed_limit_rad_s
      : std::abs(g.offset) + excursion <= limits.velocity_rad_s;
}

void MotorNode::command(const M& value, const rclcpp::MessageInfo& info) {
  if (snapshot_.state.authority != static_cast<unsigned>(Authority::topic) || value.owner == 0 ||
      snapshot_.state.owner != value.owner || snapshot_.state.motor_id != value.motor_id) {
    worker_->count_rejected();
    return;
  }
  const auto ros = now();
  const auto receipt = Clock::now();
  const auto source_ns = static_cast<std::int64_t>(value.header.stamp.sec) * ns_per_sec + value.header.stamp.nanosec;
  const auto ttl_ns = static_cast<std::int64_t>(value.valid_for.sec) * ns_per_sec + value.valid_for.nanosec;
  const auto limit_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(config_.limits.command_age).count();
  const bool well_formed = source_ns > 0 && value.header.stamp.nanosec < ns_per_sec &&
      value.valid_for.sec >= 0 && value.valid_for.nanosec < ns_per_sec && ttl_ns > 0 &&
      ros.nanoseconds() > 0 && source_ns <= ros.nanoseconds() + 1'000'000;
  const auto age = well_formed ? std::max<std::int64_t>(0, ros.nanoseconds() - source_ns) : 0;
  const auto remaining = std::min(ttl_ns, limit_ns) - age;
  std::array<std::uint8_t, RMW_GID_STORAGE_SIZE> gid{};
  const auto& from = info.get_rmw_message_info().publisher_gid;
  std::copy_n(from.data, gid.size(), gid.begin());
  if (publisher_gid_ && *publisher_gid_ != gid) {
    worker_->count_rejected();
    return;
  }
  if (well_formed && remaining > 0 && source_ns > last_source_stamp_) {
    last_source_stamp_ = source_ns;
    publisher_gid_ = gid;
  } else {
    worker_->count_expired();
    return;
  }
  auto copy = value;
  copy.header.frame_id.clear();
  worker_->submit_target({std::move(copy), receipt - std::chrono::nanoseconds(age), receipt,
                          receipt + std::chrono::nanoseconds(remaining), ros, true, true});
}

void MotorNode::publish_state(msg::MotorState state, const rclcpp::Time& time) {
  state.header.stamp = time;
  state.header.frame_id = config_.joint;
  if (states_) states_->publish(state);
  if (joints_ && (state.valid[0] || state.valid[1])) {
    sensor_msgs::msg::JointState joint;
    joint.header.stamp = time;
    joint.name = {config_.joint};
    if (state.valid[0]) joint.position = {state.position_rad};
    if (state.valid[1]) joint.velocity = {state.velocity_rad_s};
    joints_->publish(joint);
  }
}

void MotorNode::diagnostics(Snapshot snapshot) {
  const auto previous_authority = snapshot_.state.authority;
  const auto previous_owner = snapshot_.state.owner;
  snapshot.state.header.stamp = now();
  snapshot_ = snapshot;
  if (snapshot.state.authority != previous_authority || snapshot.state.owner != previous_owner) {
    publisher_gid_.reset();
    last_source_stamp_ = 0;
  }
  const auto state = static_cast<SafetyState>(snapshot.state.safety_state);
  const auto fault_key = std::make_pair(state, snapshot.fault);
  if (!last_fault_ || *last_fault_ != fault_key) {
    msg::MotorFault fault;
    fault.header.stamp = snapshot.state.header.stamp;
    fault.motor_id = snapshot.state.motor_id;
    fault.motor_error = snapshot.state.motor_error;
    fault.safety_state = snapshot.state.safety_state;
    fault.software_fault = static_cast<std::uint8_t>(snapshot.fault);
    fault.latched = state == SafetyState::fault ||
          (state == SafetyState::stopping && snapshot.fault != SafetyFault::stopped);
    fault.detail = snapshot.detail;
    faults_->publish(fault);
    last_fault_ = fault_key;
  }
  publish_state(snapshot.state, now());
  if (previous_stats_ && snapshot.stamp - previous_stats_->stamp < 100ms) return;
  msg::CommunicationStats stats;
  stats.header.stamp = now();
  const auto& c = snapshot.counters;
  stats.target_hz = config_.frequency;
  stats.tx = c.tx; stats.rx = c.rx; stats.matched = c.matched; stats.expected_replies = c.expected;
  stats.timeouts = c.timeouts; stats.invalid_rx = c.invalid; stats.ignored_rx = c.ignored;
  stats.command_received = c.command_received; stats.dropped_commands = c.dropped;
  stats.expired_commands = c.expired; stats.skipped_cycles = c.skipped; stats.overruns = c.overruns;
  stats.output_overflows = c.overflow; stats.socket_rx_drops = c.socket_drops;
  if (previous_stats_) {
    const auto seconds = std::chrono::duration<double>(snapshot.stamp - previous_stats_->stamp).count();
    stats.tx_hz = (c.tx - previous_stats_->counters.tx) / seconds;
    stats.feedback_hz = (c.matched - previous_stats_->counters.matched) / seconds;
  }
  std::vector<double> rtts(rtts_.begin(), rtts_.end()), jitter(jitters_.begin(), jitters_.end());
  std::sort(rtts.begin(), rtts.end());
  std::sort(jitter.begin(), jitter.end());
  stats.rtt_p50_us = percentile(rtts, .5); stats.rtt_p95_us = percentile(rtts, .95);
  stats.rtt_p99_us = percentile(rtts, .99); stats.rtt_max_us = percentile(rtts, 1.);
  stats.cycle_jitter_p99_us = percentile(jitter, .99);
  stats_->publish(stats);
  previous_stats_ = std::move(snapshot);
}

void MotorNode::action_update(const ActionUpdate& update) {
  if (!active_action_ || update.token != action_token_) return;
  if (update.stage == ActionUpdate::running) {
    auto feedback = std::make_shared<Experiment::Feedback>();
    feedback->elapsed_s = update.elapsed;
    feedback->progress = update.progress;
    feedback->target = update.target;
    feedback->samples = update.samples;
    active_action_->publish_feedback(feedback);
    return;
  }
  auto result = std::make_shared<Experiment::Result>();
  result->success = update.stage == ActionUpdate::success;
  result->samples = update.samples;
  result->detail = update.detail;
  if (active_action_->is_canceling()) active_action_->canceled(result);
  else if (result->success) active_action_->succeed(result);
  else active_action_->abort(result);
  active_action_.reset();
}

void MotorNode::drain() {
  // Bounded work per callback. Worker never calls ROS or waits for this callback.
  for (unsigned i = 0; i < 32; ++i) {
    auto value = worker_->take_status();
    if (!value) break;
    diagnostics(std::move(*value));
  }
  if (pending_service_) {
    if (auto result = worker_->take_job()) {
      auto reply = std::move(pending_service_);
      pending_service_ = {};
      reply(*result);
    }
  }
  for (unsigned i = 0; i < 32; ++i) {
    auto update = worker_->take_action();
    if (!update) break;
    action_update(*update);
  }
  for (unsigned i = 0; i < 64; ++i) {
    auto sample = worker_->take_sample();
    if (!sample) break;
    if (sample->matched && sample->feedback.rx_mono_ns) {
      add_metric(rtts_, (sample->feedback.rx_mono_ns - sample->tx_call_before_mono_ns) / 1000.);
    }
    if (sample->previous_cycle_interval_ns) {
      add_metric(jitters_, std::abs(sample->previous_cycle_interval_ns - ns_per_sec / config_.frequency) / 1000.);
    }
    const auto time = now();
    sample->header.stamp = time;
    sample->feedback.header.stamp = time;
    sample->ros_publish_mono_ns = mono(Clock::now());
    if (sample->feedback.rx_mono_ns && config_.publish_state) publish_state(sample->feedback, time);
    if (samples_) samples_->publish(*sample);
  }
  if (config_.raw) {
    for (unsigned i = 0; i < 128; ++i) {
      auto raw = worker_->take_raw();
      if (!raw) break;
      raw->frame.header.stamp = now();
      (raw->tx ? raw_tx_ : raw_rx_)->publish(raw->frame);
    }
  }
}
}  // namespace encos::driver
