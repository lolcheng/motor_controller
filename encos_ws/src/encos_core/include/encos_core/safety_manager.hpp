#pragma once

#include "encos_core/protocol.hpp"

namespace encos {

enum class SafetyState { disconnected, unarmed, read_only, armed, running, stopping, fault };
enum class SafetyFault { none, stopped, disconnected, bus, motor, invalid_command, limits,
                         stale_command, stale_feedback, duration };

using Owner = std::uint64_t;
using Milliseconds = std::chrono::milliseconds;

struct Limits {
  Range position_rad;
  double velocity_rad_s{}, phase_current_a{}, torque_nm{}, motor_c{}, kp{}, kd{};
  std::optional<double> mos_c;
  Milliseconds command_age{}, feedback_age{}, run_duration{};
  // Dedicated authorization is required for non-position/velocity/mixed modes.
  bool allow_current{}, allow_torque{}, allow_damping{}, allow_dissipative{}, allow_regenerative{}, allow_brake{};
  bool valid() const;
};

struct VerifiedMotor {
  std::uint16_t id{};
  WireFormat format{WireFormat::classic};
  std::optional<Version> version;
  std::uint16_t timeout_ms{};
  std::optional<bool> brake_present;
  std::optional<BrakeState> brake_state;
  bool stop_strategy_confirmed{};
  ProtocolProfile protocol;
  bool ready() const;
};

class SafetyManager {
 public:
  explicit SafetyManager(Limits limits) : limits_(limits) {}
  void connect();
  void disconnect();
  bool authorize_read_only(bool operator_confirmed);
  bool allows_discovery() const;
  bool allows_query(QueryCode code) const;
  bool set_verified_motor(const VerifiedMotor& motor, TimePoint observed);
  bool arm(Owner owner, bool operator_confirmed, TimePoint now);
  std::optional<Encoded> submit(Owner owner, const Control& command, TimePoint generated, TimePoint now);
  void observe(const ReceivedFrame& frame, TimePoint now);
  void tick(TimePoint now);
  void bus_fault();
  void command_expired();
  void invalid_command();
  void motor_fault();
  void stop();
  bool confirm_stop(bool operator_verified);
  bool reset_fault(bool operator_confirmed);
  SafetyState state() const { return state_; }
  SafetyFault fault() const { return fault_; }

 private:
  bool active() const;
  bool feedback_fresh(TimePoint now) const;
  bool update_measurement(std::optional<Measurement>& current,
                          const std::optional<Measurement>& incoming);
  void latch(SafetyFault fault);

  Limits limits_;
  SafetyState state_{SafetyState::disconnected};
  SafetyFault fault_{SafetyFault::none};
  bool connected_{};
  std::optional<VerifiedMotor> motor_;
  Telemetry feedback_;
  Owner owner_{};
  std::optional<TimePoint> verified_at_, armed_at_, started_at_, last_command_;
};

}  // namespace encos
