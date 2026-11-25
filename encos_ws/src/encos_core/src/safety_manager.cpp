#include "encos_core/safety_manager.hpp"

#include <cmath>
#include <type_traits>

namespace encos {
namespace {

bool positive(double value) {
  return std::isfinite(value) && value > 0;
}

bool valid_duration(Milliseconds value) {
  return value > Milliseconds::zero() && value <= std::chrono::duration_cast<Milliseconds>(Clock::duration::max());
}

bool fresh(TimePoint stamp, TimePoint now, Milliseconds age) {
  return stamp <= now && (stamp > TimePoint::max() - age || now <= stamp + age);
}

Control wire_command(const Control& control, const CanFrame& frame, const ProtocolProfile& profile) {
  const auto* data = frame.data.data();
  return std::visit([&](auto cmd) -> Control {
    using T = std::decay_t<decltype(cmd)>;
    if constexpr (std::is_same_v<T, Mixed>) {
      const auto& range = *profile.mixed;
      const auto decode = [&](unsigned offset, unsigned width, const Range& interval) {
        const double fraction = static_cast<double>(wire::read_bits(data, offset, width)) / ((1u << width) - 1);
        return interval.min + fraction * (interval.max - interval.min);
      };
      cmd.kp = decode(3, 12, range.kp);
      cmd.kd = decode(15, 9, range.kd);
      cmd.position_rad = decode(24, 16, range.position_rad);
      cmd.velocity_rad_s = decode(40, 12, range.velocity_rad_s);
      cmd.torque_nm = decode(52, 12, range.torque_nm);
    } else if constexpr (std::is_same_v<T, Position>) {
      cmd.position_rad = wire::float_value(wire::read_bits(data, 3, 32)) * wire::degrees_to_rad;
      cmd.speed_limit_rad_s = wire::read_bits(data, 35, 15) / (10 / wire::rpm_to_rad_s);
      cmd.current_limit_a = wire::read_bits(data, 50, 12) / 10.0;
    } else if constexpr (std::is_same_v<T, Velocity>) {
      cmd.velocity_rad_s = wire::read_float(data + 1) * wire::rpm_to_rad_s;
      cmd.current_limit_a = wire::read_u16(data + 5) / 10.0;
    } else if (cmd.mode != DriveMode::brake) {
      cmd.value = wire::read_i16(data + 1) / 100.0;
    }
    return cmd;
  }, control);
}

SafetyFault test_command(const Control& control, const Limits& limits) {
  return std::visit([&](const auto& cmd) -> SafetyFault {
    using T = std::decay_t<decltype(cmd)>;
    if constexpr (std::is_same_v<T, Mixed>) {
      if (!std::isfinite(cmd.position_rad) || !std::isfinite(cmd.velocity_rad_s) ||
          !std::isfinite(cmd.torque_nm) || !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd)) {
        return SafetyFault::invalid_command;
      }
      if (!limits.position_rad.contains(cmd.position_rad) || std::abs(cmd.velocity_rad_s) > limits.velocity_rad_s ||
          std::abs(cmd.torque_nm) > limits.torque_nm || cmd.kp < 0 || cmd.kp > limits.kp ||
          cmd.kd < 0 || cmd.kd > limits.kd) {
        return SafetyFault::limits;
      }
    } else if constexpr (std::is_same_v<T, Position> || std::is_same_v<T, Velocity>) {
      const double speed = [&] {
        if constexpr (std::is_same_v<T, Position>) {
          return cmd.speed_limit_rad_s;
        } else {
          return std::abs(cmd.velocity_rad_s);
        }
      }();
      if (!std::isfinite(speed) || !std::isfinite(cmd.current_limit_a) || static_cast<unsigned>(cmd.ack) > 3) {
        return SafetyFault::invalid_command;
      }
      if constexpr (std::is_same_v<T, Position>) {
        if (!std::isfinite(cmd.position_rad)) {
          return SafetyFault::invalid_command;
        }
        if (!limits.position_rad.contains(cmd.position_rad)) {
          return SafetyFault::limits;
        }
      }
      if (speed < 0 || speed > limits.velocity_rad_s ||
          cmd.current_limit_a < 0 || cmd.current_limit_a > limits.phase_current_a) {
        return SafetyFault::limits;
      }
    } else {
      if (!std::isfinite(cmd.value) || static_cast<unsigned>(cmd.ack) > 3) {
        return SafetyFault::invalid_command;
      }
      bool allowed = false;
      switch (cmd.mode) {
        case DriveMode::current: allowed = limits.allow_current && std::abs(cmd.value) <= limits.phase_current_a; break;
        case DriveMode::torque: allowed = limits.allow_torque && std::abs(cmd.value) <= limits.torque_nm; break;
        case DriveMode::damping: allowed = limits.allow_damping && cmd.value == 0; break;
        case DriveMode::dissipative:
          allowed = limits.allow_dissipative && cmd.value >= 0 && cmd.value <= limits.phase_current_a;
          break;
        case DriveMode::regenerative:
          allowed = limits.allow_regenerative && cmd.value >= 0 && cmd.value <= limits.phase_current_a;
          break;
        case DriveMode::brake: allowed = limits.allow_brake && (cmd.value == 0 || cmd.value == 1); break;
        default: return SafetyFault::invalid_command;
      }
      if (!allowed) {
        return SafetyFault::limits;
      }
    }
    return SafetyFault::none;
  }, control);
}

}  // namespace

bool Limits::valid() const {
  return position_rad.valid() && positive(velocity_rad_s) && positive(phase_current_a) && positive(torque_nm) &&
         positive(motor_c) && positive(kp) && positive(kd) && (!mos_c || positive(*mos_c)) &&
         valid_duration(command_age) && valid_duration(feedback_age) && valid_duration(run_duration);
}

bool VerifiedMotor::ready() const {
  return valid_motor_id(id) && valid_wire_format(format) && version && timeout_ms > 0 && brake_present &&
         (!*brake_present || brake_state == BrakeState::released) && stop_strategy_confirmed &&
         protocol.mixed && protocol.mixed->valid() && protocol.feedback1 && protocol.feedback1->valid() &&
         protocol.kt_nm_per_a && positive(*protocol.kt_nm_per_a) &&
         protocol.query35_raw && *protocol.query35_raw >= 1 && *protocol.query35_raw <= 3;
}

void SafetyManager::connect() {
  if (connected_) {
    return;
  }
  connected_ = true;
  if (state_ == SafetyState::disconnected) {
    state_ = SafetyState::unarmed;
  }
}

void SafetyManager::disconnect() {
  connected_ = false;
  motor_.reset();
  verified_at_.reset();
  feedback_ = {};
  if (active()) {
    latch(SafetyFault::disconnected);
  } else if (state_ != SafetyState::fault && state_ != SafetyState::stopping) {
    state_ = SafetyState::disconnected;
  }
}

bool SafetyManager::authorize_read_only(bool operator_confirmed) {
  if (!operator_confirmed || state_ != SafetyState::unarmed) {
    return false;
  }
  state_ = SafetyState::read_only;
  return true;
}

bool SafetyManager::allows_discovery() const {
  return state_ == SafetyState::read_only;
}

bool SafetyManager::allows_query(QueryCode code) const {
  return allows_discovery() && parameter(code);
}

bool SafetyManager::set_verified_motor(const VerifiedMotor& motor, TimePoint observed) {
  if (state_ != SafetyState::unarmed && state_ != SafetyState::read_only) {
    return false;
  }
  feedback_ = {};
  motor_.reset();
  verified_at_.reset();
  if (!motor.ready()) {
    return false;
  }
  motor_ = motor;
  verified_at_ = observed;
  return true;
}

bool SafetyManager::arm(Owner owner, bool operator_confirmed, TimePoint now) {
  if ((state_ != SafetyState::read_only && state_ != SafetyState::unarmed) ||
      !operator_confirmed || !owner || !limits_.valid() || !motor_ || !feedback_fresh(now)) {
    return false;
  }
  state_ = SafetyState::armed;
  owner_ = owner;
  armed_at_ = now;
  started_at_.reset();
  last_command_.reset();
  return true;
}

std::optional<Encoded> SafetyManager::submit(Owner owner, const Control& command, TimePoint generated, TimePoint now) {
  if (!active() || owner != owner_) {
    return {};
  }
  tick(now);
  if (!active()) {
    return {};
  }
  if (generated < *armed_at_ || !fresh(generated, now, limits_.command_age) ||
      (last_command_ && generated <= *last_command_)) {
    latch(SafetyFault::stale_command);
    return {};
  }
  const auto checked = test_command(command, limits_);
  if (checked != SafetyFault::none) {
    latch(checked);
    return {};
  }
  if (const auto* drive = std::get_if<Drive>(&command); drive && drive->mode == DriveMode::brake &&
      motor_->brake_present != true) {
    latch(SafetyFault::limits);
    return {};
  }
  Encoded encoded;
  try {
    encoded = Protocol{motor_->format}.control(motor_->id, command, motor_->protocol);
    if (test_command(wire_command(command, encoded.frame, motor_->protocol), limits_) != SafetyFault::none) {
      latch(SafetyFault::limits);
      return {};
    }
  } catch (const CodecError&) {
    latch(SafetyFault::invalid_command);
    return {};
  }
  if (!started_at_) {
    started_at_ = now;
  }
  last_command_ = generated;
  state_ = SafetyState::running;
  return encoded;
}

bool SafetyManager::update_measurement(std::optional<Measurement>& current,
                                       const std::optional<Measurement>& incoming) {
  if (!incoming) {
    return true;
  }
  if (!std::isfinite(incoming->value)) {
    return false;
  }
  if (!current || incoming->host_rx > current->host_rx) {
    current = incoming;
  }
  return true;
}

void SafetyManager::observe(const ReceivedFrame& sample, TimePoint now) {
  if (!motor_ || sample.origin != Origin::remote || sample.frame.id != motor_->id ||
      sample.stamp < *verified_at_ || sample.stamp > now ||
      state_ == SafetyState::stopping || state_ == SafetyState::fault) {
    return;
  }
  const auto frame = Protocol{motor_->format}.decode(sample, motor_->protocol);
  const auto* issue = std::get_if<CodecIssue>(&frame.feedback);
  const auto* query = std::get_if<QueryReply>(&frame.feedback);
  const auto* config = std::get_if<ConfigStatus>(&frame.feedback);
  const auto* brake = std::get_if<BrakeReply>(&frame.feedback);
  if ((issue && *issue == CodecIssue::nonfinite) || (query && query->error != MotorError::none) ||
      (config && config->error != MotorError::none) ||
      (brake && brake->error && brake->error != MotorError::none)) {
    latch(SafetyFault::motor);
    return;
  }
  const auto* data = std::get_if<Telemetry>(&frame.feedback);
  if (!data) {
    return;
  }
  if (data->error != MotorError::none) {
    latch(SafetyFault::motor);
    return;
  }
  if (!update_measurement(feedback_.position_rad, data->position_rad) ||
      !update_measurement(feedback_.velocity_rad_s, data->velocity_rad_s) ||
      !update_measurement(feedback_.phase_current_a, data->phase_current_a) ||
      !update_measurement(feedback_.motor_c, data->motor_c) ||
      !update_measurement(feedback_.mos_c, data->mos_c) ||
      !update_measurement(feedback_.estimated_torque_nm, data->estimated_torque_nm)) {
    latch(SafetyFault::motor);
    return;
  }
  const auto above = [](const auto& value, double limit) { return value && value->value > limit; };
  const auto abs_above = [](const auto& value, double limit) { return value && std::abs(value->value) > limit; };
  if ((feedback_.position_rad && !limits_.position_rad.contains(feedback_.position_rad->value)) ||
      abs_above(feedback_.velocity_rad_s, limits_.velocity_rad_s) ||
      abs_above(feedback_.phase_current_a, limits_.phase_current_a) ||
      abs_above(feedback_.estimated_torque_nm, limits_.torque_nm) ||
      above(feedback_.motor_c, limits_.motor_c) || (limits_.mos_c && above(feedback_.mos_c, *limits_.mos_c))) {
    latch(SafetyFault::limits);
  }
}

bool SafetyManager::feedback_fresh(TimePoint now) const {
  const auto recent = [&](const auto& value) { return value && fresh(value->host_rx, now, limits_.feedback_age); };
  return limits_.valid() && recent(feedback_.position_rad) && recent(feedback_.velocity_rad_s) &&
         recent(feedback_.phase_current_a) && recent(feedback_.motor_c) &&
         recent(feedback_.estimated_torque_nm) && (!limits_.mos_c || recent(feedback_.mos_c));
}

void SafetyManager::tick(TimePoint now) {
  if (!active()) {
    return;
  }
  if (!feedback_fresh(now)) {
    latch(SafetyFault::stale_feedback);
  } else if (started_at_ && (*started_at_ > now ||
                            (*started_at_ <= TimePoint::max() - limits_.run_duration &&
                             now >= *started_at_ + limits_.run_duration))) {
    latch(SafetyFault::duration);
  } else if (last_command_ && !fresh(*last_command_, now, limits_.command_age)) {
    latch(SafetyFault::stale_command);
  }
}

void SafetyManager::bus_fault() {
  if (connected_ || active()) {
    latch(SafetyFault::bus);
  }
}

void SafetyManager::motor_fault() {
  if (connected_) {
    latch(SafetyFault::motor);
  }
}

void SafetyManager::invalid_command() {
  if (connected_) {
    latch(SafetyFault::invalid_command);
  }
}

void SafetyManager::command_expired() {
  if (active()) {
    latch(SafetyFault::stale_command);
  }
}

void SafetyManager::stop() {
  if (active()) {
    latch(SafetyFault::stopped);
  }
}

bool SafetyManager::confirm_stop(bool operator_verified) {
  if (!operator_verified || state_ != SafetyState::stopping) {
    return false;
  }
  state_ = SafetyState::fault;
  return true;
}

bool SafetyManager::reset_fault(bool operator_confirmed) {
  if (!operator_confirmed || state_ != SafetyState::fault) {
    return false;
  }
  fault_ = SafetyFault::none;
  state_ = connected_ ? SafetyState::unarmed : SafetyState::disconnected;
  feedback_ = {};
  motor_.reset();
  verified_at_.reset();
  return true;
}

bool SafetyManager::active() const {
  return state_ == SafetyState::armed || state_ == SafetyState::running;
}

void SafetyManager::latch(SafetyFault fault) {
  if (state_ == SafetyState::fault || state_ == SafetyState::stopping) {
    return;
  }
  state_ = active() ? SafetyState::stopping : SafetyState::fault;
  fault_ = fault;
  owner_ = 0;
}

}  // namespace encos
