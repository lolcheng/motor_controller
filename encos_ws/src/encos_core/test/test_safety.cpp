#include "check.hpp"
#include "encos_core/safety_manager.hpp"

using namespace test;
using S = SafetyState;
using F = SafetyFault;

namespace {
Limits limits() {
  return {{-1,1}, 2, 3, 4, 70, 25, 2, {}, 10ms, 20ms, 25ms};
}

VerifiedMotor motor() {
  return {id, WireFormat::classic, Version{{1,2,3},{4,5,6}}, 500, false, {}, true, profile()};
}

ReceivedFrame full(unsigned ms, Origin origin = Origin::remote) {
  return rx({0x20, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 100, 100}, id, ms, origin);
}

void arm(SafetyManager& safety) {
  safety.connect();
  CHECK(safety.set_verified_motor(motor(), at_ms(1)));
  safety.observe(full(2), at_ms(2));
  CHECK(safety.arm(10, true, at_ms(3)));
}

constexpr Position command{0.25, 1, 2, Ack::type2};
}  // namespace

int main() {
  SafetyManager s(limits());
  CHECK(s.state() == S::disconnected && !s.allows_discovery());
  s.connect();
  CHECK(s.state() == S::unarmed && !s.authorize_read_only(false) && !s.allows_discovery());
  CHECK(s.authorize_read_only(true) && s.allows_discovery() && s.allows_query(QueryCode::kt));
  CHECK(!s.allows_query(static_cast<QueryCode>(6)));
  CHECK(!s.arm(10, true, at_ms(1)));
  auto invalid = motor();
  invalid.protocol.feedback1.reset();
  CHECK(!s.set_verified_motor(invalid, at_ms(1)));
  invalid = motor();
  invalid.timeout_ms = 0;
  CHECK(!s.set_verified_motor(invalid, at_ms(1)));
  invalid = motor();
  invalid.brake_present = true;
  invalid.brake_state = BrakeState::engaged;
  CHECK(!s.set_verified_motor(invalid, at_ms(1)));
  invalid = motor();
  invalid.stop_strategy_confirmed = false;
  CHECK(!s.set_verified_motor(invalid, at_ms(1)));
  CHECK(s.set_verified_motor(motor(), at_ms(2)));
  s.observe(full(1), at_ms(3));
  s.observe(full(3, Origin::local_echo), at_ms(3));
  CHECK(!s.arm(10, true, at_ms(3)));
  s.observe(full(4), at_ms(4));
  CHECK(!s.arm(0, true, at_ms(4)) && !s.arm(10, false, at_ms(4)));
  CHECK(s.arm(10, true, at_ms(4)) && s.state() == S::armed && !s.allows_discovery());
  CHECK(!s.submit(11, command, at_ms(5), at_ms(5)) && s.state() == S::armed);
  const auto tx = s.submit(10, command, at_ms(5), at_ms(5));
  CHECK(tx && tx->reply.code == 2 && s.state() == S::running);
  CHECK(!s.submit(10, command, at_ms(5), at_ms(6)) && s.state() == S::stopping);
  CHECK(s.fault() == F::stale_command && !s.reset_fault(true));
  CHECK(!s.confirm_stop(false) && s.confirm_stop(true) && s.state() == S::fault);
  CHECK(!s.reset_fault(false) && s.reset_fault(true) && s.state() == S::unarmed);
  CHECK(!s.arm(10, true, at_ms(7)) && !s.submit(10, command, at_ms(7), at_ms(7)));

  SafetyManager stale(limits());
  arm(stale);
  CHECK(stale.submit(10, command, at_ms(4), at_ms(4)));
  stale.observe(rx({0x40, 0, 0, 0, 0, 0, 0, 100}, id, 14), at_ms(14));
  CHECK(stale.submit(10, command, at_ms(14), at_ms(14)));
  stale.observe(rx({0x40, 0, 0, 0, 0, 0, 0, 100}, id, 23), at_ms(23));
  stale.tick(at_ms(23));
  CHECK(stale.fault() == F::stale_feedback && stale.state() == S::stopping);

  SafetyManager command_loss(limits());
  arm(command_loss);
  CHECK(command_loss.submit(10, command, at_ms(4), at_ms(4)));
  command_loss.tick(at_ms(15));
  CHECK(command_loss.fault() == F::stale_command);

  SafetyManager duration(limits());
  arm(duration);
  for (const unsigned ms : {4, 12, 20, 28}) {
    duration.observe(full(ms), at_ms(ms));
    CHECK(duration.submit(10, command, at_ms(ms), at_ms(ms)));
  }
  duration.tick(at_ms(29));
  CHECK(duration.fault() == F::duration);

  for (const auto& unsafe : {Control(Position{quiet_nan, 0, 0}), Control(Velocity{0, inf}),
                             Control(Drive{static_cast<DriveMode>(6), 0})}) {
    SafetyManager guard(limits());
    arm(guard);
    CHECK(!guard.submit(10, unsafe, at_ms(5), at_ms(5)) && guard.fault() == F::invalid_command);
  }
  for (const auto& unsafe : {Control(Position{1.5, 0, 0}), Control(Velocity{3, 0}),
                             Control(Velocity{0, 4}), Control(Mixed{26, 0, 0, 0, 0}),
                             Control(Mixed{0, 0, 0, 0, 5}), Control(Drive{DriveMode::current, 0.01}),
                             Control(Drive{DriveMode::torque, 0.01}), Control(Drive{DriveMode::brake, 1})}) {
    SafetyManager guard(limits());
    arm(guard);
    CHECK(!guard.submit(10, unsafe, at_ms(5), at_ms(5)) && guard.fault() == F::limits);
  }
  auto rounded_limit = limits();
  rounded_limit.phase_current_a = 0.26;
  SafetyManager rounded(rounded_limit);
  arm(rounded);
  CHECK(!rounded.submit(10, Velocity{0, 0.26}, at_ms(5), at_ms(5)) && rounded.fault() == F::limits);

  for (const auto mode : {DriveMode::damping, DriveMode::dissipative, DriveMode::regenerative}) {
    SafetyManager guard(limits());
    arm(guard);
    CHECK(!guard.submit(10, Drive{mode, 0}, at_ms(5), at_ms(5)) && guard.fault() == F::limits);
  }

  SafetyManager bad_motor(limits());
  arm(bad_motor);
  bad_motor.observe(rx({0x2f,0x7f,0xff,0x7f,0xf7,0xff,100,100}, id, 6), at_ms(6));
  CHECK(bad_motor.state() == S::armed);
  bad_motor.observe(rx({0xa1, 2, 0, 0, 0, 0}, 2, 6), at_ms(6));
  CHECK(bad_motor.state() == S::armed);
  bad_motor.observe(rx({0xa1, 2, 0, 0, 0, 0}, id, 7), at_ms(7));
  CHECK(bad_motor.fault() == F::motor);
  SafetyManager bad_float(limits());
  arm(bad_float);
  bad_float.observe(rx({0x60, 0x7f, 0x80, 0, 0, 0, 0, 100}, id, 7), at_ms(7));
  CHECK(bad_float.fault() == F::motor);

  for (const auto& sample : {
      rx({0x40, 0x42, 0xb4, 0, 0, 0, 0, 100}, id, 6),
      rx({0x60, 0x42, 0x70, 0, 0, 0, 0, 100}, id, 6),
      rx({0x40, 0, 0, 0, 0, 0x01, 0x90, 100}, id, 6),
      rx({0x40, 0, 0, 0, 0, 0x00, 0xc8, 100}, id, 6),
      rx({0x40, 0, 0, 0, 0, 0, 0, 200}, id, 6)}) {
    SafetyManager guard(limits());
    arm(guard);
    guard.observe(sample, at_ms(6));
    CHECK(guard.fault() == F::limits);
  }
  SafetyManager lost(limits());
  arm(lost);
  lost.disconnect();
  CHECK(lost.state() == S::stopping && lost.fault() == F::disconnected);
  lost.connect();
  CHECK(lost.state() == S::stopping && !lost.arm(10, true, at_ms(7)));
  CHECK(lost.confirm_stop(true) && lost.reset_fault(true) && lost.state() == S::unarmed);
  CHECK(!lost.arm(10, true, at_ms(7)));
  lost.bus_fault();
  CHECK(lost.state() == S::fault && lost.fault() == F::bus);

  SafetyManager manual_stop(limits());
  arm(manual_stop);
  manual_stop.stop();
  CHECK(manual_stop.state() == S::stopping && manual_stop.fault() == F::stopped);
}
