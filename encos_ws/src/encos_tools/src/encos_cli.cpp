#include "encos_tools/runtime.hpp"
#include "encos_core/safety_manager.hpp"

#include <cmath>
#include <iostream>

namespace {
using namespace encos;
using namespace encos::tools;
using namespace std::chrono_literals;

void show(const ReceivedFrame& sample) {
  std::cout << "RX " << hex(sample.frame) << " mono_ns=" << ns(sample.stamp)
            << (sample.frame.brs ? " BRS" : "") << '\n';
}

TransactionEvent call(SocketCanTransport& bus, TransactionManager& transactions, const Protocol& codec,
                      const Encoded& command, const ProtocolProfile& profile = {}) {
  std::cout << "TX " << hex(command.frame) << (command.frame.brs ? " BRS" : "") << '\n';
  const auto result = exchange(bus, transactions, codec, command, profile, 80ms, show);
  if (result.event.state != TransactionState::completed) {
    throw std::runtime_error("virtual command failed or timed out");
  }
  return result.event;
}

QueryReply query(SocketCanTransport& bus, TransactionManager& transactions, const Protocol& codec,
                 std::uint16_t id, QueryCode code) {
  return std::get<QueryReply>(call(bus, transactions, codec, codec.query(id, code)).reply->feedback);
}

double rad_arg(const std::string& arg) {
  std::size_t length{};
  const double value = std::stod(arg, &length);
  if (length != arg.size() || !std::isfinite(value)) {
    throw std::invalid_argument("angle/speed must be finite");
  }
  return value;
}

void demo(SocketCanTransport& bus, TransactionManager& tx, const Protocol& codec,
          std::uint16_t id, WireFormat format, const Control& control) {
  const auto list_range = [&](QueryCode code) {
    const auto pair = std::get<Pair>(query(bus, tx, codec, id, code).value);
    return Range{pair.first, pair.second};
  };
  VerifiedMotor motor;
  motor.id = id;
  motor.format = format;
  motor.version = std::get<Version>(query(bus, tx, codec, id, QueryCode::version).value);
  motor.timeout_ms = static_cast<std::uint16_t>(std::get<Scalar>(query(bus, tx, codec, id, QueryCode::timeout).value).value);
  motor.brake_state = static_cast<BrakeState>(std::get<Enumeration>(query(bus, tx, codec, id, QueryCode::brake).value).raw);
  motor.brake_present = true;  // This virtual motor intentionally emulates a brake.
  motor.stop_strategy_confirmed = true;  // Simulator watchdog verified in P1 tests.
  motor.protocol.kt_nm_per_a = std::get<Scalar>(query(bus, tx, codec, id, QueryCode::kt).value).value;
  motor.protocol.query35_raw = std::get<Enumeration>(query(bus, tx, codec, id, QueryCode::kt_calibration).value).raw;
  motor.protocol.mixed = MixedRanges{list_range(QueryCode::kp_range), list_range(QueryCode::kd_range),
      list_range(QueryCode::position_range), list_range(QueryCode::velocity_range), list_range(QueryCode::torque_range)};
  motor.protocol.feedback1 = Feedback1Ranges{motor.protocol.mixed->position_rad,
      motor.protocol.mixed->velocity_rad_s, list_range(QueryCode::current_range)};

  SafetyManager guard({{-0.5, 0.5}, 1.5, 3.0, 5.0, 65.0, 30.0, 3.0, 65.0, 40ms, 80ms, 200ms});
  guard.connect();
  if (!guard.set_verified_motor(motor, Clock::now())) {
    throw std::runtime_error("virtual profile is incomplete");
  }
  const auto bootstrap_end = Clock::now() + 100ms;
  bool armed = false;
  while (Clock::now() < bootstrap_end) {
    const auto raw = bus.receive_for(10ms);
    if (raw.sample) {
      guard.observe(*raw.sample, Clock::now());
      if (guard.arm(1, true, Clock::now())) {
        armed = true;
        break;
      }
    }
  }
  if (!armed) {
    throw std::runtime_error("arming requires fresh virtual telemetry; start the emulator with --idle-beacon");
  }
  const auto generated = Clock::now();
  const auto command = guard.submit(1, control, generated, Clock::now());
  if (!command) {
    throw std::runtime_error("virtual movement rejected by limits");
  }
  std::cout << "TX " << hex(command->frame) << '\n';
  const auto response = exchange(bus, tx, codec, *command, motor.protocol, 40ms,
      [&](const ReceivedFrame& sample) { show(sample); guard.observe(sample, Clock::now()); });
  if (response.event.state != TransactionState::completed || guard.state() != SafetyState::running) {
    guard.bus_fault();
    throw std::runtime_error("virtual movement failed");
  }
  guard.stop();
  std::cout << "virtual single-shot complete; simulated watchdog expires without new commands\n";
}

}  // namespace

int main(int argc, char** argv) try {
  if (argc < 4 || std::string_view(argv[1]) == "--help") {
    std::cout << "Usage: encos_cli vcan0 classic|fd discover|query CODE [GROUP]|position RAD --arm-virtual|velocity RAD_S --arm-virtual\n";
    return argc > 1 && std::string_view(argv[1]) == "--help" ? 0 : 2;
  }
  const auto format = parse_format(argv[2]);
  const std::string_view action(argv[3]);
  if (action == "discover" && argc != 4) throw std::invalid_argument("discover takes no arguments");
  const bool motion = action == "position" || action == "velocity";
  if (action != "discover" && action != "query" && !motion) throw std::invalid_argument("unknown command");
  if (motion && (argc != 6 || std::string_view(argv[5]) != "--arm-virtual")) {
    throw std::invalid_argument("virtual motion requires --arm-virtual");
  }
  if (action == "query" && (argc < 5 || argc > 6)) throw std::invalid_argument("query needs a code and optional group");
  SocketCanTransport bus({argv[1], format, {}, true});
  const Protocol codec(format);
  TransactionManager tx;
  const auto id = *std::get<ManagementReply>(call(bus, tx, codec, codec.discover()).reply->feedback).id;
  std::cout << "virtual motor ID " << id << '\n';
  if (action == "query") {
    const auto code = static_cast<QueryCode>(parse_unsigned(argv[4], 255));
    const auto group = static_cast<std::uint8_t>(argc == 6 ? parse_unsigned(argv[5], 10) : 0);
    call(bus, tx, codec, codec.query(id, code, group));
  }
  if (motion) {
    const auto value = rad_arg(argv[4]);
    const Control command = action == "position" ? Control{Position{value, 0.5, 1.0, Ack::type2}}
                                                : Control{Velocity{value, 1.0, Ack::type3}};
    demo(bus, tx, codec, id, format, command);
  }
  return 0;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n';
  return 1;
}
