#include "encos_tools/runtime.hpp"
#include "encos_tools/virtual_motor.hpp"

#include <csignal>
#include <ctime>
#include <iostream>

namespace {
using namespace encos;
using namespace encos::tools;
using namespace std::chrono_literals;

volatile std::sig_atomic_t running = 1;
void halt(int) { running = 0; }

void fault(std::string_view name, ModelOptions& model, FaultOptions& faults) {
  if (name == "silent") faults.silent = true;
  else if (name == "duplicate") faults.duplicate = true;
  else if (name == "wrong-id") faults.wrong_id = true;
  else if (name == "length") faults.short_length = true;
  else if (name == "type") faults.unknown_type = true;
  else if (name == "uuid-missing") faults.uuid_missing = true;
  else if (name == "uuid-reverse") faults.uuid_reverse = true;
  else if (name == "hot") model.motor_c = 90;
  else if (name == "reject-writes") model.reject_writes = true;
  else if (name.substr(0, 5) == "drop:") faults.drop_every = parse_unsigned(name.substr(5), 10000);
  else if (name.substr(0, 6) == "delay:") faults.delay = std::chrono::milliseconds(parse_unsigned(name.substr(6), 10000));
  else if (name.substr(0, 6) == "motor:") model.error = static_cast<MotorError>(parse_unsigned(name.substr(6), 7));
  else throw std::invalid_argument("unknown fault");
}
}  // namespace

int main(int argc, char** argv) try {
  using namespace encos;
  using namespace encos::tools;
  using namespace std::chrono_literals;
  if (argc > 1 && std::string_view(argv[1]) == "--help") {
    std::cout << "Usage: encos_virtual_motor [vcan0] [classic|fd] [--id N] [--idle-beacon|--bootstrap-beacon] [--fault NAME]\n";
    return 0;
  }
  const std::string interface = argc > 1 ? argv[1] : "vcan0";
  ModelOptions model;
  if (argc > 2) model.format = parse_format(argv[2]);
  FaultOptions faults;
  bool beacon = false;
  bool bootstrap = false;
  for (int i = 3; i < argc; ++i) {
    const std::string_view option(argv[i]);
    if (option == "--idle-beacon") beacon = true;
    else if (option == "--bootstrap-beacon") bootstrap = true;
    else if (option == "--id" && i + 1 < argc) model.id = static_cast<std::uint16_t>(parse_unsigned(argv[++i], 0x7fe));
    else if (option == "--fault" && i + 1 < argc) fault(argv[++i], model, faults);
    else throw std::invalid_argument("unknown or incomplete option");
  }
  if (beacon && bootstrap) throw std::invalid_argument("choose one beacon mode");
  std::signal(SIGINT, halt);
  std::signal(SIGTERM, halt);
  SocketCanTransport bus({interface, model.format, {}, true});
  VirtualMotor motor(model);
  FaultQueue replies(faults);
  auto next_beacon = Clock::now();
  const auto started = next_beacon;
  const auto cpu_start = std::clock();
  std::size_t requests{}, transmitted{}, send_errors{};
  std::cout << "ready " << interface << " virtual motor id " << model.id << std::endl;
  while (running) {
    const auto now = Clock::now();
    motor.advance(now);
    while (const auto reply = replies.take(now)) {
      if (bus.send(*reply) == SendStatus::accepted) ++transmitted;
      else ++send_errors;
    }
    if (beacon && now >= next_beacon) {
      if (!motor.state().active && bus.send(motor.idle_telemetry()) != SendStatus::accepted) ++send_errors;
      next_beacon = now + 10ms;
    }
    auto deadline = now + 20ms;
    if (beacon) deadline = std::min(deadline, next_beacon);
    if (const auto due = replies.next()) deadline = std::min(deadline, *due);
    const auto read = bus.receive_for(std::max(Clock::duration::zero(), deadline - Clock::now()));
    if (read.status == ReceiveStatus::disconnected || read.status == ReceiveStatus::io_error) {
      std::cerr << "virtual motor link unavailable\n";
      return 1;
    }
    if (read.sample && !read.sample->frame.error) {
      ++requests;
      const auto arrival = Clock::now();
      auto response = motor.handle(*read.sample, arrival);
      const auto& input = read.sample->frame;
      if (bootstrap && !motor.state().active && input.length == 2 &&
          input.data[0] == 0xe1 && input.data[1] == 30 && response.size() == 1) {
        // Explicit simulation-only initialization feedback. Never emitted during a control reply.
        auto initial = motor.idle_telemetry();
        replies.enqueue({initial}, arrival);
      }
      replies.enqueue(std::move(response), arrival);
    }
  }
  const auto elapsed = std::chrono::duration<double>(Clock::now() - started).count();
  const auto cpu = static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
  std::cout << "requests=" << requests << " replies=" << transmitted << " injected_or_queue_drops="
            << replies.dropped() << " send_errors=" << send_errors << " cpu_percent="
            << (elapsed > 0 ? 100 * cpu / elapsed : 0) << '\n';
  return send_errors ? 1 : 0;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n';
  return 2;
}
