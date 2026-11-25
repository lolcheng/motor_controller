#include "encos_tools/runtime.hpp"

#include <csignal>
#include <iostream>

namespace {
volatile std::sig_atomic_t running = 1;
void halt(int) { running = 0; }
}

int main(int argc, char** argv) try {
  using namespace encos;
  using namespace encos::tools;
  using namespace std::chrono_literals;
  if (argc > 3 || (argc > 1 && std::string_view(argv[1]) == "--help")) {
    std::cout << "Usage: encos_bus_monitor [vcan0] [classic|fd]\n";
    return argc > 3 ? 2 : 0;
  }
  std::signal(SIGINT, halt);
  std::signal(SIGTERM, halt);
  SocketCanTransport bus({argc > 1 ? argv[1] : "vcan0",
                          argc > 2 ? parse_format(argv[2]) : WireFormat::fd_brs, {}, true});
  while (running) {
    const auto read = bus.receive_for(100ms);
    if (read.sample) {
      const auto& raw = *read.sample;
      std::cout << ns(raw.stamp) << ' ' << (raw.frame.kind == FrameKind::fd ? "FD" : "CAN")
                << (raw.frame.brs ? " BRS" : "") << (raw.frame.esi ? " ESI" : "")
                << (raw.frame.error ? " ERROR" : "") << ' ' << hex(raw.frame) << std::endl;
    } else if (read.status == ReceiveStatus::disconnected || read.status == ReceiveStatus::io_error) {
      return 1;
    }
  }
  return 0;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n';
  return 2;
}
