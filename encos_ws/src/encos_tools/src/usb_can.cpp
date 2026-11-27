#include "cli_commands.hpp"
#include "encos_core/serial_can_transport.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <deque>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <unistd.h>
#include <vector>

namespace {
using namespace encos;
using namespace encos_tools;
using namespace std::chrono_literals;

void show_value(const ParameterValue& parameter) {
  std::visit([](const auto& value) {
    using V = std::decay_t<decltype(value)>;
    if constexpr (std::is_same_v<V, Scalar> || std::is_same_v<V, Raw16>) {
      std::cout << " value=" << value.value;
    } else if constexpr (std::is_same_v<V, Pair>) {
      std::cout << " values=" << value.first << ',' << value.second;
    } else if constexpr (std::is_same_v<V, Enumeration>) {
      std::cout << " raw_enum=" << unsigned(value.raw);
    } else if constexpr (std::is_same_v<V, UuidPart>) {
      std::ostringstream bytes;
      bytes << std::hex << std::uppercase << std::setfill('0');
      for (auto byte : value.bytes) bytes << std::setw(2) << unsigned(byte);
      std::cout << " uuid_part=" << unsigned(value.index) << " uuid_bytes=" << bytes.str();
    } else if constexpr (std::is_same_v<V, Version>) {
      std::cout << " hardware=";
      for (auto byte : value.hardware) std::cout << ' ' << unsigned(byte);
      std::cout << " software=";
      for (auto byte : value.software) std::cout << ' ' << unsigned(byte);
    } else if constexpr (std::is_same_v<V, EncoderAngles>) {
      std::cout << " encoder_calibrated=" << value.calibrated
                << " encoder_counts=" << value.raw_count << ',' << value.calibrated_count;
    }
  }, parameter);
}

const char* management_name(ManagementKind kind) {
  switch (kind) {
    case ManagementKind::discover: return "discover";
    case ManagementKind::zero: return "zero";
    case ManagementKind::set_id: return "set_id";
    case ManagementKind::reset_id: return "reset_id";
    case ManagementKind::write_failed: return "write_failed";
  }
  return "unknown";
}

void show(const Decoded& frame) {
  std::cout << "RX " << format_can(frame.raw.frame);
  if (const auto* telemetry = std::get_if<Telemetry>(&frame.feedback)) {
    const auto field = [](const char* name, const std::optional<Measurement>& value) {
      if (value) std::cout << ' ' << name << '=' << value->value;
    };
    std::cout << " type=" << unsigned(telemetry->type) << " fault=" << unsigned(telemetry->error);
    field("position_rad", telemetry->position_rad);
    field("velocity_rad_s", telemetry->velocity_rad_s);
    field("phase_current_a", telemetry->phase_current_a);
    field("motor_c", telemetry->motor_c);
    field("mos_c", telemetry->mos_c);
    if (telemetry->quantized && !telemetry->position_rad) {
      const auto& raw = *telemetry->quantized;
      std::cout << " quantized=" << raw[0] << ',' << raw[1] << ',' << raw[2];
    }
  } else if (const auto* query = std::get_if<QueryReply>(&frame.feedback)) {
    std::cout << " query=" << unsigned(query->code) << " fault=" << unsigned(query->error);
    show_value(query->value);
  } else if (const auto* status = std::get_if<ConfigStatus>(&frame.feedback)) {
    std::cout << " type=4 config=" << unsigned(status->code)
              << " status=" << (status->success ? "success" : "failure")
              << " fault=" << unsigned(status->error);
  } else if (const auto* echo = std::get_if<ConfigEcho>(&frame.feedback)) {
    std::cout << " config_echo=" << unsigned(echo->code);
    show_value(echo->value);
  } else if (const auto* group = std::get_if<KtReply>(&frame.feedback)) {
    std::cout << " Kt_header=" << unsigned(group->raw_header) << " Kt_group=";
    if (group->group) std::cout << unsigned(*group->group);
    else std::cout << "unknown";
    std::cout << " Kt_values=";
    for (auto value : group->values) std::cout << ' ' << value;
  } else if (const auto* brake = std::get_if<BrakeReply>(&frame.feedback)) {
    std::cout << (brake->error ? " type=6" : " brake_prefix=B2")
              << " brake=" << (brake->state == BrakeState::engaged ? "engaged" : "released");
    if (brake->error) std::cout << " fault=" << unsigned(*brake->error);
  } else if (const auto* management = std::get_if<ManagementReply>(&frame.feedback)) {
    std::cout << " management=" << management_name(management->kind)
              << " status=" << (management->success ? "success" : "failure");
    if (management->id) std::cout << " motor_id=" << *management->id;
  } else if (const auto* issue = std::get_if<CodecIssue>(&frame.feedback)) {
    std::cout << " undecoded";
    if (*issue == CodecIssue::ambiguous) std::cout << " reason=ambiguous";
  }
  std::cout << std::endl;
}

int monitor(const char* device, unsigned seconds) {
  SerialCanTransport bus(device);
  const Protocol codec;
  const auto deadline = Clock::now() + std::chrono::seconds(seconds);
  while (bus.connected() && Clock::now() < deadline) {
    if (const auto frame = bus.receive_for(100ms)) show(codec.decode(*frame));
  }
  return bus.connected() ? 0 : 1;
}

void send_command(SerialCanTransport& bus, const Encoded& command) {
  switch (bus.send_command(command)) {
    case SendStatus::accepted:
      print_tx(command);
      return;
    case SendStatus::disconnected:
      throw std::runtime_error("USB-CAN disconnected");
    case SendStatus::full:
      throw std::runtime_error("USB-CAN serial output buffer is full");
    case SendStatus::invalid_frame:
      throw std::runtime_error("USB-CAN requires a standard classic CAN data frame");
    case SendStatus::io_error:
      throw std::runtime_error("USB-CAN write or drain failed; transmission status is uncertain");
  }
}

struct RecentCommand {
  CanFrame frame;
  TimePoint expires;
};

void show_serial_frame(const Protocol& codec, const ReceivedFrame& sample,
                       std::deque<RecentCommand>& recent) {
  while (!recent.empty() && recent.front().expires < sample.stamp) recent.pop_front();
  const auto& rx = sample.frame;
  const auto matches = [&](const RecentCommand& tx) {
    return tx.frame.id == rx.id && tx.frame.length == rx.length &&
        std::equal(rx.data.begin(), rx.data.begin() + rx.length, tx.frame.data.begin());
  };
  if (std::any_of(recent.begin(), recent.end(), matches)) {
    std::cout << "RX " << format_can(rx) << " possible_tx_echo" << std::endl;
    return;
  }
  show(codec.decode(sample));
}

bool execute_line(const std::string& line, SerialCanTransport& bus,
                  std::deque<RecentCommand>& recent, TimePoint& next_send) {
  std::istringstream input(line);
  std::string verb;
  if (!(input >> verb)) return true;
  if (verb == "EXIT" || verb == "exit") return false;
  if (verb == "HELP" || verb == "help") {
    print_command_usage();
    std::cout.flush();
    return true;
  }
  std::vector<std::string> arguments;
  for (std::string word; input >> word;) arguments.push_back(word);
  std::vector<char*> args;
  for (auto& word : arguments) args.push_back(word.data());

  try {
    const auto command = parse_named_command(verb, static_cast<int>(args.size()), args.data());
    const auto now = Clock::now();
    if (now < next_send) {
      throw std::runtime_error("previous ENCOS command requires a 500 ms gap before the next send");
    }
    send_command(bus, command);
    const auto sent = Clock::now();
    next_send = sent + command.next_send_delay;
    while (!recent.empty() && recent.front().expires < now) recent.pop_front();
    recent.push_back({command.frame, sent + 500ms});
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
  }
  return true;
}

volatile std::sig_atomic_t stop_requested = 0;

void stop_console(int) { stop_requested = 1; }

int console(const char* device) {
  std::signal(SIGINT, stop_console);
  std::signal(SIGTERM, stop_console);
  auto bus = SerialCanTransport::for_commands(device);
  const Protocol codec;
  std::string pending;
  std::deque<RecentCommand> recent;
  auto next_send = TimePoint::min();
  std::cout << "Listening on " << device << " (HELP, EXIT)" << std::endl;

  while (!stop_requested && bus.connected()) {
    // Drain a batch of frames, then give terminal input a turn even under steady CAN traffic.
    unsigned count = 0;
    for (; count < 32; ++count) {
      const auto rx = bus.receive();
      if (!rx) break;
      show_serial_frame(codec, *rx, recent);
    }
    if (!bus.connected()) break;

    const auto serial_fd = bus.poll_descriptor();
    pollfd descriptors[]{{serial_fd == STDIN_FILENO ? -1 : STDIN_FILENO, POLLIN, 0},
                         {serial_fd, POLLIN, 0}};
    const int ready = ::poll(descriptors, 2, count == 32 ? 0 : 100);
    if (ready < 0) {
      if (errno == EINTR) continue;
      throw std::system_error(errno, std::generic_category(), "poll USB-CAN terminal");
    }
    if (!descriptors[0].revents || stop_requested) continue;
    std::array<char, 1024> bytes{};
    const auto size = ::read(STDIN_FILENO, bytes.data(), bytes.size());
    if (size < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      throw std::system_error(errno, std::generic_category(), "read terminal input");
    }
    if (size == 0) {
      execute_line(pending, bus, recent, next_send);
      return bus.connected() ? 0 : 1;
    }
    pending.append(bytes.data(), static_cast<std::size_t>(size));
    std::size_t end;
    while (bus.connected() && !stop_requested && (end = pending.find('\n')) != std::string::npos) {
      if (end > 4096) throw std::runtime_error("terminal command line exceeds 4096 bytes");
      if (!execute_line(pending.substr(0, end), bus, recent, next_send)) return 0;
      pending.erase(0, end + 1);
    }
    if (pending.size() > 4096) throw std::runtime_error("terminal command line exceeds 4096 bytes");
  }
  if (stop_requested) return 0;
  std::cerr << "USB-CAN disconnected\n";
  return 1;
}

void usage() {
  print_command_usage();
  std::cout << "\nUSB-CAN (classic CAN, read-only queries)\n"
            << "  encos_usb_can DEVICE monitor [SECONDS]\n"
            << "  encos_usb_can DEVICE discover --allow-query\n"
            << "  encos_usb_can DEVICE query ID CODE_OR_NAME [GROUP] --allow-query\n"
            << "  DISCOVER and QUERY are also accepted after DEVICE.\n"
            << "  Query transmission requires on-site authorization.\n";
}
}  // namespace

int main(int argc, char** argv) try {
  using namespace encos;
  const Protocol codec;
  if (argc == 1 || (argc == 2 && std::string_view(argv[1]) == "--help")) { usage(); return 0; }
  if (argc == 2) return console(argv[1]);
  const std::string_view action(argv[2]);
  if (action == "monitor" && argc <= 4) return monitor(argv[1], argc == 4 ? parse_unsigned(argv[3], 3600) : 5);
  if (std::string_view(argv[argc - 1]) != "--allow-query") {
    const auto command = parse_named_command(action, argc - 3, argv + 3);
    auto bus = SerialCanTransport::for_commands(argv[1]);
    send_command(bus, command);
    return 0;
  }
  auto request = codec.discover();
  if ((action == "query" || action == "QUERY") && (argc == 6 || argc == 7)) {
    request = codec.query(static_cast<std::uint16_t>(parse_unsigned(argv[3], 0x7fe)),
                          parse_query_code(argv[4]),
                          static_cast<std::uint8_t>(argc == 7 ? parse_unsigned(argv[5], 10) : 0));
  } else if ((action != "discover" && action != "DISCOVER") || argc != 4) {
    throw std::invalid_argument("invalid USB-CAN command");
  }
  SerialCanTransport bus(argv[1], true);
  TransactionManager transactions;
  std::cout << "QUERY " << format_can(request.frame) << '\n';
  const auto result = serial_query(bus, transactions, codec, request, 500ms, show);
  if (result.state != TransactionState::completed) {
    std::cerr << (result.state == TransactionState::timed_out ? "query timed out" : "query failed or rejected") << '\n';
    return 1;
  }
  if (const auto* id = std::get_if<ManagementReply>(&result.reply->feedback); id && id->id) {
    std::cout << "motor_id=" << *id->id << '\n';
  }
  if (result.uuid) {
    std::cout << "UUID=" << std::hex << std::uppercase << std::setfill('0');
    for (auto byte : *result.uuid) std::cout << std::setw(2) << unsigned(byte);
    std::cout << '\n';
  }
  return 0;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n';
  return 1;
}
