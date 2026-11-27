#include "cli_commands.hpp"

#include "encos_core/serial_can_transport.hpp"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace encos_tools {
namespace {
using namespace encos;

constexpr std::pair<std::string_view, QueryCode> query_names[] = {
    {"POSITION", QueryCode::position}, {"VELOCITY", QueryCode::velocity},
    {"PHASE_CURRENT", QueryCode::phase_current}, {"POWER", QueryCode::power},
    {"ACCELERATION", QueryCode::acceleration}, {"KT", QueryCode::kt},
    {"KP_RANGE", QueryCode::kp_range}, {"KD_RANGE", QueryCode::kd_range},
    {"POSITION_RANGE", QueryCode::position_range}, {"VELOCITY_RANGE", QueryCode::velocity_range},
    {"TORQUE_RANGE", QueryCode::torque_range}, {"CURRENT_RANGE", QueryCode::current_range},
    {"UUID", QueryCode::uuid}, {"VERSION", QueryCode::version}, {"TIMEOUT", QueryCode::timeout},
    {"CURRENT_PI", QueryCode::current_pi}, {"VELOCITY_PI", QueryCode::velocity_pi},
    {"POSITION_PD", QueryCode::position_pd}, {"KT_CALIBRATION", QueryCode::kt_calibration},
    {"KT_SEGMENT", QueryCode::kt_segment}, {"BRAKE", QueryCode::brake},
    {"ZERO_OFFSET", QueryCode::zero_offset}, {"ENCODER", QueryCode::encoder},
};

constexpr std::pair<std::string_view, ConfigCode> config_names[] = {
    {"ACCELERATION", ConfigCode::acceleration}, {"COMMUNICATION", ConfigCode::communication},
    {"KT", ConfigCode::kt}, {"KP_RANGE", ConfigCode::kp_range}, {"KD_RANGE", ConfigCode::kd_range},
    {"POSITION_RANGE", ConfigCode::position_range}, {"VELOCITY_RANGE", ConfigCode::velocity_range},
    {"TORQUE_RANGE", ConfigCode::torque_range}, {"CURRENT_RANGE", ConfigCode::current_range},
    {"TIMEOUT", ConfigCode::timeout}, {"CURRENT_PI", ConfigCode::current_pi},
    {"VELOCITY_PI", ConfigCode::velocity_pi}, {"POSITION_PD", ConfigCode::position_pd},
    {"KT_CALIBRATION", ConfigCode::kt_calibration}, {"KT_SEGMENT", ConfigCode::kt_segment},
    {"ZERO_OFFSET", ConfigCode::zero_offset},
};

constexpr std::pair<std::string_view, DriveMode> drive_modes[] = {
    {"CURRENT", DriveMode::current}, {"TORQUE", DriveMode::torque},
    {"DAMPING", DriveMode::damping}, {"DISSIPATIVE", DriveMode::dissipative},
    {"REGENERATIVE", DriveMode::regenerative},
};

template <typename Code, std::size_t N>
Code parse_code(std::string_view text, const std::pair<std::string_view, Code> (&names)[N]) {
  for (const auto& [name, code] : names) {
    if (text == name) return code;
  }
  return static_cast<Code>(parse_unsigned(text, 255));
}

template <typename Code, std::size_t N>
void print_codes(const std::pair<std::string_view, Code> (&names)[N]) {
  for (const auto& [name, code] : names) std::cout << ' ' << name << '=' << unsigned(code);
  std::cout << '\n';
}

void require_count(int count, int minimum, int maximum) {
  if (count < minimum || count > maximum) {
    throw std::invalid_argument("wrong number of command arguments; see --help");
  }
}

std::uint16_t motor_id(std::string_view text) {
  return static_cast<std::uint16_t>(parse_unsigned(text, management_id - 1));
}

double number(std::string_view text) {
  double value{};
  const auto end = text.data() + text.size();
  const auto parsed = std::from_chars(text.data(), end, value);
  if (parsed.ec != std::errc{} || parsed.ptr != end || !std::isfinite(value)) {
    throw std::invalid_argument("invalid finite number: " + std::string(text));
  }
  return value;
}

Ack ack(const char* text) { return static_cast<Ack>(parse_unsigned(text, 3)); }

BrakeState brake_state(std::string_view text) {
  if (text == "ENGAGE") return BrakeState::engaged;
  if (text == "RELEASE") return BrakeState::released;
  throw std::invalid_argument("brake state must be ENGAGE or RELEASE");
}

ConfigValue config_value(ConfigCode code, int count, char** args) {
  const auto* spec = parameter(code);
  if (!spec) throw std::invalid_argument("reserved configuration code");
  switch (spec->payload) {
    case Payload::scalar16:
      require_count(count, 1, 1);
      return Scalar{number(args[0]) * (code == ConfigCode::zero_offset ? wire::degrees_to_rad : 1)};
    case Payload::enumeration:
      require_count(count, 1, 1);
      return Enumeration{static_cast<std::uint8_t>(parse_unsigned(args[0], 255))};
    case Payload::pair16:
      require_count(count, 2, 2);
      return Pair{number(args[0]), number(args[1])};
    case Payload::segment:
      require_count(count, 5, 5);
      return KtWrite{static_cast<std::uint8_t>(parse_unsigned(args[0], 10)),
                     {number(args[1]), number(args[2]), number(args[3]), number(args[4])}};
    default:
      throw std::invalid_argument("unsupported configuration payload");
  }
}

Encoded encode_mixed(const Protocol& codec, char** args) {
  ProtocolProfile profile;
  profile.mixed = MixedRanges{
      {number(args[6]), number(args[7])}, {number(args[8]), number(args[9])},
      {number(args[10]), number(args[11])}, {number(args[12]), number(args[13])},
      {number(args[14]), number(args[15])},
  };
  return codec.control(motor_id(args[0]), Mixed{number(args[1]), number(args[2]), number(args[3]),
                                               number(args[4]), number(args[5])}, profile);
}

void print_reply(const ReplyKey& key) {
  switch (key.kind) {
    case ReplyKind::none: std::cout << "EXPECTED_REPLY none"; break;
    case ReplyKind::telemetry: std::cout << "EXPECTED_REPLY telemetry type=" << unsigned(key.code); break;
    case ReplyKind::query: std::cout << "EXPECTED_REPLY query code=" << unsigned(key.code); break;
    case ReplyKind::config: std::cout << "EXPECTED_REPLY config code=" << unsigned(key.code); break;
    case ReplyKind::kt_group: std::cout << "EXPECTED_REPLY Kt group=" << unsigned(key.code); break;
    case ReplyKind::brake:
      std::cout << (key.code == 6 ? "EXPECTED_REPLY brake type=6" : "EXPECTED_REPLY brake prefix=B2");
      break;
    case ReplyKind::discover: std::cout << "EXPECTED_REPLY management discover"; break;
    case ReplyKind::zero: std::cout << "EXPECTED_REPLY management zero"; break;
    case ReplyKind::set_id: std::cout << "EXPECTED_REPLY management set_id"; break;
    case ReplyKind::reset_id: std::cout << "EXPECTED_REPLY management reset_id"; break;
  }
  std::cout << '\n';
}
}  // namespace

Encoded parse_named_command(std::string_view verb, int count, char** args) {
  const Protocol codec;
  constexpr auto offline = WritePermission::offline;
  if (verb == "discover") verb = "DISCOVER";
  if (verb == "query") verb = "QUERY";
  if (verb == "DISCOVER" || verb == "RESET_ID") {
    require_count(count, 0, 0);
    return verb == "DISCOVER" ? codec.discover() : codec.reset_id(offline);
  }
  if (verb == "QUERY") {
    require_count(count, 2, 3);
    const auto group = count == 3 ? parse_unsigned(args[2], 10) : 0;
    return codec.query(motor_id(args[0]), parse_query_code(args[1]), static_cast<std::uint8_t>(group));
  }
  if (verb == "CONFIG") {
    require_count(count, 3, 7);
    const auto code = parse_code(args[1], config_names);
    return codec.configure(motor_id(args[0]), code, config_value(code, count - 2, args + 2), offline);
  }
  if (verb == "ZERO") {
    require_count(count, 1, 2);
    std::optional<double> offset;
    if (count == 2) offset = number(args[1]) * wire::degrees_to_rad;
    return codec.zero(motor_id(args[0]), offset, offline);
  }
  if (verb == "SET_ID") {
    require_count(count, 2, 2);
    return codec.set_id(motor_id(args[0]), motor_id(args[1]), offline);
  }
  if (verb == "ENABLE") {
    require_count(count, 1, 1);
    throw std::invalid_argument("ENABLE unavailable: manual does not define an independent CAN enable frame");
  }
  if (verb == "POS") {
    require_count(count, 2, 5);
    const auto* speed = count >= 3 ? args[2] : "20";
    const auto* current = count >= 4 ? args[3] : "1";
    const auto* reply = count == 5 ? args[4] : "2";
    const auto id = motor_id(args[0]);
    return codec.control(id, Position{number(args[1]) * wire::degrees_to_rad,
        number(speed) * wire::rpm_to_rad_s, number(current), ack(reply)});
  }
  if (verb == "VEL") {
    require_count(count, 3, 4);
    return codec.control(motor_id(args[0]), Velocity{number(args[1]) * wire::rpm_to_rad_s,
        number(args[2]), count == 4 ? ack(args[3]) : Ack::type1});
  }
  if (verb == "MIXED") {
    require_count(count, 16, 16);
    return encode_mixed(codec, args);
  }
  if (verb == "BRAKE" || verb == "ELECTROMAGNETIC_BRAKE") {
    require_count(count, 2, 2);
    const auto state = brake_state(args[1]);
    const auto id = motor_id(args[0]);
    if (verb == "BRAKE") return codec.brake(id, state, offline);
    const double value = state == BrakeState::released ? 1 : 0;
    return codec.control(id, Drive{DriveMode::brake, value, Ack::type1});
  }
  for (const auto& [name, mode] : drive_modes) {
    if (verb == name) {
      const int required = mode == DriveMode::damping ? 1 : 2;
      require_count(count, required, required + 1);
      const auto value = required == 1 ? 0.0 : number(args[1]);
      return codec.control(motor_id(args[0]), Drive{mode, value,
          count > required ? ack(args[required]) : Ack::type1});
    }
  }
  throw std::invalid_argument("unknown ENCOS command: " + std::string(verb));
}

unsigned parse_unsigned(std::string_view text, unsigned maximum) {
  unsigned value{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value > maximum) {
    throw std::invalid_argument("invalid decimal argument: " + std::string(text));
  }
  return value;
}

encos::QueryCode parse_query_code(std::string_view text) {
  return parse_code(text, query_names);
}

std::string format_can(const encos::CanFrame& frame) {
  std::ostringstream stream;
  stream << std::hex << std::uppercase << std::setfill('0') << std::setw(3) << frame.id << '#';
  for (std::size_t i = 0; i < frame.length && i < frame.data.size(); ++i) {
    stream << std::setw(2) << static_cast<unsigned>(frame.data[i]);
  }
  return stream.str();
}

void print_tx(const encos::Encoded& command) {
  const auto& frame = command.frame;
  std::cout << "TX CAN " << format_can(frame) << " DLC=" << unsigned(frame.length) << '\n';
  std::ostringstream stream;
  stream << "SERIAL" << std::hex << std::uppercase << std::setfill('0');
  for (auto byte : encos::encode_serial_can(frame)) stream << ' ' << std::setw(2) << unsigned(byte);
  std::cout << stream.str() << '\n';
  print_reply(command.reply);
  std::cout.flush();
}

void print_command_usage() {
  std::cout << R"(
USB-CAN terminal
  encos_usb_can DEVICE
  encos_usb_can DEVICE COMMAND [ARGS...]

DEVICE alone keeps the serial port open for RX and accepts commands on stdin.
Type HELP for commands or EXIT to close the port. Startup only listens.
DEVICE COMMAND writes one standard classic CAN frame and exits.
Commands
  DISCOVER
  QUERY ID CODE_OR_NAME [GROUP]
  POS ID DEG [RPM_LIMIT [AMP_LIMIT [ACK]]]
  VEL ID RPM AMP_LIMIT [ACK]
  CURRENT ID AMPS [ACK]
  TORQUE ID NM [ACK]
  DAMPING ID [ACK]
  DISSIPATIVE ID AMP_LIMIT [ACK]
  REGENERATIVE ID AMP_LIMIT [ACK]
  BRAKE|ELECTROMAGNETIC_BRAKE ID ENGAGE|RELEASE
  MIXED ID KP KD RAD RAD_S NM KP_MIN KP_MAX KD_MIN KD_MAX
        RAD_MIN RAD_MAX RAD_S_MIN RAD_S_MAX NM_MIN NM_MAX
  ZERO ID [OFFSET_DEG]
  SET_ID OLD_ID NEW_ID
  RESET_ID
  CONFIG ID CODE_OR_NAME VALUE...

Decimal motor IDs are 1 through 2046.
POS defaults are 20 rpm, 1 A, ACK 2. Other optional ACKs default to 1.
ACK 0 requests no reply, 1-3 request the corresponding telemetry type.
QUERY KT_SEGMENT takes a group from 1 through 10. Other queries take no group.
CONFIG takes one scalar or raw mode, two values for a range or gain pair,
or GROUP KT1 KT2 KT3 KT4 for KT_SEGMENT. POSITION_RANGE is in rad,
VELOCITY_RANGE in rad/s, ZERO_OFFSET in degrees and TIMEOUT in ms.
ACCELERATION accepts 0-20 rad/s^2 and requests ACK=1 per section 9.2.1.
ENABLE has no confirmed CAN frame.

Query codes
)";
  print_codes(query_names);
  std::cout << "Configuration codes\n";
  print_codes(config_names);
}

}  // namespace encos_tools
