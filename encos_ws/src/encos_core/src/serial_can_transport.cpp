#include "encos_core/serial_can_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <numeric>
#include <poll.h>
#include <sys/ioctl.h>
#include <system_error>
#include <thread>
#include <unistd.h>

namespace encos {
namespace {
bool supported(const CanFrame& frame) {
  return matches_wire_format(frame, WireFormat::classic) && frame.id <= management_id &&
      frame.length <= 8 && !frame.extended && !frame.rtr && !frame.error && !frame.esi;
}

bool same_wire_data(const CanFrame& left, const CanFrame& right) {
  return left.id == right.id && left.length == right.length &&
      std::equal(left.data.begin(), left.data.begin() + left.length, right.data.begin());
}

bool is_query(const CanFrame& frame) {
  if (frame.id == management_id) return same_wire_data(frame, Protocol{}.discover().frame);
  if (!valid_motor_id(frame.id) || frame.length != 2) return false;
  const auto code = static_cast<QueryCode>(frame.data[1]);
  if (!parameter(code)) return false;
  if (code != QueryCode::kt_segment) return frame.data[0] == 0xe1;
  return frame.data[0] >= 0xe1 && frame.data[0] <= 0xea;
}

template <class Iterator>
std::uint8_t checksum(Iterator first, Iterator last) {
  return static_cast<std::uint8_t>(std::accumulate(first, last, 0u));
}

[[noreturn]] void serial_error(const char* action) {
  throw std::system_error(errno, std::generic_category(), action);
}
}  // namespace

std::vector<std::uint8_t> encode_serial_can(const CanFrame& frame) {
  if (!supported(frame)) throw std::invalid_argument("USB-CAN framing requires a classic standard data frame");
  std::vector<std::uint8_t> packet{0xaa, static_cast<std::uint8_t>(frame.id >> 8),
      static_cast<std::uint8_t>(frame.id), static_cast<std::uint8_t>(frame.length + 5)};
  packet.insert(packet.end(), frame.data.begin(), frame.data.begin() + frame.length);
  packet.push_back(checksum(packet.begin(), packet.end()));
  return packet;
}

SerialCanTransport::SerialCanTransport(const std::string& device, bool allow_queries)
    : SerialCanTransport(device, allow_queries ? Access::queries : Access::monitor) {}

SerialCanTransport SerialCanTransport::for_commands(const std::string& device) {
  return SerialCanTransport(device, Access::commands);
}

SerialCanTransport::SerialCanTransport(const std::string& device, Access access)
    : fd_(::open(device.c_str(), (access == Access::monitor ? O_RDONLY : O_RDWR) |
                                O_NOCTTY | O_NONBLOCK | O_CLOEXEC)),
      access_(access) {
  if (fd_ < 0) serial_error("open USB-CAN serial device");
  bool exclusive = false;
  bool saved = false;
  try {
    if (::ioctl(fd_, TIOCEXCL) < 0) serial_error("reserve USB-CAN serial device");
    exclusive = true;
    if (::tcgetattr(fd_, &original_) < 0) serial_error("read serial attributes");
    saved = true;
    auto settings = original_;
    ::cfmakeraw(&settings);
    settings.c_iflag = 0;
    settings.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_cc[VMIN] = settings.c_cc[VTIME] = 0;
    if (::cfsetspeed(&settings, B115200) < 0 || ::tcsetattr(fd_, TCSANOW, &settings) < 0) {
      serial_error("set serial link to 115200 8N1");
    }
    input_.reserve(269);
  } catch (...) {
    if (saved) ::tcsetattr(fd_, TCSANOW, &original_);
    if (exclusive) ::ioctl(fd_, TIOCNXCL);
    ::close(fd_);
    throw;
  }
}

SerialCanTransport::~SerialCanTransport() {
  ::tcsetattr(fd_, TCSANOW, &original_);
  ::ioctl(fd_, TIOCNXCL);
  ::close(fd_);
}

SendStatus SerialCanTransport::send(const CanFrame& frame) {
  if (access_ != Access::queries || !supported(frame) || !is_query(frame)) return SendStatus::invalid_frame;
  return write_frame(frame);
}

SendStatus SerialCanTransport::send_command(const Encoded& command) {
  using namespace std::chrono_literals;
  if (access_ != Access::commands || !supported(command.frame)) return SendStatus::invalid_frame;
  const auto status = write_frame(command.frame);
  if (status != SendStatus::accepted) return status;

  // Drain the serial output before reporting success. A one-shot caller may close immediately.
  const auto deadline = Clock::now() + 100ms;
  do {
    int queued = 0;
    if (::ioctl(fd_, TIOCOUTQ, &queued) < 0) break;
    if (queued == 0) {
      // Allow time for the adapter to clock out up to 13 UART bytes at 115200 8N1.
      std::this_thread::sleep_for(2ms);
      return SendStatus::accepted;
    }
    std::this_thread::sleep_for(1ms);
  } while (Clock::now() < deadline);
  connected_ = false;
  return SendStatus::io_error;
}

SendStatus SerialCanTransport::write_frame(const CanFrame& frame) {
  if (!connected_) return SendStatus::disconnected;
  const auto packet = encode_serial_can(frame);
  const auto count = ::write(fd_, packet.data(), packet.size());
  if (count == static_cast<ssize_t>(packet.size())) return SendStatus::accepted;
  if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return SendStatus::full;
  // A short write can leave half a packet in the adapter. Stop using the session.
  connected_ = false;
  return SendStatus::io_error;
}

bool SerialCanTransport::discard_input() {
  input_.clear();
  return connected_ && ::tcflush(fd_, TCIFLUSH) == 0;
}

std::optional<ReceivedFrame> SerialCanTransport::extract() {
  while (!input_.empty()) {
    input_.erase(input_.begin(), std::find(input_.begin(), input_.end(), std::uint8_t{0xaa}));
    if (input_.size() < 4) return {};
    const auto length = input_[3];
    const auto id = static_cast<unsigned>((input_[1] << 8) | input_[2]);
    if (length >= 5 && length <= 13 && id <= management_id) {
      if (input_.size() < length) return {};
      const auto crc = input_.begin() + length - 1;
      if (checksum(input_.begin(), crc) == *crc) {
        ReceivedFrame decoded;
        decoded.frame.id = id;
        decoded.frame.length = static_cast<std::uint8_t>(length - 5);
        std::copy(input_.begin() + 4, crc, decoded.frame.data.begin());
        decoded.stamp = last_read_;
        decoded.userspace_stamp = last_read_;
        input_.erase(input_.begin(), crc + 1);
        return decoded;
      }
    }
    input_.erase(input_.begin());
  }
  return {};
}

std::optional<ReceivedFrame> SerialCanTransport::receive() {
  return receive_for(Clock::duration::zero());
}

std::optional<ReceivedFrame> SerialCanTransport::receive_for(Clock::duration wait) {
  const auto deadline = Clock::now() + wait;
  while (connected_) {
    if (auto frame = extract()) return frame;
    const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now()).count();
    pollfd watch{fd_, POLLIN, 0};
    const int ready = ::poll(&watch, 1, static_cast<int>(std::clamp<std::int64_t>(remaining, 0, 1000)));
    if (ready == 0) return {};
    if (ready < 0 && errno == EINTR) {
      if (Clock::now() >= deadline) return {};
      continue;
    }
    if (ready < 0 || (watch.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      connected_ = false;
      return {};
    }
    std::array<std::uint8_t, 256> bytes{};
    const auto count = ::read(fd_, bytes.data(), bytes.size());
    if (count <= 0) {
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) connected_ = false;
      return {};
    }
    last_read_ = Clock::now();
    input_.insert(input_.end(), bytes.begin(), bytes.begin() + count);
    if (last_read_ >= deadline) return extract();
  }
  return {};
}

TransactionEvent serial_query(SerialCanTransport& bus, TransactionManager& transactions,
                              const Protocol& codec, const Encoded& request, Clock::duration timeout,
                              const std::function<void(const Decoded&)>& observed) {
  using S = TransactionState;
  const auto start = Clock::now();
  const auto deadline = start + timeout;
  if (transactions.begin(request, start, deadline) != Begin::accepted) return {S::cancelled, {}, {}};
  if (!bus.discard_input() || bus.send(request.frame) != SendStatus::accepted) return transactions.cancel();
  while (Clock::now() < deadline && bus.connected()) {
    auto reply = bus.receive_for(deadline - Clock::now());
    if (!reply) continue;
    // The serial packet contains no direction flag. Suppress an exact request echo before matching.
    if (same_wire_data(reply->frame, request.frame)) continue;
    auto decoded = codec.decode(*reply);
    decoded.raw.origin = Origin::remote;
    const auto event = transactions.receive(decoded, Clock::now());
    decoded.raw.origin = Origin::unknown;
    if (observed) observed(decoded);
    if (!transactions.pending()) return event;
  }
  if (!bus.connected()) return transactions.cancel();
  return transactions.poll(Clock::now());
}
}  // namespace encos
