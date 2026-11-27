#include "check.hpp"
#include "encos_core/serial_can_transport.hpp"

#include <fcntl.h>
#include <future>
#include <poll.h>
#include <system_error>
#include <unistd.h>

namespace {
using namespace test;

struct PseudoTerminal {
  int master{::posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC)};
  std::string device;

  PseudoTerminal() {
    CHECK(master >= 0 && ::grantpt(master) == 0 && ::unlockpt(master) == 0);
    const char* name = ::ptsname(master);
    CHECK(name != nullptr);
    device = name;
  }

  ~PseudoTerminal() { if (master >= 0) ::close(master); }

  void send(const std::vector<std::uint8_t>& bytes) const {
    CHECK(::write(master, bytes.data(), bytes.size()) == ssize_t(bytes.size()));
  }

  std::vector<std::uint8_t> packet(std::size_t length) const {
    std::vector<std::uint8_t> result(length);
    for (std::size_t offset = 0; offset < length;) {
      pollfd wait{master, POLLIN, 0};
      CHECK(::poll(&wait, 1, 1000) == 1);
      auto read = ::read(master, result.data() + offset, length - offset);
      CHECK(read > 0);
      offset += static_cast<std::size_t>(read);
    }
    return result;
  }

  void expect_no_write() const {
    pollfd wait{master, POLLIN, 0};
    CHECK(::poll(&wait, 1, 10) == 0);
  }
};
}  // namespace

int main() {
  using namespace test;
  const Protocol codec;
  const std::vector<std::uint8_t> discover{0xaa, 0x07, 0xff, 0x09, 0xff, 0xff, 0x00, 0x82, 0x39};
  const std::vector<std::uint8_t> feedback{0xaa, 0x00, 0x01, 0x0d, 0, 0, 0, 0, 0, 0, 0, 0, 0xb8};
  CHECK(encode_serial_can(codec.discover().frame) == discover);
  PseudoTerminal pty;
  int side = ::open(pty.device.c_str(), O_RDONLY | O_NOCTTY | O_CLOEXEC);
  CHECK(side >= 0);
  termios before{}, after{};
  CHECK(::tcgetattr(side, &before) == 0);
  {
    SerialCanTransport bus(pty.device);
    CHECK(bus.send(codec.discover().frame) == SendStatus::invalid_frame);
    CHECK(bus.send_command(codec.discover()) == SendStatus::invalid_frame);
    pty.expect_no_write();
    pty.send({0xff, 0x20, 0xaa, 0, 1, 1, 0xaa, 0, 1, 7, 0xe1, 1, 0});
    pty.send(std::vector<std::uint8_t>(feedback.begin(), feedback.begin() + 4));
    CHECK(!bus.receive_for(10ms));
    pty.send(std::vector<std::uint8_t>(feedback.begin() + 4, feedback.end()));
    pty.send(feedback);
    for (int count = 0; count < 2; ++count) {
      auto raw = bus.receive_for(50ms);
      CHECK(raw && raw->frame.id == 1 && raw->frame.length == 8);
      CHECK(raw->origin == Origin::unknown && raw->userspace_stamp == raw->stamp && !raw->kernel_realtime);
      CHECK(std::holds_alternative<CodecIssue>(codec.decode(*raw).feedback));
    }
  }
  CHECK(::tcgetattr(side, &after) == 0);
  CHECK(before.c_iflag == after.c_iflag && before.c_lflag == after.c_lflag &&
        ::cfgetispeed(&before) == ::cfgetispeed(&after));
  {
    auto terminal = SerialCanTransport::for_commands(pty.device);
    CHECK(terminal.send(codec.discover().frame) == SendStatus::invalid_frame);
    CHECK(terminal.send_command(Protocol(WireFormat::fd_brs).discover()) == SendStatus::invalid_frame);
    pty.expect_no_write();
    CHECK(terminal.send_command(codec.discover()) == SendStatus::accepted);
    CHECK(pty.packet(discover.size()) == discover);
    pty.expect_no_write();
  }
  ::close(side);

  SerialCanTransport bus(pty.device, true);
  CHECK(bus.send_command(codec.control(1, Position{})) == SendStatus::invalid_frame);
  CHECK(bus.send(codec.control(1, Position{}).frame) == SendStatus::invalid_frame);
  CHECK(bus.send(Protocol(WireFormat::fd_brs).discover().frame) == SendStatus::invalid_frame);
  CHECK(bus.send(codec.zero(1, {}, offline).frame) == SendStatus::invalid_frame);
  pty.expect_no_write();
  TransactionManager transactions;
  auto peer = std::async(std::launch::async, [&] {
    CHECK(pty.packet(discover.size()) == discover);
    pty.send(discover);  // Serial transmit echo must not complete the request.
    pty.send({0xaa, 0x07, 0xff, 0x0a, 0xff, 0xff, 0x01, 0x00, 0x01, 0xba});
  });
  const auto found = serial_query(bus, transactions, codec, codec.discover(), 200ms);
  peer.get();
  CHECK(found.state == TransactionState::completed);
  CHECK(std::get<ManagementReply>(found.reply->feedback).id == 1);
  CHECK(serial_query(bus, transactions, codec, codec.discover(), 50ms).state == TransactionState::cancelled);
  pty.expect_no_write();
  auto measured = std::async(std::launch::async, [&] {
    CHECK(pty.packet(7) == (std::vector<std::uint8_t>{0xaa, 0, 1, 7, 0xe1, 1, 0x94}));
    pty.send({0xaa, 0, 1, 0x0b, 0xa0, 1, 0x42, 0xb4, 0, 0, 0x4d});
  });
  const auto query = serial_query(bus, transactions, codec, codec.query(1, QueryCode::position), 200ms);
  measured.get();
  CHECK(query.state == TransactionState::completed);
  near(std::get<Scalar>(std::get<QueryReply>(query.reply->feedback).value).value, 90 * deg);
  const auto timeout = serial_query(bus, transactions, codec, codec.query(1, QueryCode::velocity), 20ms);
  CHECK(timeout.state == TransactionState::timed_out);
  CHECK(pty.packet(7) == (std::vector<std::uint8_t>{0xaa, 0, 1, 7, 0xe1, 2, 0x95}));
  ::close(pty.master);
  pty.master = -1;
  CHECK(!bus.receive_for(50ms) && !bus.connected());
}
