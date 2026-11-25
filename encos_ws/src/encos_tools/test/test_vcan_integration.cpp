#include "test_support.hpp"

#include "encos_core/safety_manager.hpp"

#include <cstdlib>
#include <future>
#include <iostream>
#include <linux/can.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/socket.h>
#include <system_error>
#include <thread>
#include <unistd.h>

using namespace p1_test;

namespace {

class Peer {
 public:
  Peer(const std::string& interface, ModelOptions options = {}, FaultOptions faults = {})
      : bus_({interface, options.format, {}, true}), motor_(options), queue_(faults),
        worker_([this] { run(); }) {}
  ~Peer() {
    stopping_.store(true);
    bus_.interrupt();
    worker_.join();
  }
  bool failed() const { return failed_.load(); }
 private:
  void run() noexcept {
    try {
      while (!stopping_.load()) {
        const auto now = Clock::now();
        motor_.advance(now);
        while (const auto reply = queue_.take(now)) {
          if (bus_.send(*reply) != SendStatus::accepted) {
            failed_.store(true);
            return;
          }
        }
        const auto wake = std::min(now + 10ms, queue_.next().value_or(now + 10ms));
        const auto raw = bus_.receive_for(std::max(Clock::duration::zero(), wake - Clock::now()));
        if (raw.status == ReceiveStatus::disconnected || raw.status == ReceiveStatus::io_error) {
          failed_.store(true);
          return;
        }
        if (raw.sample) {
          const auto arrived = Clock::now();
          queue_.enqueue(motor_.handle(*raw.sample, arrived), arrived);
        }
      }
    } catch (...) {
      failed_.store(true);
    }
  }
  SocketCanTransport bus_;
  VirtualMotor motor_;
  FaultQueue queue_;
  std::atomic<bool> stopping_{}, failed_{};
  std::thread worker_;
};

struct Rig {
  Rig(const std::string& name, WireFormat format, FaultOptions faults = {}, ModelOptions model = {})
      : peer(name, ModelOptions{model.id, format, model.error, model.motor_c, model.reject_writes}, faults),
        bus({name, format, {}, true}), codec(format) {}
  Exchange run(const Encoded& packet, S expected = S::completed, Clock::duration duration = 80ms) {
    auto result = exchange(bus, tx, codec, packet, profile(), duration, ignore_sample);
    if (result.admitted != Begin::accepted || result.event.state != expected || peer.failed()) {
      throw std::runtime_error(hex(packet.frame) + " state=" + std::to_string(static_cast<int>(result.event.state)) +
                               " begin=" + std::to_string(static_cast<int>(result.admitted)));
    }
    return result;
  }
  void verify(const Encoded& packet, std::uint32_t motor, const std::vector<std::uint8_t>& expected) {
    const auto result = run(packet);
    CHECK(result.event.reply);
    const auto& raw = result.event.reply->raw;
    CHECK(raw.origin == Origin::remote && raw.kernel_realtime && raw.stamp > result.tx_before);
    CHECK(raw.frame.brs == packet.frame.brs && raw.frame.kind == packet.frame.kind);
    bytes(raw.frame, motor, expected);
  }

  Peer peer;
  SocketCanTransport bus;
  Protocol codec;
  TransactionManager tx{240ms, ReplyReuse::single_reply_simulation};
};

void socket_options(const std::string& name, WireFormat format) {
  SocketCanTransport a({name, format, {1}, true});
  SocketCanTransport b({name, format, {1}, true});
  auto raw = frame(1, {0xe1, 1}, format);
  CHECK(a.connected() && b.connected());
  CHECK(a.send(frame(3, {0xe1, 1}, format)) == SendStatus::accepted);
  CHECK(a.send(raw) == SendStatus::accepted);
  const auto read = b.receive_for(80ms);
  CHECK(read.status == ReceiveStatus::frame);
  CHECK(read.sample && read.sample->kernel_realtime && read.sample->origin == Origin::remote);
  CHECK(matches_wire_format(read.sample->frame, format));
  bytes(read.sample->frame, 1, {0xe1, 1});
  CHECK(b.receive_for(2ms).status == ReceiveStatus::timeout);
  CHECK(a.receive_for(2ms).status == ReceiveStatus::timeout);
  raw.rtr = true;
  CHECK(a.send(raw) == SendStatus::invalid_frame);
  raw.rtr = false;
  raw.extended = true;
  CHECK(a.send(raw) == SendStatus::invalid_frame);
  raw.extended = false;
  raw.length = 9;
  CHECK(a.send(raw) == SendStatus::invalid_frame);

  auto pending = std::async(std::launch::async, [&] {
    for (;;) {
      auto read = a.receive_for(3s);
      if (read.status != ReceiveStatus::busy) {
        return read;
      }
      std::this_thread::yield();
    }
  });
  const auto deadline = Clock::now() + 500ms;
  bool busy = false;
  while (Clock::now() < deadline && !busy) {
    busy = a.receive_for(0ms).status == ReceiveStatus::busy;
    if (!busy) std::this_thread::sleep_for(1ms);
  }
  a.interrupt();
  CHECK(pending.get().status == ReceiveStatus::interrupted && busy);
  CHECK(a.receive_for(1ms).status == ReceiveStatus::timeout);

  const auto send_range = [&](unsigned first) {
    for (unsigned seq = first; seq < first + 16; ++seq) {
      CHECK(a.send(frame(1, {0xe1, static_cast<std::uint8_t>(seq)}, format)) == SendStatus::accepted);
    }
  };
  auto first_sender = std::async(std::launch::async, send_range, 0);
  auto second_sender = std::async(std::launch::async, send_range, 16);
  std::array<bool, 32> seen{};
  for (std::size_t i = 0; i < seen.size(); ++i) {
    const auto item = b.receive_for(100ms);
    CHECK(item.sample && item.sample->frame.length == 2 && item.sample->frame.data[0] == 0xe1);
    const auto seq = item.sample->frame.data[1];
    CHECK(seq < seen.size() && !seen[seq]);
    seen[seq] = true;
  }
  first_sender.get();
  second_sender.get();

  VirtualMotor delayed({1, format});
  CHECK(a.send(frame(1, {0x40, 0x42, 0x48, 0, 0, 0, 0x64}, format)) == SendStatus::accepted);
  std::this_thread::sleep_for(55ms);
  const auto old_command = b.receive_for(100ms);
  CHECK(old_command.sample && delayed.handle(*old_command.sample, Clock::now()).empty());
  CHECK(!delayed.state().active);
}

void management(const std::string& name, WireFormat format) {
  Rig rig(name, format);
  const auto pause = [] { std::this_thread::sleep_for(505ms); };
  auto& p = rig.codec;
  const auto discover = p.discover();
  bytes(discover.frame, 0x7ff, {0xff, 0xff, 0, 0x82});
  rig.verify(discover, 0x7ff, {0xff, 0xff, 1, 0, 1});
  const auto set_id = p.set_id(1, 0x123, offline);
  bytes(set_id.frame, 0x7ff, {0, 1, 0, 4, 1, 0x23});
  rig.verify(set_id, 0x7ff, {1, 0x23, 1, 4});
  CHECK(rig.tx.begin(discover, Clock::now(), Clock::now() + 40ms) == Begin::wait_after_write);
  pause();
  rig.verify(discover, 0x7ff, {0xff, 0xff, 1, 1, 0x23});
  rig.verify(p.zero(0x123, {}, offline), 0x7ff, {1, 0x23, 1, 3});
  pause();
  const auto z = p.zero(0x123, -10 * wire::degrees_to_rad, offline);
  bytes(z.frame, 0x7ff, {1, 0x23, 0, 3, 0xfc, 0x18});
  rig.verify(z, 0x7ff, {1, 0x23, 1, 3});
  pause();
  rig.verify(p.query(0x123, Q::zero_offset), 0x123, {0xa0, 38, 0xfc, 0x18});
  rig.verify(p.query(0x123, Q::position), 0x123, {0xa0, 1, 0xc1, 0x20, 0, 0});
  rig.verify(p.query(0x123, Q::encoder), 0x123, {0xa0, 39, 1, 0xfc, 0x18, 0xfc, 0x18});
  rig.verify(p.reset_id(offline), 0x7ff, {0x7f, 0x7f, 1, 5, 0x7f, 0x7f});
  pause();
  rig.verify(discover, 0x7ff, {0xff, 0xff, 1, 0, 1});
}

void independent_replies(const std::string& name, WireFormat format) {
  SocketCanTransport bus({name, format, {}, true});
  SocketCanTransport source({name, format, {}, true});
  const Protocol codec(format);
  TransactionManager tx;
  const auto cfg = codec.configure(1, C::kt, Scalar{2.5}, offline);
  const auto verify = [&](const Encoded& request, const CanFrame& response, S expected) {
    const auto sent = Clock::now();
    CHECK(tx.begin(request, sent, sent + 80ms) == Begin::accepted);
    CHECK(source.send(response) == SendStatus::accepted);
    const auto read = bus.receive_for(80ms);
    CHECK(read.sample && read.sample->kernel_realtime);
    CHECK(tx.receive(codec.decode(*read.sample), Clock::now()).state == expected);
  };
  verify(cfg, frame(1, {0x80, 4, 1}, format), S::completed);
  verify(codec.discover(), frame(0x7ff, {0x80, 0x80, 1, 0x80}, format), S::rejected);

  // Kernel test fixture restricted by the already verified vcan transport.
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(::if_nametoindex(name.c_str()));
  CHECK(address.can_ifindex != 0);
  const int raw = ::socket(AF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  CHECK(raw >= 0);
  const int bound = ::bind(raw, reinterpret_cast<sockaddr*>(&address), sizeof(address));
  can_frame error{};
  error.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF;
  error.len = CAN_ERR_DLC;
  const auto count = bound == 0 ? ::send(raw, &error, sizeof(error), MSG_DONTWAIT) : -1;
  ::close(raw);
  CHECK(count == CAN_MTU);
  const auto received = bus.receive_for(80ms);
  CHECK(received.sample && received.sample->frame.error && received.sample->frame.id == CAN_ERR_BUSOFF);
  CHECK(std::get<CodecIssue>(codec.decode(*received.sample).feedback) == CodecIssue::invalid_frame);
}

void configurations(const std::string& name, WireFormat format) {
  Rig rig(name, format);
  auto& p = rig.codec;
  struct WriteRead {
    C code;
    Q query;
    ConfigValue value;
    std::vector<std::uint8_t> raw;
  };
  const std::vector<WriteRead> options{
    {C::kt, Q::kt, Scalar{2.6}, {1, 4}},
    {C::kp_range, Q::kp_range, Pair{0, 600}, {0, 0, 2, 0x58}},
    {C::kd_range, Q::kd_range, Pair{1, 50}, {0, 1, 0, 50}},
    {C::position_range, Q::position_range, Pair{-10, 12.5}, {0xfc, 0x18, 4, 0xe2}},
    {C::velocity_range, Q::velocity_range, Pair{-20, 21}, {0xf8, 0x30, 8, 0x34}},
    {C::torque_range, Q::torque_range, Pair{-90, 90}, {0xfc, 0x7c, 3, 0x84}},
    {C::current_range, Q::current_range, Pair{-60, 60}, {0xfd, 0xa8, 2, 0x58}},
    {C::timeout, Q::timeout, Scalar{625}, {2, 0x71}},
    {C::current_pi, Q::current_pi, Pair{0.02, 60}, {0, 0xc8, 2, 0x58}},
    {C::velocity_pi, Q::velocity_pi, Pair{0.007, 0.08}, {2, 0xbc, 0x1f, 0x40}},
    {C::position_pd, Q::position_pd, Pair{0.008, 0.00005}, {3, 0x20, 0, 5}},
    {C::zero_offset, Q::zero_offset, Scalar{-10 * wire::degrees_to_rad}, {0xfc, 0x18}},
  };
  for (const auto& entry : options) {
    const auto code = static_cast<std::uint8_t>(entry.code);
    const auto packet = p.configure(1, entry.code, entry.value, offline);
    std::vector<std::uint8_t> expected_tx{0xc0, code};
    expected_tx.insert(expected_tx.end(), entry.raw.begin(), entry.raw.end());
    bytes(packet.frame, 1, expected_tx);
    std::vector<std::uint8_t> expected_ack{0xff, 0xfe, code};
    expected_ack.insert(expected_ack.end(), entry.raw.begin(), entry.raw.end());
    rig.verify(packet, 1, expected_ack);
    std::vector<std::uint8_t> expected_read{0xa0, static_cast<std::uint8_t>(entry.query)};
    expected_read.insert(expected_read.end(), entry.raw.begin(), entry.raw.end());
    rig.verify(p.query(1, entry.query), 1, expected_read);
  }
  for (std::uint8_t group = 1; group <= 10; ++group) {
    const auto write = p.configure(1, C::kt_segment, KtWrite{group, {0, 40.95, 40.95, 0}}, offline);
    rig.verify(write, 1, {0xee, group, 0, 0x0f, 0xff, 0xff, 0xf0, 0});
  }
  std::this_thread::sleep_for(245ms);
  for (std::uint8_t group = 1; group <= 10; ++group) {
    rig.verify(p.query(1, Q::kt_segment, group), 1, {0xee, group, 0, 0x0f, 0xff, 0xff, 0xf0, 0});
  }
  rig.verify(p.query(1, Q::kt_calibration), 1, {0xa0, 35, 1});
  rig.verify(p.configure(1, C::kt_calibration, Enumeration{2}, offline), 1, {0xff, 0xfe, 0x0f, 2});
  std::this_thread::sleep_for(245ms);
  CHECK(rig.run(p.query(1, Q::kt_calibration), S::timed_out, 12ms).observed == 0);
  CHECK(rig.run(p.query(1, Q::kt_segment, 8), S::timed_out, 12ms).observed == 0);
  try {
    p.configure(1, C::acceleration, Scalar{20}, offline);
    CHECK(false);
  } catch (const CodecError& e) {
    CHECK(e.issue == CodecIssue::ambiguous);
  }
  CHECK(rig.run(p.configure(1, C::communication, Enumeration{1}, offline), S::timed_out, 20ms).observed == 0);
}

void controls_queries(const std::string& name, WireFormat format) {
  Rig rig(name, format);
  auto& p = rig.codec;
  for (const auto q : {Q::position, Q::velocity, Q::phase_current, Q::power}) {
    rig.verify(p.query(1, q), 1, {0xa0, static_cast<std::uint8_t>(q), 0, 0, 0, 0});
  }
  rig.verify(p.query(1, Q::acceleration), 1, {0xa0, 5, 7, 0xd0});
  rig.verify(p.query(1, Q::version), 1, {0xa0, 30, 1, 0, 0, 1, 19, 0});
  rig.verify(p.query(1, Q::brake), 1, {0xa0, 37, 1});
  rig.verify(p.query(1, Q::encoder), 1, {0xa0, 39, 1, 0, 0, 0, 0});
  const auto parts = rig.run(p.query(1, Q::uuid));
  CHECK(parts.observed == 3 && parts.event.uuid == (std::array<std::uint8_t, 12>{0x45,0x4e,0x43,0x4f,0x53,2,0,1,2,3,4,5}));

  const auto mix = p.control(1, Mixed{20, 5, 0, 0, 0}, profile());
  bytes(mix.frame, 1, {1, 0x47, 0xff, 0x7f, 0xff, 0x7f, 0xf7, 0xff});
  rig.verify(mix, 1, {0x20, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 100, 104});
  const auto pos = p.control(1, Position{-90 * wire::degrees_to_rad, 20 * wire::rpm_to_rad_s, 5, Ack::type2});
  bytes(pos.frame, 1, {0x38, 0x56, 0x80, 0, 0, 0x32, 0, 0xca});
  rig.verify(pos, 1, {0x40, 0xc2, 0xb4, 0, 0, 0, 0, 100});
  const auto speed = p.control(1, Velocity{-60 * wire::rpm_to_rad_s, 5, Ack::type3});
  bytes(speed.frame, 1, {0x43, 0xc2, 0x70, 0, 0, 0, 0x32});
  rig.verify(speed, 1, {0x60, 0xc2, 0x70, 0, 0, 0, 0, 100});
  const auto current = p.control(1, Drive{DriveMode::current, -5, Ack::type2});
  bytes(current.frame, 1, {0x62, 0xfe, 0x0c});
  const auto i = std::get<Telemetry>(rig.run(current).event.reply->feedback);
  near(i.phase_current_a->value, -5);
  const auto torque = p.control(1, Drive{DriveMode::torque, 5, Ack::type2});
  bytes(torque.frame, 1, {0x66, 1, 0xf4});
  near(std::get<Telemetry>(rig.run(torque).event.reply->feedback).phase_current_a->value, 2);
  for (const auto sub : {DriveMode::damping, DriveMode::dissipative, DriveMode::regenerative}) {
    const auto packet = p.control(1, Drive{sub, sub == DriveMode::damping ? 0.0 : 5.0, Ack::type3});
    rig.verify(packet, 1, {0x60, 0, 0, 0, 0, 0, 0, 100});
  }
  rig.verify(p.brake(1, BrakeState::engaged, offline), 1, {0xb2, 0});
  rig.verify(p.control(1, Drive{DriveMode::brake, 1, Ack::type1}), 1, {0xc0, 1});
  CHECK(rig.run(p.control(1, Velocity{0, 0, Ack::none})).observed == 0);
  CHECK(rig.bus.receive_for(10ms).status == ReceiveStatus::timeout);
}

void faults(const std::string& name, WireFormat format) {
  for (bool FaultOptions::* member : {&FaultOptions::silent, &FaultOptions::wrong_id,
                                      &FaultOptions::short_length, &FaultOptions::unknown_type}) {
    FaultOptions injection;
    injection.*member = true;
    Rig rig(name, format, injection);
    const auto result = rig.run(rig.codec.query(1, Q::position), S::timed_out, 15ms);
    CHECK(result.observed == (member == &FaultOptions::silent ? 0u : 1u));
  }
  {
    FaultOptions injection;
    injection.drop_every = 2;
    Rig rig(name, format, injection);
    rig.run(rig.codec.query(1, Q::position));
    CHECK(rig.run(rig.codec.query(1, Q::velocity), S::timed_out, 15ms).observed == 0);
    rig.run(rig.codec.query(1, Q::phase_current));
  }
  {
    FaultOptions injection;
    injection.delay = 45ms;
    Rig rig(name, format, injection);
    const auto expired = rig.codec.query(1, Q::position);
    CHECK(rig.run(expired, S::timed_out, 10ms).observed == 0);
    CHECK(rig.tx.begin(expired, Clock::now(), Clock::now() + 10ms) == Begin::quarantined);
    const auto following = rig.run(rig.codec.query(1, Q::velocity));
    CHECK(following.observed == 2 && following.event.reply);
    CHECK(std::get<QueryReply>(following.event.reply->feedback).code == Q::velocity);
  }
  {
    FaultOptions injection;
    injection.uuid_reverse = true;
    injection.duplicate = true;
    Rig rig(name, format, injection);
    const auto reversed = rig.run(rig.codec.query(1, Q::uuid));
    CHECK(reversed.observed == 5 && reversed.event.uuid && (*reversed.event.uuid)[0] == 0x45);
    CHECK(rig.bus.receive_for(10ms).status == ReceiveStatus::frame);
  }
  {
    FaultOptions injection;
    injection.uuid_missing = true;
    Rig rig(name, format, injection);
    CHECK(rig.run(rig.codec.query(1, Q::uuid), S::timed_out, 15ms).observed == 2);
  }
  {
    Rig rig(name, format, {}, ModelOptions{1, format, MotorError::overcurrent});
    const auto fault = rig.run(rig.codec.query(1, Q::phase_current), S::rejected);
    CHECK(std::get<QueryReply>(fault.event.reply->feedback).error == MotorError::overcurrent);
    const auto fail = rig.run(rig.codec.configure(1, C::kt, Scalar{2.3}, offline), S::rejected);
    bytes(fail.event.reply->raw.frame, 1, {0x82, 4, 0});
    const auto zero = rig.run(rig.codec.zero(1, {}, offline), S::rejected);
    bytes(zero.event.reply->raw.frame, 0x7ff, {0, 1, 1, 0});
  }
  {
    Rig rig(name, format);
    const auto restart = rig.codec.control(1, Velocity{0.25, 1, Ack::type3});
    const auto value = [](const Exchange& e) {
      return std::get<Scalar>(std::get<QueryReply>(e.event.reply->feedback).value).value;
    };
    rig.run(restart);
    near(value(rig.run(rig.codec.query(1, Q::velocity))), 0.25, 1e-6);
    std::this_thread::sleep_for(520ms);
    near(value(rig.run(rig.codec.query(1, Q::velocity))), 0);
  }
}

void safety_on_bus(const std::string& name, WireFormat format) {
  SocketCanTransport source({name, format, {}, true});
  SocketCanTransport target({name, format, {}, true});
  const auto observe = [&](SafetyManager& guard, const std::vector<std::uint8_t>& raw) {
    CHECK(source.send(frame(1, raw, format)) == SendStatus::accepted);
    const auto received = target.receive_for(70ms);
    CHECK(received.sample);
    guard.observe(*received.sample, Clock::now());
  };
  Limits limits{{-1, 1}, 2, 3, 5, 70, 25, 2, 70, 25ms, 50ms, 300ms};
  VerifiedMotor motor{1, format, Version{{1, 2, 3}, {4, 5, 6}}, 500, true,
                      BrakeState::released, true, profile()};
  const std::vector<std::uint8_t> normal{0x20, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 100, 104};
  const auto armed = [&] {
    SafetyManager guard(limits);
    guard.connect();
    CHECK(guard.set_verified_motor(motor, Clock::now()));
    observe(guard, normal);
    CHECK(guard.arm(1, true, Clock::now()));
    return guard;
  };
  auto hot = armed();
  observe(hot, {0x20, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 230, 104});
  CHECK(hot.state() == SafetyState::stopping && hot.fault() == SafetyFault::limits);
  CHECK(!hot.reset_fault(true));
  CHECK(hot.confirm_stop(true) && hot.reset_fault(true));
  CHECK(!hot.arm(1, true, Clock::now()));
  auto error = armed();
  observe(error, {0x21, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 100, 104});
  CHECK(error.fault() == SafetyFault::motor);
  auto exceeded = armed();
  const auto exceeded_generated = Clock::now();
  CHECK(!exceeded.submit(1, Position{1.25, 0.5, 1, Ack::type2}, exceeded_generated, Clock::now()));
  CHECK(exceeded.fault() == SafetyFault::limits);
  CHECK(source.receive_for(2ms).status == ReceiveStatus::timeout);
  auto expired = armed();
  const auto generated = Clock::now();
  const auto packet = expired.submit(1, Position{0.1, 0.5, 1, Ack::type2}, generated, Clock::now());
  CHECK(packet && target.send(packet->frame) == SendStatus::accepted);
  CHECK(source.receive_for(20ms).status == ReceiveStatus::frame);
  std::this_thread::sleep_for(56ms);
  expired.tick(Clock::now());
  CHECK(expired.state() == SafetyState::stopping && expired.fault() == SafetyFault::stale_feedback);
}

void link_down(const std::string& name) {
  if (name != "vcan0" || !std::getenv("ENCOS_TEST_ADMIN")) {
    std::cout << "link-down injection skipped (requires private namespace with ENCOS_TEST_ADMIN)\n";
    return;
  }
  SocketCanTransport bus({name, WireFormat::classic, {}, true});
  CHECK(std::system("ip link set dev vcan0 down") == 0);
  const auto sent = bus.send_with_status(Protocol{}.query(1, Q::position).frame);
  const auto start = Clock::now();
  const auto read = bus.receive_for(30ms);
  const bool up = bus.connected();
  const auto elapsed = Clock::now() - start;
  bool opening_down_failed = false;
  try {
    SocketCanTransport down({name, WireFormat::classic, {}, true});
  } catch (const std::system_error& error) {
    opening_down_failed = error.code().value() == ENETDOWN;
  }
  CHECK(std::system("ip link set dev vcan0 up") == 0);
  CHECK(sent.status == SendStatus::disconnected && !up && opening_down_failed && elapsed < 1s);
  CHECK(read.status == ReceiveStatus::disconnected);
  SafetyManager guard({{-1, 1}, 2, 3, 5, 70, 25, 2, 70, 20ms, 50ms, 200ms});
  guard.connect();
  guard.bus_fault();
  CHECK(guard.state() == SafetyState::fault && !guard.arm(1, true, Clock::now()));
  CHECK(guard.reset_fault(true) && guard.state() == SafetyState::unarmed);
  CHECK(!guard.arm(1, true, Clock::now()));
}

}  // namespace

int main() try {
  const char* override = std::getenv("ENCOS_TEST_VCAN");
  const std::string name = override ? override : "vcan0";
  try {
    SocketCanTransport missing({"encos-missing", WireFormat::classic, {}, true});
    CHECK(false);
  } catch (const std::system_error& error) {
    CHECK(error.code().value() == ENODEV);
  }
  try {
    SocketCanTransport rejected({"lo", WireFormat::classic, {}, true});
    CHECK(false);
  } catch (const std::system_error& error) {
    CHECK(error.code().value() == EPERM);
  }
  try {
    SocketCanTransport probe({name, WireFormat::fd_brs, {}, true});
  } catch (const std::system_error& error) {
    if (!override) {
      std::cerr << "vcan integration unavailable: " << error.what() << '\n';
      return 77;
    }
    throw;
  }
  for (const auto format : {WireFormat::classic, WireFormat::fd_brs}) {
    socket_options(name, format);
    management(name, format);
    independent_replies(name, format);
    configurations(name, format);
    controls_queries(name, format);
    faults(name, format);
    safety_on_bus(name, format);
    std::cout << (format == WireFormat::classic ? "classic CAN" : "CAN FD+BRS")
              << ": SocketCAN, management, configuration, control, queries, fault injection, safety PASS\n";
  }
  link_down(name);
  return 0;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
