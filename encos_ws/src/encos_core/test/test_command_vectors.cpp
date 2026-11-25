#include "check.hpp"

using namespace test;

int main() {
  const Protocol p;
  const Protocol fd(WireFormat::fd_brs);
  // Vendor §7.2–§8.1, printed pp.23–25, and signed offset derived from §7.2.
  bytes(p.zero(1, {}, offline), 0x7ff, {0, 1, 0, 3});
  bytes(p.zero(0x7fe, -10 * deg, offline), 0x7ff, {7, 0xfe, 0, 3, 0xfc, 0x18});
  bytes(p.set_id(1, 2, offline), 0x7ff, {0, 1, 0, 4, 0, 2});
  bytes(p.reset_id(offline), 0x7ff, {0x7f, 0x7f, 0, 5, 0x7f, 0x7f});
  bytes(p.discover(), 0x7ff, {0xff, 0xff, 0, 0x82});
  bytes(fd.discover(), 0x7ff, {0xff, 0xff, 0, 0x82}, WireFormat::fd_brs);

  // Vendor §9.1.1 p.26 with the example's explicitly supplied ranges.
  bytes(p.control(id, Mixed{20, 5, 0, 0, 0}, profile()), id,
        {0x01, 0x47, 0xff, 0x7f, 0xff, 0x7f, 0xf7, 0xff});
  // p.43 inserts raw values and even gives two different velocity words.
  const std::array<std::uint8_t, 8> conflicting_example{0, 0x28, 0x0a, 0, 5, 1, 0xf0, 0x0a};
  CHECK(wire::read_bits(conflicting_example.data(), 3, 12) == 20);
  CHECK(wire::read_bits(conflicting_example.data(), 40, 12) == 31);
  throws(CodecIssue::value, [&] { p.control(id, Mixed{20, 10, 5, 30, 10}, profile()); });
  bytes(p.control(id, Mixed{500, 5, 12.5, 18, 30}, profile()), id,
        {0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff});
  bytes(p.control(id, Mixed{0, 0, -12.5, -18, -30}, profile()), id, {0, 0, 0, 0, 0, 0, 0, 0});
  auto custom = profile();
  custom.mixed->kp = {0, 40};
  CHECK(p.control(id, Mixed{20, 5, 0, 0, 0}, custom).frame.data[0] == 0x0f);

  // Vendor §9.1.2–§9.1.5 pp.27–29 and §11 pp.43–44.
  bytes(p.control(id, Position{0, 50 * rpm, 10, Ack::type1}), id, {0x20, 0, 0, 0, 0, 0x7d, 1, 0x91});
  bytes(p.control(id, Position{0, 20 * rpm, 5, Ack::type2}), id, {0x20, 0, 0, 0, 0, 0x32, 0, 0xca});
  bytes(p.control(id, Position{90 * deg, 20 * rpm, 5, Ack::type2}), id, {0x28, 0x56, 0x80, 0, 0, 0x32, 0, 0xca});
  bytes(p.control(id, Position{-90 * deg, 20 * rpm, 5, Ack::type2}), id, {0x38, 0x56, 0x80, 0, 0, 0x32, 0, 0xca});
  bytes(p.control(id, Velocity{50 * rpm, 10, Ack::type1}), id, {0x41, 0x42, 0x48, 0, 0, 0, 0x64});
  bytes(p.control(id, Velocity{20 * rpm, 5, Ack::type3}), id, {0x43, 0x41, 0xa0, 0, 0, 0, 0x32});
  bytes(p.control(id, Velocity{60 * rpm, 5, Ack::type3}), id, {0x43, 0x42, 0x70, 0, 0, 0, 0x32});
  bytes(fd.control(id, Velocity{-60 * rpm, 5, Ack::type3}), id,
        {0x43, 0xc2, 0x70, 0, 0, 0, 0x32}, WireFormat::fd_brs);
  bytes(p.control(id, Drive{DriveMode::current, 5, Ack::type1}), id, {0x61, 1, 0xf4});
  bytes(p.control(id, Drive{DriveMode::torque, 5, Ack::type1}), id, {0x65, 1, 0xf4});
  bytes(p.control(id, Drive{DriveMode::damping, 0, Ack::type1}), id, {0x69, 0, 0});
  bytes(p.control(id, Drive{DriveMode::dissipative, 5, Ack::type1}), id, {0x6d, 1, 0xf4});
  bytes(p.control(id, Drive{DriveMode::regenerative, 5, Ack::type1}), id, {0x71, 1, 0xf4});
  bytes(p.control(id, Drive{DriveMode::brake, 1, Ack::type1}), id, {0x75, 0, 1});
  bytes(p.brake(id, BrakeState::released, offline), id, {0x80, 1});
  bytes(p.brake(id, BrakeState::engaged, offline), id, {0x80, 0});
  bytes(p.control(id, Drive{DriveMode::current, 3, Ack::type2}), id, {0x62, 1, 0x2c});
  bytes(p.control(id, Drive{DriveMode::current, 5, Ack::type2}), id, {0x62, 1, 0xf4});
  bytes(p.control(id, Drive{DriveMode::current, -5, Ack::type2}), id, {0x62, 0xfe, 0x0c});
  bytes(p.control(id, Drive{DriveMode::torque, 3, Ack::type2}), id, {0x66, 1, 0x2c});
  bytes(p.control(id, Drive{DriveMode::torque, 5, Ack::type2}), id, {0x66, 1, 0xf4});
  bytes(p.control(id, Drive{DriveMode::torque, -5, Ack::type2}), id, {0x66, 0xfe, 0x0c});

  bytes(p.control(id, Position{0, 3276.7 * rpm, 409.5, Ack::type3}), id, {0x20, 0, 0, 0, 0x1f, 0xff, 0xff, 0xff});
  bytes(p.control(id, Velocity{0, 6553.5, Ack::none}), id, {0x40, 0, 0, 0, 0, 0xff, 0xff});
  bytes(p.control(id, Drive{DriveMode::torque, -327.68, Ack::none}), id, {0x64, 0x80, 0});
  bytes(p.control(id, Drive{DriveMode::torque, 327.67, Ack::none}), id, {0x64, 0x7f, 0xff});
  for (unsigned i = 0; i < 4; ++i) {
    const auto ack = static_cast<Ack>(i);
    const auto pos = p.control(id, Position{0, 0, 0, ack});
    const auto speed = p.control(id, Velocity{0, 0, ack});
    const auto current = p.control(id, Drive{DriveMode::current, 0, ack});
    CHECK((pos.frame.data[7] & 3) == i && (speed.frame.data[0] & 3) == i && (current.frame.data[0] & 3) == i);
    CHECK(pos.reply.kind == (i ? ReplyKind::telemetry : ReplyKind::none));
    CHECK(p.control(id, Drive{DriveMode::brake, 0, ack}).reply.code == 6);
  }
}
