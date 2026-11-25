#include "test_support.hpp"

#include <iostream>

using namespace p1_test;

int main() try {
  for (const auto format : {WireFormat::classic, WireFormat::fd_brs}) {
    VirtualMotor model({1, format});
    const auto handle = [&](unsigned ms, const std::vector<std::uint8_t>& raw,
                            std::uint16_t id = 1, Origin origin = Origin::remote) {
      return model.handle({frame(id, raw, format), at(ms), origin}, at(ms));
    };
    CHECK(handle(0, {0xe1, 1}, 1, Origin::local_echo).empty());
    CHECK(handle(0, {0xe1, 1}, 2).empty());
    CHECK(handle(0, {0xe1, 0}).empty());
    CHECK(handle(0, {0xe1, 1, 0}).empty());
    CHECK(handle(0, {0xe0, 1}).empty());
    CHECK(handle(0, {0xc0, 1, 7, 0xd0}).empty());
    CHECK(handle(0, {0xc0, 0x11, 0x03}).empty());
    CHECK(handle(0, {0xec, 36}).empty());
    CHECK(handle(0, {0x79, 0, 0}).empty());
    CHECK(handle(0, {0x7d, 0, 0}).empty());
    CHECK(handle(0, {0x41, 0x7f, 0xc0, 0, 0, 0, 0}).empty());
    CHECK(handle(0, {0x45, 0, 0, 0, 0, 0, 0}).empty());
    // Vendor p.44 -60 rpm, type 3. Output float bytes independently specified.
    const auto speed = handle(1, {0x43, 0xc2, 0x70, 0, 0, 0, 0x32});
    CHECK(speed.size() == 1 && matches_wire_format(speed.front(), format));
    bytes(speed.front(), 1, {0x60, 0xc2, 0x70, 0, 0, 0, 0, 100});
    near(model.state().velocity_rad_s, -60 * wire::rpm_to_rad_s);
    CHECK(model.state().active);
    bytes(handle(2, {0, 1, 0, 3}, 0x7ff).front(), 0x7ff, {0, 1, 1, 0});
    bytes(handle(3, {0xc0, 4, 0, 0xfa}).front(), 1, {0x80, 4, 0});
    CHECK(handle(4, {0x7f, 0x7f, 0, 5, 0x7f, 0x7f}, 0x7ff).empty());
    model.advance(at(501));
    CHECK(!model.state().active && model.state().velocity_rad_s == 0 && model.state().phase_current_a == 0);
    near(model.state().position_rad, -30 * wire::rpm_to_rad_s);
    model.advance(at(800));
    near(model.state().position_rad, -30 * wire::rpm_to_rad_s);

    bytes(handle(801, {0x80, 0}).front(), 1, {0xb2, 0});
    CHECK(handle(802, {0x40, 0x42, 0x48, 0, 0, 0, 0x64}).empty());
    CHECK(model.state().brake == BrakeState::engaged && !model.state().active);
    bytes(handle(803, {0x75, 0, 1}).front(), 1, {0xc0, 1});
    CHECK(model.state().brake == BrakeState::released);
    bytes(handle(804, {0xc0, 0x0f, 2}).front(), 1, {0xff, 0xfe, 0x0f, 2});
    CHECK(handle(805, {0xe1, 35}).empty() && handle(805, {0xe1, 36}).empty());
    CHECK(handle(806, {0xc1, 2, 1}).empty());
    CHECK(handle(1305, {0xe1, 37}).empty());
    bytes(handle(1306, {0xe1, 37}).front(), 1, {0xa0, 37, 1});
    const auto queued_motion = frame(1, {0x43, 0xc2, 0x70, 0, 0, 0, 0x32}, format);
    CHECK(model.handle({queued_motion, at(1306), Origin::remote}, at(1357)).empty());
    CHECK(!model.state().active);
  }

  FaultOptions option;
  option.duplicate = true;
  option.delay = 15ms;
  option.uuid_missing = true;
  option.uuid_reverse = true;
  FaultQueue queue(option);
  VirtualMotor model;
  auto parts = model.handle({frame(1, {0xe1, 29}), at(1), Origin::remote}, at(1));
  queue.enqueue(std::move(parts), at(1));
  CHECK(queue.next() == at(16) && !queue.take(at(15)));
  for (const unsigned index : {3u, 3u, 1u, 1u}) {
    const auto item = queue.take(at(16));
    CHECK(item && item->data[2] == index);
  }
  CHECK(!queue.take(at(16)) && queue.dropped() == 1);

  FaultOptions drop;
  drop.drop_every = 2;
  FaultQueue every_two(drop);
  for (unsigned i = 0; i < 4; ++i) every_two.enqueue({frame(1, {0x40, 2})}, at(i));
  CHECK(every_two.take(at(4)) && every_two.take(at(4)) && !every_two.take(at(4)));
  CHECK(every_two.dropped() == 2);

  FaultOptions full;
  full.delay = 20ms;
  FaultQueue scheduled(full);
  for (unsigned i = 0; i < 260; ++i) scheduled.enqueue({frame(1, {1, 2})}, at(1));
  CHECK(scheduled.dropped() == 4 && scheduled.next() == at(21));

  std::cout << "virtual model fixtures, watchdog, command rejection and deterministic faults passed\n";
  return 0;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
