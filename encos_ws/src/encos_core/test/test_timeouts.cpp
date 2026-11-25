#include "check.hpp"
#include "encos_core/transaction_manager.hpp"

using namespace test;
using Q = QueryCode;
using C = ConfigCode;
using S = TransactionState;

int main() {
  const Protocol p;
  TransactionManager transactions;
  const auto feed = [&](std::initializer_list<std::uint8_t> data, unsigned ms, std::uint16_t motor = id,
                        Origin origin = Origin::remote) {
    return transactions.receive(p.decode(rx(data, motor, ms, origin)), at_ms(ms));
  };
  const auto pos = p.query(id, Q::position);
  CHECK(transactions.begin(pos, at_ms(10), at_ms(10)) == Begin::invalid_time);
  CHECK(transactions.begin({}, at_ms(10), at_ms(11)) == Begin::invalid_command);
  CHECK(transactions.begin(pos, at_ms(10), at_ms(20)) == Begin::accepted);
  CHECK(transactions.begin(p.query(id, Q::velocity), at_ms(11), at_ms(21)) == Begin::busy);
  CHECK(feed({0xa0, 1, 0, 0, 0, 0}, 11, id, Origin::local_echo).state == S::ignored);
  CHECK(feed({0xa0, 1, 0, 0, 0, 0}, 12, id, Origin::unknown).state == S::ignored);
  CHECK(feed({0xa0, 1, 0, 0, 0, 0}, 12, 2).state == S::ignored);
  CHECK(transactions.receive(p.decode(rx({0xa0, 1, 0, 0, 0, 0}, id, 10)), at_ms(12)).state == S::ignored);
  CHECK(feed({0xa0, 2, 0, 0, 0, 0}, 13).state == S::ignored);
  CHECK(feed({0x40, 0, 0, 0, 0, 0, 0, 50}, 14).state == S::ignored);
  CHECK(transactions.receive(p.decode(rx({0xa0, 1, 0, 0, 0, 0}, id, 9)), at_ms(15)).state == S::ignored);
  CHECK(transactions.receive(p.decode(rx({0xa0, 1, 0, 0, 0, 0}, id, 18)), at_ms(15)).state == S::ignored);
  CHECK(transactions.poll(at_ms(19)).state == S::ignored);
  CHECK(feed({0xa0, 1, 0, 0, 0, 0}, 20).state == S::timed_out);
  CHECK(!transactions.pending());
  CHECK(transactions.begin(pos, at_ms(21), at_ms(25)) == Begin::quarantined);

  const auto uuid = p.query(id, Q::uuid);
  CHECK(transactions.begin(uuid, at_ms(21), at_ms(40)) == Begin::accepted);
  CHECK(feed({0xa0, 1, 0, 0, 0, 0}, 22).state == S::ignored);
  CHECK(feed({0xa0, 29, 3, 8, 9, 10, 11}, 24).state == S::partial);
  CHECK(feed({0xa0, 29, 3, 8, 9, 10, 11}, 25).state == S::duplicate);
  CHECK(feed({0xa0, 29, 1, 0, 1, 2, 3}, 26).state == S::partial);
  const auto done = feed({0xa0, 29, 2, 4, 5, 6, 7}, 27);
  CHECK(done.state == S::completed && done.uuid == (std::array<std::uint8_t, 12>{0,1,2,3,4,5,6,7,8,9,10,11}));
  CHECK(transactions.begin(uuid, at_ms(28), at_ms(40)) == Begin::quarantined);

  TransactionManager bounded(20ms);
  CHECK(bounded.begin(uuid, at_ms(0), at_ms(21)) == Begin::invalid_time);
  CHECK(bounded.begin(uuid, at_ms(0), at_ms(10)) == Begin::accepted);
  const auto part = p.decode(rx({0xa0, 29, 1, 0, 1, 2, 3}, id, 1));
  CHECK(bounded.receive(part, at_ms(1)).state == S::partial);
  CHECK(bounded.poll(at_ms(10)).state == S::timed_out);
  CHECK(bounded.begin(uuid, at_ms(19), at_ms(30)) == Begin::quarantined);
  CHECK(bounded.begin(uuid, at_ms(20), at_ms(30)) == Begin::quarantined);
  CHECK(bounded.begin(uuid, at_ms(21), at_ms(31)) == Begin::accepted);
  CHECK(bounded.receive(part, at_ms(22)).state == S::ignored);
  auto part2 = p.decode(rx({0xa0, 29, 1, 0, 1, 2, 3}, id, 22));
  CHECK(bounded.receive(part2, at_ms(22)).state == S::partial);
  part2 = p.decode(rx({0xa0, 29, 1, 0, 1, 2, 4}, id, 23));
  CHECK(bounded.receive(part2, at_ms(23)).state == S::rejected);
  CHECK(bounded.begin(uuid, at_ms(24), at_ms(31)) == Begin::quarantined);
  CHECK(bounded.begin(uuid, at_ms(42), at_ms(50)) == Begin::accepted);
  CHECK(bounded.cancel().state == S::cancelled);

  CHECK(transactions.begin(p.query(id, Q::kt_segment, 2), at_ms(41), at_ms(60)) == Begin::accepted);
  CHECK(feed({0xee, 1, 0, 0, 0, 0, 0, 0}, 42).state == S::ignored);
  CHECK(feed({0xee, 0x12, 0, 0, 0, 0, 0, 0}, 43).state == S::ignored);
  CHECK(feed({0xee, 2, 0, 0, 0, 0, 0, 0}, 44).state == S::completed);
  CHECK(transactions.begin(p.configure(id, C::kt_segment, KtWrite{2, {0, 0, 0, 0}}, offline),
                           at_ms(45), at_ms(60)) == Begin::quarantined);
  CHECK(transactions.begin(p.configure(id, C::kt_segment, KtWrite{3, {0, 0, 0, 0}}, offline),
                           at_ms(45), at_ms(60)) == Begin::accepted);
  CHECK(feed({0xee, 3, 0, 0, 0, 0, 0, 1}, 46).state == S::rejected);
  CHECK(transactions.begin(p.configure(id, C::kt, Scalar{2.5}, offline), at_ms(47), at_ms(65)) == Begin::accepted);
  CHECK(feed({0xff, 0xfe, 1, 0, 0xfa}, 48).state == S::ignored);
  CHECK(feed({0xff, 0xfe, 4, 0, 0xfa}, 49).state == S::completed);
  CHECK(transactions.begin(p.configure(id, C::timeout, Scalar{500}, offline), at_ms(50), at_ms(65)) == Begin::accepted);
  CHECK(feed({0x80, 11, 0}, 51).state == S::rejected);
  CHECK(transactions.begin(p.configure(id, C::kd_range, Pair{0, 50}, offline), at_ms(52), at_ms(65)) == Begin::accepted);
  CHECK(feed({0xff, 0xfe, 6, 0, 0, 0, 40}, 53).state == S::rejected);
  CHECK(transactions.begin(p.configure(id, C::communication, Enumeration{1}, offline),
                           at_ms(54), at_ms(60)) == Begin::accepted);
  CHECK(feed({0xff, 0xfe, 2, 1}, 55).state == S::ignored);
  CHECK(feed({0xff, 0xfe, 2, 1, 0}, 56).state == S::ignored);
  CHECK(transactions.poll(at_ms(60)).state == S::timed_out);
  CHECK(transactions.begin(p.discover(), at_ms(553), at_ms(560)) == Begin::wait_after_write);
  CHECK(transactions.begin(p.discover(), at_ms(554), at_ms(560)) == Begin::accepted);
  CHECK(feed({0x80, 0x80, 1, 0x80}, 555, 0x7ff).state == S::rejected);
  CHECK(transactions.begin(p.set_id(id, 2, offline), at_ms(556), at_ms(560)) == Begin::accepted);
  CHECK(feed({0, id, 1, 4}, 557, 0x7ff).state == S::ignored);
  CHECK(feed({0, 2, 1, 4}, 558, 0x7ff).state == S::completed);
  CHECK(transactions.begin(p.discover(), at_ms(1057), at_ms(1060)) == Begin::wait_after_write);
  CHECK(transactions.begin(p.zero(id, {}, offline), at_ms(1058), at_ms(1060)) == Begin::quarantined);
  CHECK(transactions.begin(p.zero(2, {}, offline), at_ms(1058), at_ms(1065)) == Begin::accepted);
  CHECK(feed({0, id, 1, 0}, 1059, 0x7ff).state == S::ignored);
  CHECK(feed({0, 2, 1, 0}, 1060, 0x7ff).state == S::rejected);
  CHECK(transactions.begin(p.reset_id(offline), at_ms(1559), at_ms(1570)) == Begin::wait_after_write);
  CHECK(transactions.begin(p.reset_id(offline), at_ms(1560), at_ms(1570)) == Begin::accepted);
  CHECK(transactions.poll(at_ms(1570)).state == S::timed_out);

  CHECK(transactions.begin(p.brake(id, BrakeState::released, offline), at_ms(2059), at_ms(2070)) ==
        Begin::wait_after_write);
  CHECK(transactions.begin(p.brake(id, BrakeState::released, offline), at_ms(2060), at_ms(2070)) == Begin::accepted);
  CHECK(feed({0xc0, 1}, 2061).state == S::ignored);
  CHECK(feed({0xb2, 0}, 2062).state == S::rejected);
  const auto brake = p.control(id, Drive{DriveMode::brake, 1, Ack::none});
  CHECK(transactions.begin(brake, at_ms(2063), at_ms(2070)) == Begin::accepted);
  CHECK(feed({0xb2, 1}, 2064).state == S::ignored);
  CHECK(feed({0xc0, 1}, 2065).state == S::completed);
  const auto speed = p.control(id, Velocity{0, 0, Ack::type3});
  CHECK(transactions.begin(speed, at_ms(2066), at_ms(2070)) == Begin::accepted);
  CHECK(feed({0x40, 0, 0, 0, 0, 0, 0, 50}, 2067).state == S::ignored);
  CHECK(feed({0x62, 0, 0, 0, 0, 0, 0, 50}, 2068).state == S::rejected);
  CHECK(transactions.begin(p.control(id, Velocity{}), at_ms(2069), at_ms(2070)) == Begin::accepted);
  CHECK(!transactions.pending());
}
