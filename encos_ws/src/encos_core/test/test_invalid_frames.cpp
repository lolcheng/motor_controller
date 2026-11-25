#include "check.hpp"

using namespace test;

int main() {
  const Protocol p;
  const auto issue = [&](CodecIssue expected, const ReceivedFrame& frame) {
    const auto result = p.decode(frame);
    CHECK(std::get<CodecIssue>(result.feedback) == expected);
    CHECK(result.raw.frame.id == frame.frame.id && result.raw.frame.data == frame.frame.data);
    CHECK(result.raw.frame.length == frame.frame.length && result.raw.stamp == frame.stamp);
  };
  const auto odd = [&](CodecIssue expected, std::initializer_list<std::uint8_t> data) {
    issue(expected, rx(data));
  };
  auto sample = rx({0xa0, 3, 0, 0, 0, 0});
  for (bool CanFrame::* flag : {&CanFrame::extended, &CanFrame::rtr, &CanFrame::error, &CanFrame::brs}) {
    auto bad = sample;
    bad.frame.*flag = true;
    issue(CodecIssue::invalid_frame, bad);
  }
  for (auto invalid : {0u, 0x800u, 0x10000001u}) {
    auto bad = sample;
    bad.frame.id = invalid;
    issue(CodecIssue::invalid_frame, bad);
  }
  auto bad = sample;
  bad.frame.kind = static_cast<FrameKind>(10);
  issue(CodecIssue::invalid_frame, bad);
  issue(CodecIssue::invalid_frame, rx({0xa0, 0x23, 1}, id, 0, Origin::remote, WireFormat::fd_brs));
  CHECK(std::get<CodecIssue>(Protocol{WireFormat::fd_brs}.decode(sample).feedback) == CodecIssue::invalid_frame);
  auto no_brs = rx({0xc0, 0}, id, 0, Origin::remote, WireFormat::fd_brs);
  no_brs.frame.brs = false;
  CHECK(std::get<CodecIssue>(Protocol{WireFormat::fd_brs}.decode(no_brs).feedback) == CodecIssue::invalid_frame);
  for (auto length : {0, 1, 5, 7, 8, 9, 64, 65, 255}) {
    auto wrong = sample;
    wrong.frame.length = static_cast<std::uint8_t>(length);
    issue(CodecIssue::length, wrong);
  }
  odd(CodecIssue::length, {0x20, 0, 0, 0, 0, 0, 0});
  odd(CodecIssue::length, {0x40, 0, 0, 0, 0, 0, 0});
  odd(CodecIssue::length, {0x60, 0, 0, 0, 0, 0, 0});
  odd(CodecIssue::length, {0x80, 4});
  odd(CodecIssue::length, {0xc0, 0, 0});
  odd(CodecIssue::length, {0xb2, 0, 0});
  odd(CodecIssue::length, {0xff, 0xfe});
  odd(CodecIssue::length, {0xff, 0xfe, 2});
  odd(CodecIssue::length, {0xff, 0xfe, 17, 0});
  odd(CodecIssue::length, {0xee, 1});
  odd(CodecIssue::value, {0xc0, 2});
  odd(CodecIssue::value, {0xb2, 2});
  odd(CodecIssue::value, {0x80, 4, 2});
  odd(CodecIssue::value, {0xa0, 29, 0, 0, 0, 0, 0});
  odd(CodecIssue::value, {0xa0, 29, 4, 0, 0, 0, 0});
  odd(CodecIssue::value, {0xa0, 35, 0});
  odd(CodecIssue::value, {0xa0, 35, 4});
  odd(CodecIssue::value, {0xa0, 37, 2});
  odd(CodecIssue::value, {0xa0, 39, 2, 0, 0, 0, 0});
  odd(CodecIssue::value, {0xff, 0xfe, 15, 3});
  odd(CodecIssue::value, {0xff, 0xfe, 1, 7, 0xd1});
  odd(CodecIssue::value, {0xa0, 23, 0, 2, 0, 1});
  odd(CodecIssue::nonfinite, {0x40, 0x7f, 0x80, 0, 0, 0, 0, 0});
  odd(CodecIssue::nonfinite, {0x60, 0xff, 0x80, 0, 0, 0, 0, 0});
  odd(CodecIssue::nonfinite, {0xa0, 3, 0x7f, 0xc0, 0, 0});
  odd(CodecIssue::unsupported, {0x00, 0});
  odd(CodecIssue::unsupported, {0xe0, 0});
  odd(CodecIssue::unsupported, {0x28, 0, 0, 0, 0, 0, 0, 0});
  odd(CodecIssue::unsupported, {0xbf, 3, 0, 0, 0, 0});
  odd(CodecIssue::unsupported, {0xa0, 36, 0, 0, 0, 0, 0, 0});
  odd(CodecIssue::unsupported, {0xa0, 0, 0, 0});
  odd(CodecIssue::unsupported, {0x80, 3, 1});
  odd(CodecIssue::unsupported, {0xff, 0xfe, 3});
  issue(CodecIssue::value, rx({0x7f, 0x7f, 1, 5, 0x7f, 0x00}, 0x7ff));
  issue(CodecIssue::value, rx({0xff, 0xff, 1, 7, 0xff}, 0x7ff));
  issue(CodecIssue::unsupported, rx({0xff, 0xff, 0, 0x82}, 0x7ff));
  issue(CodecIssue::length, rx({0xff, 0xff, 1, 0x00}, 0x7ff));

  throws(CodecIssue::value, [&] { p.control(0, Position{}); });
  throws(CodecIssue::value, [&] { p.query(0x7ff, QueryCode::position); });
  throws(CodecIssue::value, [&] { p.control(id, Position{quiet_nan, 0, 0}); });
  throws(CodecIssue::value, [&] { p.control(id, Position{0, -1, 0}); });
  throws(CodecIssue::value, [&] { p.control(id, Position{0, 0, 409.6}); });
  throws(CodecIssue::value, [&] { p.control(id, Velocity{inf, 0}); });
  throws(CodecIssue::value, [&] { p.control(id, Velocity{0, 6553.6}); });
  throws(CodecIssue::value, [&] { p.control(id, Velocity{0, 0, static_cast<Ack>(4)}); });
  throws(CodecIssue::profile, [&] { p.control(id, Mixed{}); });
  throws(CodecIssue::value, [&] { p.control(id, Mixed{0, 6, 0, 0, 0}, profile()); });
  throws(CodecIssue::value, [&] { p.control(id, Mixed{0, 0, quiet_nan, 0, 0}, profile()); });
  throws(CodecIssue::value, [&] { p.control(id, Drive{DriveMode::torque, 450}); });
  throws(CodecIssue::value, [&] { p.control(id, Drive{DriveMode::damping, 0.001}); });
  throws(CodecIssue::value, [&] { p.control(id, Drive{DriveMode::dissipative, -1}); });
  throws(CodecIssue::value, [&] { p.control(id, Drive{DriveMode::brake, 2}); });
  throws(CodecIssue::value, [&] { p.brake(id, static_cast<BrakeState>(2), offline); });
  throws(CodecIssue::unsupported, [&] { p.control(id, Drive{static_cast<DriveMode>(6), 0}); });
  throws(CodecIssue::unsupported, [&] { p.control(id, Drive{static_cast<DriveMode>(7), 0}); });
  throws(CodecIssue::value, [&] { Protocol{static_cast<WireFormat>(6)}.discover(); });
}
