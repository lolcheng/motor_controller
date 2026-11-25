#include "check.hpp"

using namespace test;
using C = ConfigCode;
using Q = QueryCode;

int main() {
  const Protocol p;
  const auto cfg = [&](C code, ConfigValue value) { return p.configure(id, code, value, offline); };
  // Vendor §9.2.2–§9.2.16, pp.30–37. p.32's position example has a wrong negative word.
  bytes(cfg(C::communication, Enumeration{1}), id, {0xc1, 2, 1});
  bytes(cfg(C::communication, Enumeration{0}), id, {0xc1, 2, 0});
  bytes(cfg(C::kt, Scalar{2.5}), id, {0xc0, 4, 0, 0xfa});
  bytes(cfg(C::kp_range, Pair{0, 500}), id, {0xc0, 5, 0, 0, 1, 0xf4});
  bytes(cfg(C::kd_range, Pair{0, 50}), id, {0xc0, 6, 0, 0, 0, 0x32});
  bytes(cfg(C::position_range, Pair{-12.5, 12.5}), id, {0xc0, 7, 0xfb, 0x1e, 4, 0xe2});
  const std::uint8_t printed[] = {0xfa, 0x1e};
  CHECK(wire::read_i16(printed) == -1506);
  bytes(cfg(C::velocity_range, Pair{-18, 18}), id, {0xc0, 8, 0xf8, 0xf8, 7, 8});
  bytes(cfg(C::torque_range, Pair{-90, 90}), id, {0xc0, 9, 0xfc, 0x7c, 3, 0x84});
  bytes(cfg(C::current_range, Pair{-90, 90}), id, {0xc0, 10, 0xfc, 0x7c, 3, 0x84});
  bytes(cfg(C::timeout, Scalar{500}), id, {0xc0, 11, 1, 0xf4});
  bytes(cfg(C::timeout, Scalar{0}), id, {0xc0, 11, 0, 0});
  bytes(cfg(C::current_pi, Pair{0.01, 50}), id, {0xc0, 12, 0, 0x64, 1, 0xf4});
  bytes(cfg(C::velocity_pi, Pair{0.006, 0.1}), id, {0xc0, 13, 2, 0x58, 0x27, 0x10});
  bytes(cfg(C::position_pd, Pair{0.006, 0.00004}), id, {0xc0, 14, 2, 0x58, 0, 4});
  bytes(cfg(C::kt_calibration, Enumeration{2}), id, {0xc0, 15, 2});
  bytes(cfg(C::zero_offset, Scalar{10 * deg}), id, {0xc0, 17, 3, 0xe8});
  bytes(cfg(C::zero_offset, Scalar{-10 * deg}), id, {0xc0, 17, 0xfc, 0x18});
  bytes(cfg(C::kt_segment, KtWrite{1, {2.5, 2.6, 2.7, 2.8}}), id,
        {0xc1, 16, 0x0f, 0xa1, 4, 0x10, 0xe1, 0x18});
  for (std::uint8_t group = 1; group <= 10; ++group) {
    bytes(cfg(C::kt_segment, KtWrite{group, {0, 40.95, 40.95, 0}}), id,
          {static_cast<std::uint8_t>(0xc0 | group), 16, 0, 0x0f, 0xff, 0xff, 0xf0, 0});
    bytes(p.query(id, Q::kt_segment, group), id, {static_cast<std::uint8_t>(0xe0 | group), 36});
  }
  for (unsigned code = 0; code < 256; ++code) {
    const auto number = static_cast<std::uint8_t>(code);
    const auto q = static_cast<Q>(number);
    if ((code >= 1 && code <= 5) || (code >= 22 && code <= 39)) {
      if (q != Q::kt_segment) {
        bytes(p.query(id, q), id, {0xe1, number});
      }
    } else {
      throws(CodecIssue::unsupported, [&] { p.query(id, q); });
    }
  }
  throws(CodecIssue::ambiguous, [&] { cfg(C::acceleration, Scalar{20}); });
  throws(CodecIssue::unsupported, [&] { cfg(C::communication, Enumeration{2}); });
  throws(CodecIssue::unsupported, [&] { cfg(C::kt_calibration, Enumeration{3}); });
  throws(CodecIssue::unsupported, [&] { cfg(static_cast<C>(3), Scalar{1}); });
  throws(CodecIssue::value, [&] { cfg(C::kp_range, Pair{500, 2}); });
  throws(CodecIssue::value, [&] { cfg(C::kd_range, Pair{0, 0.1}); });
  throws(CodecIssue::value, [&] { cfg(C::kt, Pair{0, 1}); });
  throws(CodecIssue::value, [&] { cfg(C::kt_segment, KtWrite{11, {0, 1, 2, 3}}); });
  throws(CodecIssue::value, [&] { cfg(C::kt_segment, KtWrite{1, {0, 1, 2, 41}}); });
  throws(CodecIssue::value, [&] { cfg(C::zero_offset, Scalar{inf}); });
  throws(CodecIssue::value, [&] { p.query(id, Q::kt_segment); });
  throws(CodecIssue::value, [&] { p.query(id, Q::kt_segment, 11); });
  throws(CodecIssue::value, [&] { p.query(id, Q::position, 1); });
  throws(CodecIssue::permission, [&] { p.configure(id, C::kt, Scalar{1}); });
  throws(CodecIssue::permission, [&] { p.zero(id); });
  throws(CodecIssue::permission, [&] { p.set_id(id, 2); });
  throws(CodecIssue::permission, [&] { p.reset_id(); });
  throws(CodecIssue::permission, [&] { p.brake(id, BrakeState::engaged); });
}
