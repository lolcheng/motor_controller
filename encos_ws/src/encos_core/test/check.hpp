#pragma once

#include "encos_core/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <limits>

#define CHECK(condition) do { \
  if (!(condition)) { \
    std::cerr << __FILE__ << ':' << __LINE__ << ": " << #condition << '\n'; \
    std::exit(EXIT_FAILURE); \
  } \
} while (false)

namespace test {
using namespace encos;
using namespace std::chrono_literals;
constexpr std::uint16_t id = 1;
constexpr auto offline = WritePermission::offline;
constexpr double deg = 0.01745329251994329577;
constexpr double rpm = 0.10471975511965977462;
constexpr auto quiet_nan = std::numeric_limits<double>::quiet_NaN();
constexpr auto inf = std::numeric_limits<double>::infinity();

inline TimePoint at_ms(unsigned ms) { return TimePoint{} + std::chrono::milliseconds(ms); }

inline ReceivedFrame rx(std::initializer_list<std::uint8_t> data, std::uint32_t motor = id,
                        unsigned ms = 0, Origin origin = Origin::remote,
                        WireFormat format = WireFormat::classic) {
  ReceivedFrame sample;
  sample.frame.id = motor;
  sample.frame.length = static_cast<std::uint8_t>(data.size());
  sample.frame.kind = format == WireFormat::classic ? FrameKind::classic : FrameKind::fd;
  sample.frame.brs = format == WireFormat::fd_brs;
  std::copy(data.begin(), data.end(), sample.frame.data.begin());
  sample.stamp = at_ms(ms);
  sample.origin = origin;
  return sample;
}

inline void bytes(const Encoded& result, std::uint32_t id, std::initializer_list<std::uint8_t> data,
                  WireFormat format = WireFormat::classic) {
  CHECK(result.frame.id == id);
  CHECK(result.frame.length == data.size());
  CHECK(std::equal(data.begin(), data.end(), result.frame.data.begin()));
  CHECK(matches_wire_format(result.frame, format));
  CHECK(!result.frame.rtr && !result.frame.extended && !result.frame.error);
}

template<class Fn>
void throws(CodecIssue expected, Fn&& operation) {
  try {
    operation();
  } catch (const CodecError& e) {
    CHECK(e.issue == expected);
    return;
  }
  CHECK(false);
}

inline void near(double actual, double expected, double tolerance = 1.e-10) {
  CHECK(std::isfinite(actual) && std::abs(actual - expected) <= tolerance);
}

inline ProtocolProfile profile() {
  return {MixedRanges{{0, 500}, {0, 5}, {-12.5, 12.5}, {-18, 18}, {-30, 30}},
          Feedback1Ranges{{-12.5, 12.5}, {-18, 18}, {-90, 90}}, 2.5, 1};
}

}  // namespace test
