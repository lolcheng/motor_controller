#pragma once

#include "encos_tools/runtime.hpp"
#include "encos_tools/virtual_motor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#define CHECK(expr) do { if (!(expr)) throw std::runtime_error( \
  std::string(__FILE__) + ':' + std::to_string(__LINE__) + ": " + #expr); } while (false)

namespace p1_test {
using namespace encos;
using namespace encos::tools;
using namespace std::chrono_literals;
using Q = QueryCode;
using C = ConfigCode;
using S = TransactionState;
constexpr auto offline = WritePermission::offline;

inline CanFrame frame(std::uint32_t id, std::vector<std::uint8_t> payload,
                      WireFormat format = WireFormat::classic) {
  CanFrame raw;
  CHECK(payload.size() <= raw.data.size());
  raw.id = id;
  raw.length = static_cast<std::uint8_t>(payload.size());
  raw.kind = format == WireFormat::classic ? FrameKind::classic : FrameKind::fd;
  raw.brs = format == WireFormat::fd_brs;
  std::copy(payload.begin(), payload.end(), raw.data.begin());
  return raw;
}

inline void bytes(const CanFrame& actual, std::uint32_t id, const std::vector<std::uint8_t>& expected) {
  CHECK(actual.id == id && actual.length == expected.size());
  CHECK(std::equal(expected.begin(), expected.end(), actual.data.begin()));
}

inline TimePoint at(unsigned ms) { return TimePoint{} + std::chrono::milliseconds(ms); }

inline ProtocolProfile profile() {
  return {MixedRanges{{0, 500}, {0, 5}, {-12.5, 12.5}, {-18, 18}, {-30, 30}},
          Feedback1Ranges{{-12.5, 12.5}, {-18, 18}, {-90, 90}}, 2.5, 1};
}

inline void near(double actual, double expected, double error = 1e-8) {
  CHECK(std::abs(actual - expected) <= error);
}

}  // namespace p1_test
