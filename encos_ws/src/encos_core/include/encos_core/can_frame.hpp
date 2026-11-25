#pragma once

#include "encos_core/clock.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace encos {

constexpr std::uint32_t management_id = 0x7ff;
constexpr bool valid_motor_id(std::uint32_t id) {
  return id >= 1 && id < management_id;
}

enum class WireFormat { classic, fd_brs };
enum class FrameKind { classic, fd };
enum class Origin { unknown, remote, local_echo };

struct CanFrame {
  std::uint32_t id{};
  FrameKind kind{FrameKind::classic};
  bool brs{};
  bool extended{};
  bool rtr{};
  bool error{};
  std::uint8_t length{};
  std::array<std::uint8_t, 64> data{};
  bool esi{};
};

struct ReceivedFrame {
  CanFrame frame;
  TimePoint stamp{};
  Origin origin{Origin::unknown};
  std::optional<std::chrono::nanoseconds> kernel_realtime{};
  std::optional<std::uint32_t> socket_dropped{};
  std::optional<TimePoint> userspace_stamp{};
};

inline bool valid_wire_format(WireFormat format) {
  return format == WireFormat::classic || format == WireFormat::fd_brs;
}

inline bool matches_wire_format(const CanFrame& frame, WireFormat format) {
  return (format == WireFormat::classic && frame.kind == FrameKind::classic && !frame.brs) ||
         (format == WireFormat::fd_brs && frame.kind == FrameKind::fd && frame.brs);
}

}  // namespace encos
