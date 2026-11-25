#pragma once

#include "encos_core/socketcan_transport.hpp"
#include "encos_core/transaction_manager.hpp"

#include <atomic>
#include <charconv>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace encos::tools {

inline WireFormat parse_format(std::string_view text) {
  if (text == "classic") return WireFormat::classic;
  if (text == "fd") return WireFormat::fd_brs;
  throw std::invalid_argument("frame format must be classic or fd");
}

inline unsigned parse_unsigned(std::string_view value, unsigned max) {
  unsigned result{};
  const auto conversion = std::from_chars(value.data(), value.data() + value.size(), result);
  if (conversion.ec != std::errc{} || conversion.ptr != value.data() + value.size() || result > max) {
    throw std::invalid_argument("invalid numeric argument");
  }
  return result;
}

inline std::string hex(const CanFrame& frame) {
  std::ostringstream stream;
  stream << std::hex << std::uppercase << std::setfill('0') << std::setw(3) << frame.id << '#';
  for (std::size_t i = 0; i < frame.length && i < frame.data.size(); ++i) {
    stream << std::setw(2) << static_cast<unsigned>(frame.data[i]);
  }
  return stream.str();
}

inline std::int64_t ns(TimePoint point) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(point.time_since_epoch()).count();
}

struct Exchange {
  Begin admitted{Begin::accepted};
  SendStatus sent{SendStatus::io_error};
  TransactionEvent event;
  std::size_t observed{};
  TimePoint tx_before{}, tx_after{};
  std::optional<ReceivedFrame> last_rx;
  std::optional<std::uint32_t> socket_drops;
};

template<class Observe>
Exchange exchange(SocketCanTransport& bus, TransactionManager& transactions, const Protocol& codec,
                  const Encoded& request, const ProtocolProfile& profile, Clock::duration timeout,
                  Observe&& observe) {
  Exchange result;
  result.tx_before = Clock::now();
  const auto deadline = result.tx_before + timeout;
  result.admitted = transactions.begin(request, result.tx_before, deadline);
  if (result.admitted != Begin::accepted) {
    return result;
  }
  result.sent = bus.send(request.frame);
  result.tx_after = Clock::now();
  if (result.sent != SendStatus::accepted) {
    result.event = transactions.cancel();
    return result;
  }
  if (!transactions.pending()) {
    result.event.state = TransactionState::completed;
    return result;
  }
  while (transactions.pending()) {
    const auto now = Clock::now();
    if (now >= deadline) {
      result.event = transactions.poll(now);
      break;
    }
    const auto read = bus.receive_for(deadline - now);
    if (read.status == ReceiveStatus::disconnected || read.status == ReceiveStatus::io_error ||
        read.status == ReceiveStatus::busy || read.status == ReceiveStatus::interrupted) {
      result.event = transactions.cancel();
      break;
    }
    if (!read.sample) {
      continue;
    }
    const auto& input = *read.sample;
    result.last_rx = input;
    if (input.socket_dropped) result.socket_drops = input.socket_dropped;
    ++result.observed;
    observe(input);
    if (input.frame.error) {
      result.event = transactions.cancel();
      break;
    }
    result.event = transactions.receive(codec.decode(input, profile), Clock::now());
  }
  return result;
}

inline auto ignore_sample = [](const ReceivedFrame&) {};

}  // namespace encos::tools
