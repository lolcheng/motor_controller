#pragma once

#include "encos_core/transport.hpp"

#include <memory>
#include <string>
#include <vector>

namespace encos {

struct SocketCanOptions {
  std::string interface{"vcan0"};
  WireFormat format{WireFormat::classic};
  std::vector<std::uint16_t> ids;  // Empty accepts all standard data IDs.
  bool error_frames{true};
};

struct SocketSend {
  SendStatus status;
  int error{};
};

enum class ReceiveStatus { frame, timeout, interrupted, disconnected, busy, io_error };

struct SocketReceive {
  ReceiveStatus status;
  std::optional<ReceivedFrame> sample;
  int error{};
};

// This P1 backend only binds verified Linux vcan devices.
class SocketCanTransport final : public Transport {
 public:
  explicit SocketCanTransport(const SocketCanOptions& options = {});
  ~SocketCanTransport() override;
  SocketCanTransport(const SocketCanTransport&) = delete;
  SocketCanTransport& operator=(const SocketCanTransport&) = delete;

  SendStatus send(const CanFrame& frame) override;
  SocketSend send_with_status(const CanFrame& frame);
  std::optional<ReceivedFrame> receive() override;
  SocketReceive receive_for(Clock::duration wait);
  void interrupt() noexcept;
  bool connected() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace encos
