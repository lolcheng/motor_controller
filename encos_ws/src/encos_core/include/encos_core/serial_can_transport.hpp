#pragma once

#include "encos_core/transaction_manager.hpp"
#include "encos_core/transport.hpp"

#include <functional>
#include <string>
#include <termios.h>

namespace encos {

// The confirmed USB wrapper carries standard CAN identifiers and at most eight data bytes.
std::vector<std::uint8_t> encode_serial_can(const CanFrame& frame);

class SerialCanTransport final : public Transport {
 public:
  explicit SerialCanTransport(const std::string& device, bool allow_queries = false);
  // On-demand terminal commands. The caller supplies each operator-requested command.
  static SerialCanTransport for_commands(const std::string& device);
  ~SerialCanTransport() override;
  SerialCanTransport(const SerialCanTransport&) = delete;
  SerialCanTransport& operator=(const SerialCanTransport&) = delete;

  SendStatus send(const CanFrame& frame) override;
  SendStatus send_command(const Encoded& command);
  std::optional<ReceivedFrame> receive() override;
  std::optional<ReceivedFrame> receive_for(Clock::duration wait);
  bool connected() const override { return connected_; }
  // Borrow for poll only. Serial I/O and ownership stay with this transport.
  int poll_descriptor() const { return fd_; }
  bool discard_input();

 private:
  enum class Access { monitor, queries, commands };
  SerialCanTransport(const std::string& device, Access access);
  SendStatus write_frame(const CanFrame& frame);
  std::optional<ReceivedFrame> extract();
  int fd_;
  Access access_;
  bool connected_{true};
  termios original_{};
  std::vector<std::uint8_t> input_;
  TimePoint last_read_{};
};

// One caller owns the serial stream. Reuse the transaction manager for the life of the session.
TransactionEvent serial_query(SerialCanTransport& bus, TransactionManager& transactions,
                              const Protocol& codec, const Encoded& request, Clock::duration timeout,
                              const std::function<void(const Decoded&)>& observed = {});

}  // namespace encos
