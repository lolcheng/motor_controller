#pragma once

#include "encos_core/transport.hpp"

#include <cstddef>
#include <deque>
#include <mutex>

namespace encos {

class MockTransport final : public Transport {
 public:
  explicit MockTransport(std::size_t capacity = 64) : capacity_(capacity) {}
  SendStatus send(const CanFrame& frame) override;
  std::optional<ReceivedFrame> receive() override;
  bool connected() const override;
  void set_connected(bool value);
  bool inject(const ReceivedFrame& sample);
  std::optional<CanFrame> take_sent();

 private:
  mutable std::mutex mutex_;
  const std::size_t capacity_;
  bool connected_{true};
  std::deque<CanFrame> tx_;
  std::deque<ReceivedFrame> rx_;
};

}  // namespace encos
