#include "encos_core/mock_transport.hpp"

namespace encos {
namespace {
template<class T>
std::optional<T> pop(std::deque<T>& queue) {
  if (queue.empty()) {
    return {};
  }
  auto value = queue.front();
  queue.pop_front();
  return value;
}
}  // namespace

SendStatus MockTransport::send(const CanFrame& frame) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_) {
    return SendStatus::disconnected;
  }
  if (tx_.size() >= capacity_) {
    return SendStatus::full;
  }
  tx_.push_back(frame);
  return SendStatus::accepted;
}

std::optional<ReceivedFrame> MockTransport::receive() {
  const std::lock_guard<std::mutex> lock(mutex_);
  return pop(rx_);
}

bool MockTransport::connected() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

void MockTransport::set_connected(bool value) {
  const std::lock_guard<std::mutex> lock(mutex_);
  connected_ = value;
  if (!connected_) {
    rx_.clear();
    tx_.clear();
  }
}

bool MockTransport::inject(const ReceivedFrame& sample) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_ || rx_.size() >= capacity_) {
    return false;
  }
  rx_.push_back(sample);
  return true;
}

std::optional<CanFrame> MockTransport::take_sent() {
  const std::lock_guard<std::mutex> lock(mutex_);
  return pop(tx_);
}

}  // namespace encos
