#pragma once

#include <array>
#include <atomic>
#include <optional>
#include <utility>

namespace encos::driver {
// One producer and one consumer. Full queues drop new entries instead of waiting.
template<class T, std::size_t Capacity>
class SampleQueue {
 public:
  bool push(T value) {
    const auto tail = tail_.load(std::memory_order_relaxed);
    const auto next = (tail + 1) % Capacity;
    if (next == head_.load(std::memory_order_acquire)) return false;
    slots_[tail] = std::move(value);
    tail_.store(next, std::memory_order_release);
    return true;
  }
  std::optional<T> take() {
    const auto head = head_.load(std::memory_order_relaxed);
    if (head == tail_.load(std::memory_order_acquire)) return {};
    auto value = std::move(slots_[head]);
    head_.store((head + 1) % Capacity, std::memory_order_release);
    return value;
  }
 private:
  static_assert(Capacity > 1);
  std::array<T, Capacity> slots_;
  alignas(64) std::atomic<std::size_t> head_{};
  alignas(64) std::atomic<std::size_t> tail_{};
};
}  // namespace encos::driver
