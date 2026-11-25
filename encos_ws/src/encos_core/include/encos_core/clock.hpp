#pragma once

#include <chrono>

namespace encos {
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
}  // namespace encos
