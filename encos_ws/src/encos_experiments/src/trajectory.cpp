#include "encos_experiments/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace encos::experiments {
namespace {
constexpr double tau = 6.2831853071795864769;
}

bool Trajectory::valid() const {
  if (!std::isfinite(duration) || !std::isfinite(offset) || !std::isfinite(amplitude) ||
      !std::isfinite(start_hz) || !std::isfinite(end_hz) || !std::isfinite(step_at) ||
      duration <= 0 || !std::isfinite(std::abs(offset) + std::abs(amplitude))) {
    return false;
  }
  switch (shape) {
    case Shape::hold: return amplitude == 0;
    case Shape::step: return step_at >= 0 && step_at < duration;
    case Shape::sine: return start_hz > 0 && start_hz <= 10;
    case Shape::sweep: return start_hz > 0 && end_hz >= start_hz && end_hz <= 10;
  }
  return false;
}

double Trajectory::sample(double elapsed) const {
  const double t = std::clamp(elapsed, 0., duration);
  switch (shape) {
    case Shape::hold: return offset;
    case Shape::step: return offset + (t >= step_at ? amplitude : 0.);
    case Shape::sine: return offset + amplitude * std::sin(tau * start_hz * t);
    case Shape::sweep: {
      const double cycles = start_hz * t + (end_hz - start_hz) * t * t / (2 * duration);
      return offset + amplitude * std::sin(tau * cycles);
    }
  }
  return offset;
}

double Trajectory::periodic_slope_bound() const {
  if (shape != Shape::sine && shape != Shape::sweep) {
    return 0;
  }
  return tau * std::abs(amplitude) * (shape == Shape::sine ? start_hz : end_hz);
}

}  // namespace encos::experiments
