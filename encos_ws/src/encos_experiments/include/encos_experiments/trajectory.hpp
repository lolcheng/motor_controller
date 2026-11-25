#pragma once

namespace encos::experiments {

enum class Shape { hold, step, sine, sweep };

struct Trajectory {
  Shape shape;
  double duration, offset, amplitude, start_hz, end_hz, step_at;
  bool valid() const;
  double sample(double elapsed) const;
  double periodic_slope_bound() const;
};

}  // namespace encos::experiments
