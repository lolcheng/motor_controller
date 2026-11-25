#include "encos_experiments/trajectory.hpp"
#include <cmath>
#include <limits>
using namespace encos::experiments;
int main() {
  const Trajectory step{Shape::step, 2, .1, .2, 0, 0, 1};
  const Trajectory wave{Shape::sine, 2, 0, .2, 1, 0, 0};
  const Trajectory sweep{Shape::sweep, 2, 0, .2, 1, 2, 0};
  if (!step.valid() || !wave.valid() || !sweep.valid() ||
      std::abs(step.sample(1.0) - .3) > 1e-9 || std::abs(step.sample(.9) - .1) > 1e-9 ||
      std::abs(wave.sample(.25) - .2) > 1e-9 || std::abs(sweep.sample(1) - .2) > 1e-9 ||
      sweep.periodic_slope_bound() <= wave.periodic_slope_bound()) return 1;
  auto invalid = sweep;
  invalid.end_hz = std::numeric_limits<double>::infinity();
  return invalid.valid() ? 2 : 0;
}
