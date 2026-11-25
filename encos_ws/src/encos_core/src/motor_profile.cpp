#include "encos_core/motor_profile.hpp"

#include <cmath>

namespace encos {

bool Range::valid() const {
  return std::isfinite(min) && std::isfinite(max) && min < max && std::isfinite(max - min);
}
bool Range::contains(double value) const {
  return valid() && std::isfinite(value) && value >= min && value <= max;
}
bool MixedRanges::valid() const {
  return kp.valid() && kd.valid() && kp.min >= 0 && kd.min >= 0 &&
         position_rad.valid() && velocity_rad_s.valid() && torque_nm.valid();
}
bool Feedback1Ranges::valid() const {
  return position_rad.valid() && velocity_rad_s.valid() && phase_current_a.valid();
}

}  // namespace encos
