#pragma once

#include <optional>

namespace encos {

struct Range {
  double min{};
  double max{};
  bool valid() const;
  bool contains(double value) const;
};

struct MixedRanges {
  Range kp;
  Range kd;
  Range position_rad;
  Range velocity_rad_s;
  Range torque_nm;
  bool valid() const;
};

struct Feedback1Ranges {
  Range position_rad;
  Range velocity_rad_s;
  Range phase_current_a;
  bool valid() const;
};

struct ProtocolProfile {
  // Populate only after the ranges and their applicability have been verified.
  std::optional<MixedRanges> mixed;
  std::optional<Feedback1Ranges> feedback1;
  std::optional<double> kt_nm_per_a;
  // §10.5 query 35 has its own raw domain. Code 1 is "no compensation".
  std::optional<unsigned> query35_raw;
  // Query 39 specifies scale but does not explicitly name a physical unit.
  bool query39_degrees_confirmed{};
};

}  // namespace encos
