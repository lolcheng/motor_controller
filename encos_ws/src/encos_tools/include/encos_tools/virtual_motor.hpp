#pragma once

#include "encos_core/protocol.hpp"

#include <deque>
#include <vector>

namespace encos::tools {

struct ModelOptions {
  std::uint16_t id{1};
  WireFormat format{WireFormat::classic};
  MotorError error{MotorError::none};
  double motor_c{25};
  bool reject_writes{};
};

struct MotorSnapshot {
  std::uint16_t id{};
  double position_rad{}, velocity_rad_s{}, phase_current_a{};
  double motor_c{}, mos_c{};
  MotorError error{};
  BrakeState brake{BrakeState::released};
  bool active{};
};

class VirtualMotor {
 public:
  explicit VirtualMotor(ModelOptions options = {});
  std::vector<CanFrame> handle(const ReceivedFrame& request, TimePoint now);
  void advance(TimePoint now);
  CanFrame idle_telemetry() const;
  const MotorSnapshot& state() const { return state_; }

 private:
  std::vector<CanFrame> management(const CanFrame& request, TimePoint now);
  std::vector<CanFrame> control(const CanFrame& request, TimePoint now);
  std::vector<CanFrame> configure(const CanFrame& request, TimePoint now);
  std::vector<CanFrame> query(const CanFrame& request) const;
  CanFrame make(std::initializer_list<std::uint8_t> bytes, std::uint16_t id) const;
  CanFrame telemetry(std::uint8_t type) const;
  CanFrame segment(std::uint8_t group) const;
  void set_position(double value);
  void stop_motion() noexcept;
  void refresh_ranges();

  ModelOptions options_;
  MotorSnapshot state_;
  ProtocolProfile profile_;
  std::array<std::array<std::uint8_t, 4>, 18> settings_{};
  std::array<std::array<std::uint8_t, 6>, 10> kt_{};
  std::optional<std::uint8_t> calibration_written_;
  std::optional<WireFormat> staged_format_;
  std::optional<TimePoint> last_tick_, last_motion_, quiet_until_;
  std::int16_t zero_offset_count_{};
  bool encoder_calibrated_{true};
};

struct FaultOptions {
  bool silent{}, duplicate{}, wrong_id{}, short_length{}, unknown_type{}, uuid_missing{}, uuid_reverse{};
  unsigned drop_every{};
  std::chrono::milliseconds delay{};
};

class FaultQueue {
 public:
  explicit FaultQueue(FaultOptions options = {}) : options_(options) {}
  void enqueue(std::vector<CanFrame> frames, TimePoint now);
  std::optional<CanFrame> take(TimePoint now);
  std::optional<TimePoint> next() const;
  std::size_t dropped() const { return dropped_; }

 private:
  struct Delivery { TimePoint due; CanFrame frame; };
  FaultOptions options_;
  std::deque<Delivery> queue_;
  std::size_t request_count_{}, dropped_{};
};

}  // namespace encos::tools
