#pragma once

#include "encos_core/feedback.hpp"
#include "encos_core/motor_profile.hpp"

namespace encos {

enum class ReplyKind { none, telemetry, query, config, kt_group, brake, discover, zero, set_id, reset_id };

struct ReplyKey {
  ReplyKind kind{ReplyKind::none};
  std::uint16_t id{};
  std::uint8_t code{};
  std::uint16_t motor{};
  std::uint16_t new_id{};
};

struct Encoded {
  CanFrame frame;
  ReplyKey reply;
  std::chrono::milliseconds next_send_delay{};
};

class Protocol {
 public:
  explicit Protocol(WireFormat format = WireFormat::classic) : format_(format) {}

  Encoded control(std::uint16_t id, const Control& command, const ProtocolProfile& profile = {}) const;
  Encoded query(std::uint16_t id, QueryCode code, std::uint8_t group = 0) const;
  Encoded configure(std::uint16_t id, ConfigCode code, const ConfigValue& value,
                    WritePermission permission = WritePermission::denied) const;
  Encoded discover() const;
  Encoded zero(std::uint16_t id, std::optional<double> offset_rad = {},
               WritePermission permission = WritePermission::denied) const;
  Encoded set_id(std::uint16_t id, std::uint16_t new_id,
                 WritePermission permission = WritePermission::denied) const;
  Encoded reset_id(WritePermission permission = WritePermission::denied) const;
  Encoded brake(std::uint16_t id, BrakeState state,
                WritePermission permission = WritePermission::denied) const;
  Decoded decode(const ReceivedFrame& sample, const ProtocolProfile& profile = {}) const;

 private:
  CanFrame frame(std::uint16_t id, std::uint8_t length) const;
  WireFormat format_;
};

}  // namespace encos
