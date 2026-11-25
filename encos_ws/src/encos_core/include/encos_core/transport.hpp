#pragma once

#include "encos_core/can_frame.hpp"

#include <optional>

namespace encos {

enum class SendStatus { accepted, disconnected, full, invalid_frame, io_error };

class Transport {
 public:
  virtual ~Transport() = default;
  virtual SendStatus send(const CanFrame& frame) = 0;
  virtual std::optional<ReceivedFrame> receive() = 0;
  virtual bool connected() const = 0;
};

}  // namespace encos
