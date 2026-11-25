#pragma once

#include "encos_core/protocol.hpp"

#include <vector>

namespace encos {

enum class Begin { accepted, busy, quarantined, wait_after_write, invalid_time, invalid_command };
enum class ReplyReuse { quarantine_all, single_reply_simulation };
enum class TransactionState { ignored, partial, duplicate, completed, rejected, timed_out, cancelled };

struct TransactionEvent {
  TransactionState state{TransactionState::ignored};
  std::optional<Decoded> reply;
  std::optional<std::array<std::uint8_t, 12>> uuid;
};

class TransactionManager {
 public:
  // Only supply a finite lifetime when the device and transport bound has been established.
  // With no bound, overlapping replies cannot be associated again in this bus epoch.
  explicit TransactionManager(std::optional<std::chrono::milliseconds> reply_lifetime = {},
                              ReplyReuse reuse = ReplyReuse::quarantine_all);
  Begin begin(const Encoded& command, TimePoint sent, TimePoint deadline);
  TransactionEvent receive(const Decoded& decoded, TimePoint now);
  TransactionEvent poll(TimePoint now);
  TransactionEvent cancel();
  bool pending() const { return pending_.has_value(); }

 private:
  struct Pending {
    Encoded command;
    TimePoint sent;
    TimePoint deadline;
    std::array<std::optional<std::array<std::uint8_t, 4>>, 3> uuid{};
  };
  struct Retired {
    ReplyKey key;
    std::optional<TimePoint> expires;
  };

  TransactionEvent finish(TransactionState result, std::optional<Decoded> decoded = {},
                          std::optional<std::array<std::uint8_t, 12>> uuid = {});
  std::optional<std::chrono::milliseconds> reply_lifetime_;
  ReplyReuse reuse_;
  std::optional<TimePoint> last_send_, next_send_;
  std::optional<Pending> pending_;
  std::vector<Retired> retired_;
};

}  // namespace encos
