#include "encos_tools/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>

namespace {
using namespace encos;
using namespace encos::tools;
using namespace std::chrono_literals;

volatile std::sig_atomic_t running = 1;
void halt(int) { running = 0; }

struct Sample {
  TimePoint scheduled{}, generated{}, finished{};
  CanFrame tx;
  Exchange exchange;
  unsigned skipped_after{};
};

double percentile(const std::vector<double>& sorted, double proportion) {
  if (sorted.empty()) return 0;
  const auto rank = static_cast<std::size_t>(std::ceil(sorted.size() * proportion));
  return sorted[rank - 1];
}

void write_csv(std::ostream& out, const std::vector<Sample>& samples) {
  out << "scheduled_mono_ns,generated_mono_ns,tx_call_before_mono_ns,tx_call_after_mono_ns,"
         "rx_host_mono_ns,rx_kernel_realtime_ns,rx_userspace_mono_ns,rx_origin,finish_mono_ns,tx_hex,rx_hex,tx_brs,rx_brs,"
         "rx_observed,rxq_overflow_total,begin_status,send_status,transaction,status_cycle_slots_skipped\n";
  for (const auto& row : samples) {
    const auto& e = row.exchange;
    const auto& r = e.last_rx;
    out << ns(row.scheduled) << ',' << ns(row.generated) << ',' << ns(e.tx_before) << ',' << ns(e.tx_after) << ',';
    if (r) out << ns(r->stamp);
    out << ',';
    if (r && r->kernel_realtime) out << r->kernel_realtime->count();
    out << ',';
    if (r && r->userspace_stamp) out << ns(*r->userspace_stamp);
    out << ',';
    if (r) out << static_cast<unsigned>(r->origin);
    out << ',' << ns(row.finished) << ',' << hex(row.tx) << ',';
    if (r) out << hex(r->frame);
    out << ',' << row.tx.brs << ',';
    if (r) out << r->frame.brs;
    out << ',' << e.observed << ',';
    if (e.socket_drops) out << *e.socket_drops;
    out << ',' << static_cast<unsigned>(e.admitted) << ',' << static_cast<unsigned>(e.sent)
        << ',' << static_cast<unsigned>(e.event.state) << ',' << row.skipped_after << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) try {
  if (argc != 6) {
    std::cout << "Usage: encos_latency_benchmark vcan0 classic|fd HZ SECONDS CSV\n";
    return argc == 2 && std::string_view(argv[1]) == "--help" ? 0 : 2;
  }
  const auto format = parse_format(argv[2]);
  const auto hz = parse_unsigned(argv[3], 1000);
  const auto seconds = parse_unsigned(argv[4], 600);
  if (!hz || !seconds || hz * seconds > 250000) throw std::invalid_argument("invalid or excessive benchmark size");
  SocketCanTransport bus({argv[1], format, {1}, true});
  const Protocol codec(format);
  TransactionManager transactions({}, ReplyReuse::single_reply_simulation);
  const auto period = std::chrono::nanoseconds(1'000'000'000 / hz);
  std::vector<Sample> data;
  data.reserve(hz * seconds);
  std::signal(SIGINT, halt);
  std::signal(SIGTERM, halt);
  std::ofstream csv(argv[5]);
  if (!csv) throw std::runtime_error("cannot open benchmark CSV");
  const auto start_cpu = std::clock();
  const auto start = Clock::now() + 10ms;
  const auto end = start + std::chrono::seconds(seconds);
  unsigned plan_index = 0;
  const unsigned planned = hz * seconds;
  std::string_view final_status = "complete";
  while (running && plan_index < planned) {
    const auto slot = start + period * plan_index;
    std::this_thread::sleep_until(slot);
    if (!running || Clock::now() >= end) break;
    const auto generated = Clock::now();
    const double target = plan_index % 2 ? -0.05 : 0.05;
    const auto packet = codec.control(1, Position{target, 0.5, 1, Ack::type2});
    auto result = exchange(bus, transactions, codec, packet, {}, 20ms, ignore_sample);
    if (result.event.state == TransactionState::completed) {
      const auto& feedback = std::get<Telemetry>(result.event.reply->feedback);
      if (!feedback.position_rad || std::abs(feedback.position_rad->value - target) > 1.e-6) {
        result.event.state = TransactionState::rejected;
      }
    }
    const auto finish = Clock::now();
    unsigned skipped = 0;
    while (plan_index + skipped + 1 < planned && start + period * (plan_index + skipped + 1) < finish) ++skipped;
    data.push_back({slot, generated, finish, packet.frame, std::move(result), skipped});
    plan_index += 1 + skipped;
    if (data.back().exchange.event.state != TransactionState::completed) {
      final_status = "latched_failure";
      break;  // Never reuse this unsequenced reply signature after a failed exchange.
    }
  }
  if (!running) final_status = "interrupted";
  const auto measured_end = Clock::now();
  const auto cpu_seconds = static_cast<double>(std::clock() - start_cpu) / CLOCKS_PER_SEC;
  const double wall = std::chrono::duration<double>(measured_end - start).count();
  write_csv(csv, data);
  csv.close();
  if (!csv) throw std::runtime_error("benchmark CSV write failed");
  std::vector<double> latency, jitter;
  std::size_t rx{}, tx{}, valid{}, timed_out{}, overruns{}, skipped{};
  std::uint32_t socket_drops{};
  std::optional<TimePoint> previous_tx;
  for (const auto& row : data) {
    const auto& e = row.exchange;
    rx += e.observed;
    overruns += row.finished > row.scheduled + period;
    skipped += row.skipped_after;
    if (e.socket_drops) socket_drops = *e.socket_drops;
    if (e.sent == SendStatus::accepted) {
      ++tx;
      if (previous_tx) {
        jitter.push_back(std::abs(std::chrono::duration<double, std::micro>(e.tx_before - *previous_tx - period).count()));
      }
      previous_tx = e.tx_before;
    }
    timed_out += e.event.state == TransactionState::timed_out;
    if (e.event.state == TransactionState::completed && e.event.reply) {
      ++valid;
      latency.push_back(std::chrono::duration<double, std::micro>(e.event.reply->raw.stamp - e.tx_before).count());
    }
  }
  std::sort(latency.begin(), latency.end());
  std::sort(jitter.begin(), jitter.end());
  std::cout << std::fixed << std::setprecision(3)
      << "format=" << argv[2] << " target_hz=" << hz << " wall_s=" << wall << " planned=" << planned
      << " tx=" << tx << " rx=" << rx << " valid=" << valid << " tx_hz=" << (wall > 0 ? tx / wall : 0)
      << " feedback_hz=" << (wall > 0 ? valid / wall : 0) << " loss_pct=" << (tx ? (tx - valid) * 100.0 / tx : 0)
      << " timeouts=" << timed_out << " rxq_drops=" << socket_drops << " cycle_overruns=" << overruns
      << " skipped_slots=" << skipped << " cpu_percent=" << (wall > 0 ? 100 * cpu_seconds / wall : 0)
      << " rtt_p50_us=" << percentile(latency, 0.50) << " rtt_p95_us=" << percentile(latency, 0.95)
      << " rtt_p99_us=" << percentile(latency, 0.99) << " rtt_max_us=" << percentile(latency, 1)
      << " jitter_p50_us=" << percentile(jitter, 0.50) << " jitter_p95_us=" << percentile(jitter, 0.95)
      << " jitter_p99_us=" << percentile(jitter, 0.99) << " jitter_max_us=" << percentile(jitter, 1)
      << " status=" << final_status << '\n';
  return final_status == "complete" && valid != 0 ? 0 : 1;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n';
  return 2;
}
