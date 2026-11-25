#include "encos_msgs/msg/motor_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using Command = encos_msgs::msg::MotorCommand;

std::int64_t ns(Clock::time_point time) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}
struct Published { std::int64_t release, mono, ros; };
}  // namespace

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: encos_topic_source OWNER HZ SECONDS CSV (isolated vcan setup)\n";
    return 2;
  }
  try {
    const auto owner = std::stoull(argv[1]);
    const auto hz = std::stoul(argv[2]);
    const auto seconds = std::stod(argv[3]);
    if (owner == 0 || hz < 10 || hz > 1000 || !std::isfinite(seconds) || seconds <= 0 || seconds > 90) {
      throw std::invalid_argument("invalid source arguments");
    }
    std::ofstream csv(argv[4]);
    if (!csv) throw std::runtime_error("cannot open source timing CSV");

    rclcpp::init(1, argv);
    auto node = std::make_shared<rclcpp::Node>("encos_topic_source");
    auto publisher = node->create_publisher<Command>("/encos/motor_command", rclcpp::QoS(1).best_effort());
    const auto match_limit = Clock::now() + 4s;
    while (rclcpp::ok() && publisher->get_subscription_count() == 0 && Clock::now() < match_limit) {
      std::this_thread::sleep_for(5ms);
    }
    if (publisher->get_subscription_count() != 1) {
      throw std::runtime_error("exactly one vcan driver subscription is required");
    }
    std::vector<Published> trace;
    trace.reserve(static_cast<std::size_t>(hz * seconds) + 1);
    const auto period = std::chrono::nanoseconds(1'000'000'000 / hz);
    const auto start = Clock::now() + 10ms;
    const auto end = start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    Command command;
    command.motor_id = 1;
    command.owner = owner;
    command.mode = Command::POSITION;
    command.feedback = 1;
    command.position_rad = .05;
    command.speed_limit_rad_s = .5;
    command.current_limit_a = 1.;
    command.valid_for.nanosec = 150000000;
    std::uint64_t skipped = 0;
    for (auto release = start; rclcpp::ok() && release < end; release += period) {
      std::this_thread::sleep_until(release);
      const auto stamp = Clock::now();
      if (stamp >= end || !rclcpp::ok()) break;
      const auto behind = (stamp - release) / period;
      skipped += static_cast<std::uint64_t>(behind);
      release += period * behind;
      const auto ros = node->now();
      command.header.stamp = ros;
      publisher->publish(command);
      trace.push_back({ns(release), ns(stamp), ros.nanoseconds()});
    }
    csv << "scheduled_mono_ns,publish_mono_ns,source_ros_ns\n";
    for (const auto& sample : trace) csv << sample.release << ',' << sample.mono << ',' << sample.ros << '\n';
    csv.close();
    if (!csv) throw std::runtime_error("source timing CSV write failed");
    std::cout << "published=" << trace.size() << " skipped=" << skipped << '\n';
    rclcpp::shutdown();
    return trace.empty() ? 1 : 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
