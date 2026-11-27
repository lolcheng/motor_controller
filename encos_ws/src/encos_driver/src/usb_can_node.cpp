#include "encos_core/serial_can_transport.hpp"
#include "encos_driver/ros_interfaces.hpp"
#include "encos_msgs/srv/discover_motor.hpp"
#include "encos_msgs/srv/read_parameter.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>

namespace encos::driver {
namespace {
using Discover = encos_msgs::srv::DiscoverMotor;
using Read = encos_msgs::srv::ReadParameter;
using namespace std::chrono_literals;

class UsbCanNode final : public rclcpp::Node {
 public:
  UsbCanNode() : Node("encos_usb_can") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    const auto device = declare_parameter<std::string>("device", "/dev/ttyUSB0", fixed);
    auto id = declare_parameter<int>("motor_id", 1, fixed);
    if (id < 1 || id >= int(management_id)) throw std::invalid_argument("invalid motor_id");
    id_ = static_cast<std::uint16_t>(id);
    const bool allow = declare_parameter<bool>("allow_queries", false, fixed);
    bus_ = std::make_unique<SerialCanTransport>(device, allow);
    raw_ = create_publisher<msg::RawFrame>("/encos/raw_rx", rclcpp::SensorDataQoS());
    state_ = create_publisher<msg::MotorState>("/encos/motor_state", rclcpp::SensorDataQoS());
    if (allow) {
      discover_ = create_service<Discover>("/encos/discover_motor", [this](
          const std::shared_ptr<Discover::Request>, std::shared_ptr<Discover::Response> reply) {
        const auto event = query(codec_.discover());
        populate(event, *reply);
        if (event.reply) {
          if (const auto* id_reply = std::get_if<ManagementReply>(&event.reply->feedback)) {
            reply->motor_id = id_reply->id.value_or(0);
          }
        }
      });
      read_ = create_service<Read>("/encos/read_parameter", [this](
          const std::shared_ptr<Read::Request> request, std::shared_ptr<Read::Response> reply) {
        try {
          const auto event = query(codec_.query(id_, static_cast<QueryCode>(request->code), request->group));
          populate(event, *reply);
          if (!reply->success || !event.reply) return;
          if (event.uuid) {
            reply->value.kind = msg::ParameterValue::UUID;
            reply->value.bytes = *event.uuid;
          } else if (const auto* group = std::get_if<KtReply>(&event.reply->feedback)) {
            reply->value.kind = msg::ParameterValue::KT_GROUP;
            reply->value.group = group->group.value_or(0);
            reply->value.kt_raw_header = group->raw_header;
            reply->value.values = group->values;
          } else if (const auto* value = std::get_if<QueryReply>(&event.reply->feedback)) {
            reply->value = typed_value(value->value);
            publish_measurement(*value, event.reply->raw);
          }
        } catch (const CodecError& error) {
          reply->detail = error.what();
        }
      });
    }
    timer_ = create_wall_timer(5ms, [this] {
      for (int index = 0; index < 64; ++index) {
        const auto frame = bus_->receive();
        if (!frame) break;
        publish(codec_.decode(*frame));
      }
      if (!bus_->connected()) {
        publish_state(nullptr, nullptr);
        timer_->cancel();
      }
    });
    RCLCPP_INFO(get_logger(), "USB-CAN on %s, %s", device.c_str(), allow ? "read-only queries enabled" : "passive monitor");
  }

 private:
  TransactionEvent query(const Encoded& frame) {
    return serial_query(*bus_, transactions_, codec_, frame, 500ms,
                        [this](const Decoded& decoded) { publish(decoded); });
  }

  template<class Reply>
  void populate(const TransactionEvent& event, Reply& reply) {
    using S = TransactionState;
    reply.success = event.state == S::completed;
    switch (event.state) {
      case S::completed: reply.detail = "matched"; break;
      case S::timed_out: reply.detail = "query timed out"; break;
      case S::rejected: reply.detail = "error response"; break;
      default: reply.detail = "query unavailable or reply signature quarantined"; break;
    }
    if (event.reply) {
      const auto& sample = event.reply->raw;
      reply.raw = raw_message(sample.frame, sample.stamp, &sample);
      reply.raw.header.stamp = now();
    }
  }

  void publish(const Decoded& decoded) {
    const auto& sample = decoded.raw;
    auto raw = raw_message(sample.frame, sample.stamp, &sample);
    raw.header.stamp = now();
    raw_->publish(raw);
    if (sample.frame.id != id_) return;
    if (const auto* telemetry = std::get_if<Telemetry>(&decoded.feedback)) publish_state(telemetry, &sample);
  }

  void publish_measurement(const QueryReply& query, const ReceivedFrame& sample) {
    const auto* value = std::get_if<Scalar>(&query.value);
    if (!value) return;
    Telemetry telemetry{};
    const Measurement measurement{value->value, MeasurementSource::float_feedback, sample.stamp};
    switch (query.code) {
      case QueryCode::position: telemetry.position_rad = measurement; break;
      case QueryCode::velocity: telemetry.velocity_rad_s = measurement; break;
      case QueryCode::phase_current: telemetry.phase_current_a = measurement; break;
      default: return;
    }
    publish_state(&telemetry, &sample);
  }

  void publish_state(const Telemetry* telemetry, const ReceivedFrame* sample) {
    auto state = state_message(id_, telemetry, sample);
    state.header.stamp = now();
    state.connected = bus_->connected();
    state.safety_state = state.connected ? msg::MotorState::READ_ONLY : msg::MotorState::DISCONNECTED;
    state_->publish(state);
  }

  std::uint16_t id_{};
  std::unique_ptr<SerialCanTransport> bus_;
  Protocol codec_;
  TransactionManager transactions_;
  rclcpp::Publisher<msg::RawFrame>::SharedPtr raw_;
  rclcpp::Publisher<msg::MotorState>::SharedPtr state_;
  rclcpp::Service<Discover>::SharedPtr discover_;
  rclcpp::Service<Read>::SharedPtr read_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace
}  // namespace encos::driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<encos::driver::UsbCanNode>());
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
