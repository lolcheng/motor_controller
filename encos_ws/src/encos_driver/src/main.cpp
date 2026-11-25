#include "encos_driver/motor_node.hpp"
#include <iostream>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<encos::driver::MotorNode>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "encos_driver: " << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
