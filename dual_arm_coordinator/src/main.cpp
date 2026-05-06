#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "dual_arm_coordinator/dual_arm_coordinator.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node =
      std::make_shared<dual_arm_coordinator::DualArmCoordinator>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
