#ifndef DUAL_ARM_COORDINATOR_H_INCLUDED
#define DUAL_ARM_COORDINATOR_H_INCLUDED

#include <atomic>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace dual_arm_coordinator {

class DualArmCoordinator : public rclcpp::Node {
public:
  explicit DualArmCoordinator(const rclcpp::NodeOptions &options);

private:
  struct ArmState {
    std::string ns;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_mode_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ack_sub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
        traj_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_freeze_pub;
    int32_t robot_mode{0};
    bool robot_mode_seen{false};
    bool acked_current{false};
  };

  void onCombinedTrajectory(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void onArmRobotMode(size_t idx, const std_msgs::msg::Int32::SharedPtr msg);
  void onArmAck(size_t idx, const std_msgs::msg::Empty::SharedPtr msg);

  bool armSafe(const ArmState &arm) const;
  bool allArmsSafe() const;
  void publishSafetyFreeze(bool freeze);

  std::vector<ArmState> arms_;
  std::unordered_set<int32_t> safe_robot_modes_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      combined_traj_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr combined_ack_pub_;

  rclcpp::TimerBase::SharedPtr safety_timer_;
  void safetyTick();

  bool trajectory_in_flight_{false};
  bool safety_frozen_{false};
};

} // namespace dual_arm_coordinator

#endif
