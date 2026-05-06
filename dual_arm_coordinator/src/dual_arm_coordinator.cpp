#include "dual_arm_coordinator/dual_arm_coordinator.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>

namespace dual_arm_coordinator {

namespace {

std::string joinedTopic(const std::string &ns, const std::string &leaf) {
  std::string out;
  out.reserve(ns.size() + leaf.size() + 2);
  if (ns.empty() || ns.front() != '/') {
    out.push_back('/');
  }
  out.append(ns);
  if (out.back() != '/') {
    out.push_back('/');
  }
  out.append(leaf);
  return out;
}

} // namespace

DualArmCoordinator::DualArmCoordinator(const rclcpp::NodeOptions &options)
    : rclcpp::Node("dual_arm_coordinator", options) {
  const auto arm_namespaces = declare_parameter<std::vector<std::string>>(
      "arms", std::vector<std::string>{});
  const auto combined_trajectory_topic = declare_parameter<std::string>(
      "combined_trajectory_topic", "/target_joint_trajectory");
  const auto combined_ack_topic =
      declare_parameter<std::string>("combined_ack_topic", "/trajectory_ack");
  const auto safe_modes = declare_parameter<std::vector<int64_t>>(
      "safe_robot_modes", std::vector<int64_t>{1, 2});
  const auto safety_tick_period_ms =
      declare_parameter<int>("safety_tick_period_ms", 50);

  if (arm_namespaces.size() < 1) {
    throw std::runtime_error(
        "dual_arm_coordinator: 'arms' parameter must list at least one arm "
        "namespace (typically two).");
  }

  for (auto m : safe_modes) {
    safe_robot_modes_.insert(static_cast<int32_t>(m));
  }

  arms_.resize(arm_namespaces.size());
  for (size_t i = 0; i < arm_namespaces.size(); ++i) {
    auto &arm = arms_[i];
    arm.ns = arm_namespaces[i];

    arm.traj_pub = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        joinedTopic(arm.ns, "target_joint_trajectory"), 1);

    arm.robot_mode_sub = create_subscription<std_msgs::msg::Int32>(
        joinedTopic(arm.ns, "robot_mode"), 10,
        [this, i](const std_msgs::msg::Int32::SharedPtr msg) {
          this->onArmRobotMode(i, msg);
        });

    arm.ack_sub = create_subscription<std_msgs::msg::Empty>(
        joinedTopic(arm.ns, "trajectory_ack"), 10,
        [this, i](const std_msgs::msg::Empty::SharedPtr msg) {
          this->onArmAck(i, msg);
        });
  }

  combined_ack_pub_ =
      create_publisher<std_msgs::msg::Empty>(combined_ack_topic, 1);

  combined_traj_sub_ =
      create_subscription<trajectory_msgs::msg::JointTrajectory>(
          combined_trajectory_topic, 1,
          std::bind(&DualArmCoordinator::onCombinedTrajectory, this,
                    std::placeholders::_1));

  safety_timer_ = create_wall_timer(
      std::chrono::milliseconds(safety_tick_period_ms),
      std::bind(&DualArmCoordinator::safetyTick, this));

  RCLCPP_INFO(get_logger(),
              "dual_arm_coordinator started with %zu arm(s); listening on '%s' "
              "→ per-arm '<arm>/target_joint_trajectory'; combined ack on '%s'.",
              arms_.size(), combined_trajectory_topic.c_str(),
              combined_ack_topic.c_str());
}

bool DualArmCoordinator::armSafe(const ArmState &arm) const {
  if (!arm.robot_mode_seen) {
    return false;
  }
  return safe_robot_modes_.count(arm.robot_mode) > 0;
}

bool DualArmCoordinator::allArmsSafe() const {
  return std::all_of(arms_.begin(), arms_.end(),
                     [this](const ArmState &a) { return armSafe(a); });
}

void DualArmCoordinator::publishFreezeToAll() {
  trajectory_msgs::msg::JointTrajectory empty;
  empty.header.stamp = now();
  for (auto &arm : arms_) {
    arm.traj_pub->publish(empty);
    arm.acked_current = false;
  }
}

void DualArmCoordinator::onCombinedTrajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  // Empty trajectory = explicit cancel from upstream. Forward to all arms so
  // they each freezeDesiredPoses() and revert to CARTESIAN. Controllers do not
  // ack empties, so we don't aggregate or emit a combined ack here.
  if (msg->points.empty()) {
    for (auto &arm : arms_) {
      arm.traj_pub->publish(*msg);
      arm.acked_current = false;
    }
    trajectory_in_flight_ = false;
    RCLCPP_INFO(get_logger(),
                "Forwarded empty trajectory to all arms (cancel/freeze).");
    return;
  }

  if (!allArmsSafe()) {
    std::string detail;
    for (const auto &arm : arms_) {
      detail += " " + arm.ns + "=" +
                (arm.robot_mode_seen ? std::to_string(arm.robot_mode)
                                     : std::string("?"));
    }
    RCLCPP_WARN(get_logger(),
                "Dropping combined trajectory: not all arms safe. modes:%s",
                detail.c_str());
    return;
  }

  for (auto &arm : arms_) {
    arm.acked_current = false;
  }
  // Back-to-back fan-out (loose message-arrival sync).
  for (auto &arm : arms_) {
    arm.traj_pub->publish(*msg);
  }
  trajectory_in_flight_ = true;
  RCLCPP_INFO(get_logger(),
              "Forwarded combined trajectory (%zu points) to %zu arm(s).",
              msg->points.size(), arms_.size());
}

void DualArmCoordinator::onArmRobotMode(
    size_t idx, const std_msgs::msg::Int32::SharedPtr msg) {
  if (idx >= arms_.size()) {
    return;
  }
  arms_[idx].robot_mode = msg->data;
  arms_[idx].robot_mode_seen = true;
}

void DualArmCoordinator::onArmAck(
    size_t idx, const std_msgs::msg::Empty::SharedPtr /*msg*/) {
  if (idx >= arms_.size()) {
    return;
  }
  if (!trajectory_in_flight_) {
    // Stray ack (e.g. from a freeze/empty) - ignore.
    return;
  }
  arms_[idx].acked_current = true;
  const bool all_acked =
      std::all_of(arms_.begin(), arms_.end(),
                  [](const ArmState &a) { return a.acked_current; });
  if (all_acked) {
    combined_ack_pub_->publish(std_msgs::msg::Empty());
    RCLCPP_INFO(get_logger(),
                "All %zu arm(s) acked; emitted combined trajectory_ack.",
                arms_.size());
  }
}

void DualArmCoordinator::safetyTick() {
  if (!trajectory_in_flight_) {
    return;
  }
  if (allArmsSafe()) {
    return;
  }
  RCLCPP_WARN(get_logger(),
              "Mid-execution safety cutoff: an arm left the safe set; "
              "publishing empty trajectory to all arms.");
  publishFreezeToAll();
  trajectory_in_flight_ = false;
}

} // namespace dual_arm_coordinator
