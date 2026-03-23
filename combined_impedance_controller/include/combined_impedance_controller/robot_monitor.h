#ifndef ROBOT_MONITOR_H_INCLUDED
#define ROBOT_MONITOR_H_INCLUDED

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

namespace combined_impedance_controller {

/// Robot-agnostic base class for monitoring robot state and managing the
/// controller state machine (RUNNING / STOPPED / WAITING).  Concrete
/// implementations supply the robot-specific readiness check, state-interface
/// requirements, and recovery logic.
class RobotMonitor {
public:
  enum class ControllerState { RUNNING, WAITING, STOPPED };

  enum class MimicRobotMode {
    UNKNOWN = 0u,
    IDLE = 1,
    MOVE = 2,
    USER_STOPPED = 3,
  };

  explicit RobotMonitor(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  virtual ~RobotMonitor() = default;

  /// One-time setup: creates the collision-heartbeat subscriber and robot-mode
  /// publisher, then calls onConfigure() for robot-specific setup.
  void configure(const std::string &controller_name);

  /// Call from on_activate to initialise heartbeat timing.
  void activate();

  /// Return the state-interface names this monitor needs the controller to
  /// claim.  The controller will later pass the corresponding values (in the
  /// same order) to updateState().
  virtual std::vector<std::string>
  requiredStateInterfaces(const std::string &tf_prefix) const = 0;

  /// Feed the latest state-interface values (same order as
  /// requiredStateInterfaces()).
  virtual void updateState(const std::vector<double> &state_values) = 0;

  /// Call once per update() cycle.  Checks the collision heartbeat, runs the
  /// controller state machine, and publishes the robot mode.
  /// Returns true when the controller should freeze desired poses.
  bool update();

  ControllerState controllerState() const { return controller_state_; }
  MimicRobotMode mimicRobotMode() const { return mimic_robot_mode_; }

protected:
  /// Robot-specific one-time setup (service clients, parameters, …).
  virtual void onConfigure() = 0;

  /// Return true when the robot is in a state that allows normal operation.
  virtual bool isReady() const = 0;

  /// Called every cycle by the state machine.
  /// Override to implement automatic recovery logic.
  virtual void onRecoveryTick() {}

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  ControllerState controller_state_;
  MimicRobotMode mimic_robot_mode_;

private:
  void heartbeatCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void updateCollisionHeartbeat();
  bool updateControllerState();

  // Collision heartbeat state
  std::atomic<bool> is_safe_{false};
  rclcpp::Time last_heartbeat_time_;
  std::mutex heartbeat_mutex_;
  std::atomic<bool> initial_heartbeat_received_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr robot_mode_publisher_;
};

} // namespace combined_impedance_controller

#endif
