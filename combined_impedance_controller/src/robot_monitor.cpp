#include <combined_impedance_controller/robot_monitor.h>

namespace combined_impedance_controller {

RobotMonitor::RobotMonitor(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : node_(std::move(node)), controller_state_(ControllerState::RUNNING),
      mimic_robot_mode_(MimicRobotMode::MOVE) {}

void RobotMonitor::configure(const std::string &controller_name) {
  // Collision-detection heartbeat subscriber
  heartbeat_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
      "collision_detection_heartbeat", 1,
      std::bind(&RobotMonitor::heartbeatCallback, this, std::placeholders::_1));

  // Robot mode publisher
  robot_mode_publisher_ = node_->create_publisher<std_msgs::msg::Int32>(
      controller_name + "/robot_mode", 1);

  // Robot-specific setup
  onConfigure();
}

void RobotMonitor::activate() {
  std::lock_guard<std::mutex> lock(heartbeat_mutex_);
  last_heartbeat_time_ = node_->get_clock()->now();
}

bool RobotMonitor::update() {
  updateCollisionHeartbeat();
  const bool should_freeze = updateControllerState();

  // Publish mimic robot mode for high-level components
  std_msgs::msg::Int32 mode_msg;
  mode_msg.data = static_cast<int>(mimic_robot_mode_);
  robot_mode_publisher_->publish(mode_msg);

  return should_freeze;
}

bool RobotMonitor::updateControllerState() {
  const bool is_safe = is_safe_.load();
  const bool ready = is_safe && isReady();

  if (controller_state_ == ControllerState::RUNNING && !ready) {
    controller_state_ = ControllerState::STOPPED;
    if (!is_safe) {
      RCLCPP_INFO(
          node_->get_logger(),
          "Collision detected! Freezing current pose. Recycle e-stops and move "
          "arms into a non collision config to continue operation.");
    } else {
      RCLCPP_INFO(node_->get_logger(),
                  "Robot not ready — freezing current pose.");
    }
  }

  if (!ready) {
    mimic_robot_mode_ = MimicRobotMode::USER_STOPPED;
  }

  // If collision had occurred, we now enter a pending state to wait for
  // recovery to finish.
  if (is_safe && controller_state_ == ControllerState::STOPPED) {
    controller_state_ = ControllerState::WAITING;
  }

  if (controller_state_ == ControllerState::WAITING && ready) {
    RCLCPP_INFO(node_->get_logger(),
                "Robot back in safe remote control state. Resuming...");
    controller_state_ = ControllerState::RUNNING;
    mimic_robot_mode_ = MimicRobotMode::MOVE;
  }

  // Let the concrete implementation attempt recovery
  onRecoveryTick();

  // Return true when controller should freeze desired poses
  return !ready;
}

void RobotMonitor::updateCollisionHeartbeat() {
  rclcpp::Time current_last_heartbeat_time;
  bool initial_heartbeat_was_received = false;
  const auto time = node_->get_clock()->now();

  {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    current_last_heartbeat_time = last_heartbeat_time_;
    initial_heartbeat_was_received = initial_heartbeat_received_.load();
  }

  if (initial_heartbeat_was_received) {
    double time_diff = (time - current_last_heartbeat_time).seconds();
    if (time_diff > 0.5 && is_safe_.load()) {
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Heartbeat timed out. Setting controller to UNSAFE. Current time: "
          "%f, Last heartbeat: %f",
          time.seconds(), current_last_heartbeat_time.seconds());
      is_safe_.store(false);
      initial_heartbeat_received_.store(false);
    }
  }
}

void RobotMonitor::heartbeatCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  bool is_now_safe = msg->data;

  {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    last_heartbeat_time_ = node_->get_clock()->now();

    if (!initial_heartbeat_received_.load()) {
      initial_heartbeat_received_.store(true);
      RCLCPP_INFO(node_->get_logger(),
                  "Initial collision detection heartbeat received. "
                  "Controller operational.");
    }
  }

  bool was_safe = is_safe_.exchange(is_now_safe);

  if (is_now_safe != was_safe) {
    if (is_now_safe) {
      RCLCPP_INFO(node_->get_logger(),
                  "Controller state changed to SAFE (no collision).");
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Controller state changed to UNSAFE (collision detected).");
    }
  }
}

} // namespace combined_impedance_controller
