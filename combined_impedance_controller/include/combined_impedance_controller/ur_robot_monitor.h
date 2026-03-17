#ifndef UR_ROBOT_MONITOR_H_INCLUDED
#define UR_ROBOT_MONITOR_H_INCLUDED

#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_dashboard_msgs/srv/load.hpp>

namespace combined_impedance_controller {

class UrRobotMonitor {
public:
  enum class ControllerState { RUNNING, WAITING, STOPPED };

  enum class MimicRobotMode {
    UNKNOWN = 0u,
    IDLE = 1,
    MOVE = 2,
    USER_STOPPED = 3,
  };

  explicit UrRobotMonitor(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /// Read parameters and create service clients. Call from on_configure().
  void configure();

  /// Feed raw state_interface values. Call once per update() cycle.
  void updateState(double robot_mode_val, double safety_mode_val,
                   double program_running_val);

  /// Run the controller state machine. Call once per update() cycle.
  /// Returns true when the controller should freeze desired poses.
  bool updateControllerState(bool is_safe);

  /// True when robot is RUNNING, safety is NORMAL, program is PLAYING.
  bool isReady() const;

  ControllerState controllerState() const { return controller_state_; }
  MimicRobotMode mimicRobotMode() const { return mimic_robot_mode_; }
  RobotMode robotMode() const { return robot_mode_; }
  SafetyMode safetyMode() const { return safety_mode_; }
  ProgramMode programMode() const { return program_mode_; }

  /// State interface names the controller must claim.
  std::vector<std::string>
  requiredStateInterfaces(const std::string &tf_prefix) const;

private:
  enum class RobotMode {
    NO_CONTROLLER = -1,
    DISCONNECTED = 0,
    CONFIRM_SAFETY = 1,
    BOOTING = 2,
    POWER_OFF = 3,
    POWER_ON = 4,
    IDLE = 5,
    BACKDRIVE = 6,
    RUNNING = 7,
    UPDATING_FIRMWARE = 8,
  };

  enum class SafetyMode {
    NORMAL = 1u,
    REDUCED = 2,
    PROTECTIVE_STOP = 3,
    RECOVERY = 4,
    SAFEGUARD_STOP = 5,
    SYSTEM_EMERGENCY_STOP = 6,
    ROBOT_EMERGENCY_STOP = 7,
    VIOLATION = 8,
    FAULT = 9,
    VALIDATE_JOINT_ID = 10,
    UNDEFINED_SAFETY_MODE = 11,
    AUTOMATIC_MODE_SAFEGUARD_STOP = 12,
    SYSTEM_THREE_POSITION_ENABLING_STOP = 13,
  };

  enum class ProgramMode {
    STOPPED = 0u,
    PLAYING = 1,
    PAUSED = 2,
  };

  static const char *toString(RobotMode mode);
  static const char *toString(SafetyMode mode);
  static const char *toString(ProgramMode mode);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  RobotMode robot_mode_{RobotMode::DISCONNECTED};
  SafetyMode safety_mode_{SafetyMode::UNDEFINED_SAFETY_MODE};
  ProgramMode program_mode_{ProgramMode::STOPPED};

  // Controller state machine
  ControllerState controller_state_{ControllerState::STOPPED};
  MimicRobotMode mimic_robot_mode_{MimicRobotMode::UNKNOWN};
  void tryRestartExternalProgram();

  // External program auto-restart
  enum class ProgramRestartState {
    IDLE,
    LOADING,
    WAITING_FOR_PLAY,
    PLAYING,
  };
  ProgramRestartState program_restart_state_{ProgramRestartState::IDLE};
  rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedPtr load_program_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr play_client_;
  rclcpp::Time last_program_restart_attempt_;
  static constexpr double kProgramRestartCooldown{3.0};
  std::string ur_program_name_;
  std::string dashboard_prefix_;
};

} // namespace combined_impedance_controller

#endif
