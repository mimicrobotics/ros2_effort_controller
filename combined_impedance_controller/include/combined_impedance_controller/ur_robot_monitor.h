#ifndef UR_ROBOT_MONITOR_H_INCLUDED
#define UR_ROBOT_MONITOR_H_INCLUDED

#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_dashboard_msgs/srv/get_loaded_program.hpp>
#include <ur_dashboard_msgs/srv/load.hpp>

#include "combined_impedance_controller/robot_monitor.h"

namespace combined_impedance_controller {

class UrRobotMonitor : public RobotMonitor {
public:
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

  explicit UrRobotMonitor(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  void onConfigure() override;

  void updateState(const std::vector<double> &state_values) override;

  std::vector<std::string>
  requiredStateInterfaces(const std::string &tf_prefix) const override;

  RobotMode robotMode() const { return robot_mode_; }
  SafetyMode safetyMode() const { return safety_mode_; }
  ProgramMode programMode() const { return program_mode_; }

protected:
  bool isReady() const override;
  void onRecoveryTick() override;

private:
  RobotMode robot_mode_{RobotMode::DISCONNECTED};
  SafetyMode safety_mode_{SafetyMode::UNDEFINED_SAFETY_MODE};
  ProgramMode program_mode_{ProgramMode::STOPPED};

  void tryRestartExternalProgram();
  void tryRecoverFromStop();
  void validateRunningProgram();

  // Async Trigger helper: fires an async call and transitions recovery_state_
  // to next_state on success, or resets to IDLE on failure.
  using TriggerClient = rclcpp::Client<std_srvs::srv::Trigger>;
  void asyncTrigger(TriggerClient::SharedPtr &client, const char *description);

  // Stop recovery state machine
  enum class RecoveryState {
    INIT,
    IDLE,
    UNLOCKING_PROTECTIVE_STOP,
    RESTARTING_SAFETY,
    POWERING_ON,
    RELEASING_BRAKES,
  };
  static const char *toString(RecoveryState state);
  RecoveryState recovery_state_{RecoveryState::IDLE};
  RecoveryState recovery_next_state_{RecoveryState::IDLE};
  SafetyMode recovery_trigger_mode_{SafetyMode::NORMAL};
  bool recovery_call_in_flight_{false};
  rclcpp::Time last_recovery_attempt_;
  static constexpr double kRecoveryCooldown{3.0};

  TriggerClient::SharedPtr close_safety_popup_client_;
  TriggerClient::SharedPtr unlock_protective_stop_client_;
  TriggerClient::SharedPtr restart_safety_client_;
  TriggerClient::SharedPtr power_on_client_;
  TriggerClient::SharedPtr brake_release_client_;

  // External program auto-restart
  enum class ProgramRestartState {
    IDLE,
    LOADING,
    WAITING_FOR_PLAY,
    PLAYING,
  };
  ProgramRestartState program_restart_state_{ProgramRestartState::IDLE};
  rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedPtr load_program_client_;
  TriggerClient::SharedPtr play_client_;
  rclcpp::Time last_program_restart_attempt_;
  static constexpr double kProgramRestartCooldown{3.0};
  std::string ur_program_name_;
  std::string dashboard_prefix_;

  // Running program validation
  enum class ProgramValidationState {
    IDLE,
    CHECKING,
    STOPPING,
  };
  ProgramValidationState program_validation_state_{
      ProgramValidationState::IDLE};
  bool program_validated_{false};
  rclcpp::Client<ur_dashboard_msgs::srv::GetLoadedProgram>::SharedPtr
      get_loaded_program_client_;
  TriggerClient::SharedPtr stop_program_client_;
};

} // namespace combined_impedance_controller

#endif
