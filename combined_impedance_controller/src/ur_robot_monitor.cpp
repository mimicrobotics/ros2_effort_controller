#include <combined_impedance_controller/ur_robot_monitor.h>

namespace combined_impedance_controller {

UrRobotMonitor::UrRobotMonitor(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : RobotMonitor(std::move(node)) {}

void UrRobotMonitor::onConfigure() {
  ur_program_name_ = node_->get_parameter("ur_program_name").as_string();
  dashboard_prefix_ = node_->get_parameter("dashboard_prefix").as_string();
  load_program_client_ = node_->create_client<ur_dashboard_msgs::srv::Load>(
      dashboard_prefix_ + "/load_program");
  play_client_ =
      node_->create_client<std_srvs::srv::Trigger>(dashboard_prefix_ + "/play");
  last_program_restart_attempt_ = node_->get_clock()->now();
  last_recovery_attempt_ = node_->get_clock()->now();

  // Recovery service clients
  close_safety_popup_client_ = node_->create_client<std_srvs::srv::Trigger>(
      dashboard_prefix_ + "/close_safety_popup");
  unlock_protective_stop_client_ = node_->create_client<std_srvs::srv::Trigger>(
      dashboard_prefix_ + "/unlock_protective_stop");
  restart_safety_client_ = node_->create_client<std_srvs::srv::Trigger>(
      dashboard_prefix_ + "/restart_safety");
  power_on_client_ = node_->create_client<std_srvs::srv::Trigger>(
      dashboard_prefix_ + "/power_on");
  brake_release_client_ = node_->create_client<std_srvs::srv::Trigger>(
      dashboard_prefix_ + "/brake_release");
}

void UrRobotMonitor::updateState(const std::vector<double> &state_values) {
  // Indices correspond to requiredStateInterfaces() order:
  //   [0] robot_mode, [1] safety_mode, [2] program_running
  const auto robot_mode_new = static_cast<RobotMode>(state_values[0]);
  if (robot_mode_new != robot_mode_) {
    robot_mode_ = robot_mode_new;
    RCLCPP_INFO(node_->get_logger(), "Robot mode switched to: %s",
                toString(robot_mode_));
  }
  const auto safety_mode_new = static_cast<SafetyMode>(state_values[1]);
  if (safety_mode_new != safety_mode_) {
    safety_mode_ = safety_mode_new;
    RCLCPP_INFO(node_->get_logger(), "Safety mode switched to: %s",
                toString(safety_mode_));
  }
  const auto program_mode_new = static_cast<ProgramMode>(state_values[2]);
  if (program_mode_new != program_mode_) {
    program_mode_ = program_mode_new;
    RCLCPP_INFO(node_->get_logger(), "Program mode switched to: %s",
                toString(program_mode_));
  }
}

bool UrRobotMonitor::isReady() const {
  return robot_mode_ == RobotMode::RUNNING &&
         safety_mode_ == SafetyMode::NORMAL &&
         program_mode_ == ProgramMode::PLAYING;
}

void UrRobotMonitor::onRecoveryTick() {
  tryRecoverFromStop();
  tryRestartExternalProgram();
}

std::vector<std::string>
UrRobotMonitor::requiredStateInterfaces(const std::string &tf_prefix) const {
  return {
      tf_prefix + "gpio/robot_mode",
      tf_prefix + "gpio/safety_mode",
      tf_prefix + "gpio/program_running",
  };
}

void UrRobotMonitor::asyncTrigger(TriggerClient::SharedPtr &client,
                                  const char *description) {
  if (!client->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "%s service not available, waiting...", description);
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Recovery: calling %s...", description);
  recovery_call_in_flight_ = true;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(request, [this, description](
                                          TriggerClient::SharedFuture future) {
    auto result = future.get();
    recovery_call_in_flight_ = false;
    if (result->success) {
      RCLCPP_INFO(node_->get_logger(), "Recovery: %s succeeded.", description);
      recovery_state_ = recovery_next_state_;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Recovery: %s failed: %s", description,
                  result->message.c_str());
      recovery_state_ = RecoveryState::IDLE;
      last_recovery_attempt_ = node_->get_clock()->now();
    }
  });
}

void UrRobotMonitor::tryRecoverFromStop() {
  // Nothing to do when safety is normal
  if (safety_mode_ == SafetyMode::NORMAL && robot_mode_ == RobotMode::RUNNING) {
    if (recovery_state_ != RecoveryState::IDLE) {
      RCLCPP_INFO(node_->get_logger(), "Recovery: safety mode is normal.");
      recovery_state_ = RecoveryState::IDLE;
    }
    return;
  }

  // Don't start a new recovery during transient states (RECOVERY, BOOTING)
  if (safety_mode_ == SafetyMode::RECOVERY ||
      robot_mode_ == RobotMode::BOOTING) {
    recovery_state_ = RecoveryState::INIT;
    return;
  }

  // Don't start a new recovery while robot or system is e-stopped
  if (safety_mode_ == SafetyMode::SYSTEM_EMERGENCY_STOP ||
      safety_mode_ == SafetyMode::ROBOT_EMERGENCY_STOP) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Waiting for E-Stop release...");
    return;
  }

  // Wait for in-flight service calls to complete
  if (recovery_call_in_flight_) {
    return;
  }

  const auto now = node_->get_clock()->now();

  // Determine which recovery sequence to run based on the triggering safety
  // mode. Only start a new sequence from IDLE.
  if (recovery_state_ == RecoveryState::IDLE) {
    if ((now - last_recovery_attempt_).seconds() < kRecoveryCooldown) {
      return;
    }
    recovery_trigger_mode_ = safety_mode_;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Recovery: starting recovery from %s...",
                         toString(safety_mode_));
    recovery_state_ = RecoveryState::INIT;
  }

  // State transitions based on
  // https://docs.universal-robots.com/tutorials/controlling-robot-externally/stop-recovery.html
  switch (recovery_state_) {
  case RecoveryState::IDLE:
    break;
  case RecoveryState::INIT:
    // Determine what comes first based on the trigger mode.
    switch (recovery_trigger_mode_) {
    case SafetyMode::FAULT:
      recovery_next_state_ = RecoveryState::RESTARTING_SAFETY;
      asyncTrigger(close_safety_popup_client_, "close_safety_popup");
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Recovery next state: %s",
                           toString(recovery_next_state_));
      break;
    case SafetyMode::PROTECTIVE_STOP:
      recovery_state_ = RecoveryState::UNLOCKING_PROTECTIVE_STOP;
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Recovery next state: %s",
                           toString(recovery_state_));
      break;
    case SafetyMode::NORMAL:
      switch (robot_mode_) {
      case RobotMode::IDLE:
        recovery_next_state_ = RecoveryState::RELEASING_BRAKES;
        asyncTrigger(close_safety_popup_client_, "close_safety_popup");
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Recovery next state: %s",
                             toString(recovery_next_state_));
        break;
      case RobotMode::POWER_OFF:
        recovery_state_ = RecoveryState::POWERING_ON;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Recovery next state: %s",
                             toString(recovery_state_));
        break;
      case RobotMode::POWER_ON:
        recovery_state_ = RecoveryState::RELEASING_BRAKES;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Recovery next state: %s",
                             toString(recovery_state_));
        break;
      default:
        RCLCPP_WARN(node_->get_logger(),
                    "Recovery (INIT): no recovery procedure for %s / %s.",
                    toString(recovery_trigger_mode_), toString(robot_mode_));
        recovery_state_ = RecoveryState::INIT;
        last_recovery_attempt_ = now;
        break;
      }
      break;
    default:
      RCLCPP_WARN(node_->get_logger(),
                  "Recovery: no recovery procedure for %s.",
                  toString(recovery_trigger_mode_));
      recovery_state_ = RecoveryState::IDLE;
      last_recovery_attempt_ = now;
      return;
    }
    break;

  case RecoveryState::UNLOCKING_PROTECTIVE_STOP:
    recovery_next_state_ = RecoveryState::INIT;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Recovery next state: %s",
                         toString(recovery_next_state_));
    asyncTrigger(unlock_protective_stop_client_, "unlock_protective_stop");
    break;

  case RecoveryState::RESTARTING_SAFETY:
    switch (robot_mode_) {
    case RobotMode::POWER_OFF:
      recovery_next_state_ = RecoveryState::POWERING_ON;
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Recovery next state: %s",
                           toString(recovery_next_state_));
      asyncTrigger(restart_safety_client_, "restart_safety");
      break;
    default:
      RCLCPP_WARN(
          node_->get_logger(),
          "Recovery (RESTARTING_SAFETY): no recovery procedure for %s / %s.",
          toString(recovery_trigger_mode_), toString(robot_mode_));
      recovery_state_ = RecoveryState::INIT;
      last_recovery_attempt_ = now;
      break;
    }
    break;

  case RecoveryState::POWERING_ON:
    switch (robot_mode_) {
    case RobotMode::IDLE:
    case RobotMode::POWER_OFF:
      recovery_next_state_ = RecoveryState::RELEASING_BRAKES;
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Recovery next state: %s",
                           toString(recovery_next_state_));
      asyncTrigger(power_on_client_, "power_on");
      break;
    default:
      RCLCPP_WARN(node_->get_logger(),
                  "Recovery (POWERING_ON): no recovery procedure for %s / %s.",
                  toString(recovery_trigger_mode_), toString(robot_mode_));
      recovery_state_ = RecoveryState::INIT;
      last_recovery_attempt_ = now;
      break;
    }
    break;

  case RecoveryState::RELEASING_BRAKES:
    switch (robot_mode_) {
    case RobotMode::IDLE:
      recovery_next_state_ = RecoveryState::IDLE;
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Recovery next state: %s",
                           toString(recovery_next_state_));
      asyncTrigger(power_on_client_, "brake_release");
      break;
    default:
      RCLCPP_WARN(
          node_->get_logger(),
          "Recovery (RELEASING_BRAKES): no recovery procedure for %s / %s.",
          toString(recovery_trigger_mode_), toString(robot_mode_));
      recovery_state_ = RecoveryState::INIT;
      last_recovery_attempt_ = now;
      break;
    }
    break;
  }
}

void UrRobotMonitor::tryRestartExternalProgram() {
  const auto now = node_->get_clock()->now();

  // Reset state machine when program is playing again
  if (program_mode_ == ProgramMode::PLAYING) {
    if (program_restart_state_ != ProgramRestartState::IDLE) {
      RCLCPP_INFO(node_->get_logger(), "External program is playing again.");
    }
    program_restart_state_ = ProgramRestartState::IDLE;
    return;
  }

  // Only attempt restart when robot is RUNNING, safety is NORMAL, but program
  // is STOPPED or PAUSED
  if (robot_mode_ != RobotMode::RUNNING || safety_mode_ != SafetyMode::NORMAL ||
      program_mode_ == ProgramMode::PLAYING) {
    return;
  }

  switch (program_restart_state_) {
  case ProgramRestartState::IDLE: {
    // Enforce cooldown between attempts
    if ((now - last_program_restart_attempt_).seconds() <
        kProgramRestartCooldown) {
      return;
    }
    if (!load_program_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "load_program service not available, waiting...");
      return;
    }
    RCLCPP_INFO(node_->get_logger(),
                "Program stopped while robot is ready. Loading %s...",
                ur_program_name_.c_str());
    auto request = std::make_shared<ur_dashboard_msgs::srv::Load::Request>();
    request->filename = ur_program_name_;
    load_program_client_->async_send_request(
        request,
        [this](
            rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedFuture future) {
          auto result = future.get();
          if (result->success) {
            RCLCPP_INFO(node_->get_logger(), "%s loaded successfully.",
                        ur_program_name_.c_str());
            program_restart_state_ = ProgramRestartState::WAITING_FOR_PLAY;
          } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to load %s: %s",
                        ur_program_name_.c_str(), result->answer.c_str());
            program_restart_state_ = ProgramRestartState::IDLE;
            last_program_restart_attempt_ = node_->get_clock()->now();
          }
        });
    program_restart_state_ = ProgramRestartState::LOADING;
    last_program_restart_attempt_ = now;
    break;
  }
  case ProgramRestartState::LOADING:
    // Waiting for load callback to fire
    break;
  case ProgramRestartState::WAITING_FOR_PLAY: {
    if (!play_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "play service not available, waiting...");
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Playing %s...", ur_program_name_.c_str());
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    play_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          auto result = future.get();
          if (result->success) {
            RCLCPP_INFO(node_->get_logger(),
                        "%s play command sent successfully.",
                        ur_program_name_.c_str());
          } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to play %s: %s",
                        ur_program_name_.c_str(), result->message.c_str());
          }
          program_restart_state_ = ProgramRestartState::IDLE;
          last_program_restart_attempt_ = node_->get_clock()->now();
        });
    program_restart_state_ = ProgramRestartState::PLAYING;
    break;
  }
  case ProgramRestartState::PLAYING:
    // Waiting for play callback to fire
    break;
  }
}

const char *UrRobotMonitor::toString(RobotMode mode) {
  switch (mode) {
  case RobotMode::NO_CONTROLLER:
    return "NO_CONTROLLER";
  case RobotMode::DISCONNECTED:
    return "DISCONNECTED";
  case RobotMode::CONFIRM_SAFETY:
    return "CONFIRM_SAFETY";
  case RobotMode::BOOTING:
    return "BOOTING";
  case RobotMode::POWER_OFF:
    return "POWER_OFF";
  case RobotMode::POWER_ON:
    return "POWER_ON";
  case RobotMode::IDLE:
    return "IDLE";
  case RobotMode::BACKDRIVE:
    return "BACKDRIVE";
  case RobotMode::RUNNING:
    return "RUNNING";
  case RobotMode::UPDATING_FIRMWARE:
    return "UPDATING_FIRMWARE";
  default:
    return "UNKNOWN_ROBOT_MODE";
  }
}

const char *UrRobotMonitor::toString(SafetyMode mode) {
  switch (mode) {
  case SafetyMode::NORMAL:
    return "NORMAL";
  case SafetyMode::REDUCED:
    return "REDUCED";
  case SafetyMode::PROTECTIVE_STOP:
    return "PROTECTIVE_STOP";
  case SafetyMode::RECOVERY:
    return "RECOVERY";
  case SafetyMode::SAFEGUARD_STOP:
    return "SAFEGUARD_STOP";
  case SafetyMode::SYSTEM_EMERGENCY_STOP:
    return "SYSTEM_EMERGENCY_STOP";
  case SafetyMode::ROBOT_EMERGENCY_STOP:
    return "ROBOT_EMERGENCY_STOP";
  case SafetyMode::VIOLATION:
    return "VIOLATION";
  case SafetyMode::FAULT:
    return "FAULT";
  case SafetyMode::VALIDATE_JOINT_ID:
    return "VALIDATE_JOINT_ID";
  case SafetyMode::UNDEFINED_SAFETY_MODE:
    return "UNDEFINED_SAFETY_MODE";
  case SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP:
    return "AUTOMATIC_MODE_SAFEGUARD_STOP";
  case SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP:
    return "SYSTEM_THREE_POSITION_ENABLING_STOP";
  default:
    return "UNKNOWN_SAFETY_MODE";
  }
}

const char *UrRobotMonitor::toString(ProgramMode mode) {
  switch (mode) {
  case ProgramMode::STOPPED:
    return "STOPPED";
  case ProgramMode::PLAYING:
    return "PLAYING";
  case ProgramMode::PAUSED:
    return "PAUSED";
  default:
    return "UNKNOWN_PROGRAM_MODE";
  }
}

const char *UrRobotMonitor::toString(RecoveryState state) {
  switch (state) {
  case RecoveryState::INIT:
    return "INIT";
  case RecoveryState::IDLE:
    return "IDLE";
  case RecoveryState::UNLOCKING_PROTECTIVE_STOP:
    return "UNLOCKING_PROTECTIVE_STOP";
  case RecoveryState::RESTARTING_SAFETY:
    return "RESTARTING_SAFETY";
  case RecoveryState::POWERING_ON:
    return "POWERING_ON";
  case RecoveryState::RELEASING_BRAKES:
    return "RELEASING_BRAKES";
  default:
    return "UNKNOWN_RECOVERY_STATE";
  }
}

} // namespace combined_impedance_controller
