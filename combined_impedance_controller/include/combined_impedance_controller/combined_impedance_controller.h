#ifndef EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED
#define EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <mutex>

#include <effort_controller_base/effort_controller_base.h>

#include "controller_interface/controller_interface.hpp"
#include "debug_msg/msg/debug.hpp"
#include "effort_controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_dashboard_msgs/srv/load.hpp>

#define DEBUG 0
#if LOGGING
#include <matlogger2/matlogger2.h>
#endif

namespace combined_impedance_controller {

/**
 * @brief A ROS2-control controller for Effort force control
 *
 * This controller implements 6-dimensional end effector force control for
 * robots with a wrist force-torque sensor.  Users command
 * geometry_msgs::msg::WrenchStamped targets to steer the robot in task space.
 * The controller additionally listens to the specified force-torque sensor
 * signals and computes the superposition with the target wrench.
 *
 * The underlying solver maps this remaining wrench to joint motion.
 * Users can steer their robot with this control in free space. The speed of
 * the end effector motion is set with PD gains on each Effort axes.
 * In contact, the controller regulates the net force of the two wrenches to
 * zero.
 *
 * Note that during free motion, users can generally set higher control gains
 * for faster motion.  In contact with the environment, however, normally lower
 * gains are required to maintain stability.  The ranges to operate in mainly
 * depend on the stiffness of the environment and the controller cycle of the
 * real hardware, such that some experiments might be required for each use
 * case.
 *
 */
class CombinedImpedanceController
    : public virtual effort_controller_base::EffortControllerBase {
public:
  CombinedImpedanceController();

  virtual LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  virtual controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  void updateControllerState();

  ctrl::VectorND computeTorque();

  using Base = effort_controller_base::EffortControllerBase;

  // ====================================================
  // = Config variables for cartesian impedance control =
  // ====================================================
  ctrl::Matrix6D m_cartesian_stiffness;
  ctrl::Matrix6D m_cartesian_damping;
  ctrl::Matrix6D m_cartesian_integral_gain;
  double m_null_space_stiffness;
  double m_null_space_damping;
  double m_damping_ratio;

  // ================================================
  // = Config variables for joint impedance control =
  // ================================================
  ctrl::MatrixND m_joint_stiffness;
  ctrl::MatrixND m_joint_damping;
  ctrl::MatrixND m_joint_integral_gain;

  // ===========================
  // = Common config variables =
  // ===========================
  ctrl::Vector6D m_target_wrench;
  std::string tf_prefix;

private:
  void targetWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void ftSensorWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void
  targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target);
  void jointTrajectoryCallback(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr target);
  void heartbeatCallback(const std_msgs::msg::Bool::SharedPtr msg);
  ctrl::Vector6D computeCartMotionError();
  ctrl::VectorND computeJointMotionError();
  ctrl::VectorND computeJointTrajectoryTaskTorque(const ctrl::VectorND &q_dot);
  ctrl::VectorND computeCartesianTaskTorque(const ctrl::MatrixND &jac,
                                            const ctrl::VectorND &q_dot,
                                            const ctrl::Matrix6D &Lambda);
  void publishDebugTopics(const ctrl::VectorND &tau);
  void freezeDesiredPoses();
  void updateNextTrajectoryPoint(const rclcpp::Duration &period);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_heartbeat_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_target_wrench_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      m_target_joint_trajectory_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_ft_sensor_subscriber;
  rclcpp::Publisher<debug_msg::msg::Debug>::SharedPtr m_data_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      m_data_impedance_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_robot_mode_publisher;

  // Debug publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      next_goal_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr control_mode_pub_;

  // Debug state cached by computeCartMotionError for publishDebugTopics
  KDL::Frame debug_target_frame_;
  KDL::Frame debug_next_goal_frame_;
  double debug_angle_{0.0};
  bool debug_cart_valid_{
      false}; ///< True when cart debug data was updated this cycle.

  // Controller mode service (replaces topic-based mode switching)
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_switch_srv_;
  bool modeSwitchCallback(std_srvs::srv::SetBool::Request::SharedPtr req,
                          std_srvs::srv::SetBool::Response::SharedPtr res);

  // Mode heartbeat (received from Python while in JOINT_TRAJECTORY)
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mode_heartbeat_sub_;
  void modeHeartbeatCallback(const std_msgs::msg::Empty::SharedPtr msg);
  rclcpp::Time last_mode_heartbeat_time_;
  std::mutex mode_heartbeat_mutex_;
  std::atomic<bool> mode_heartbeat_received_{false};

  // Trajectory execution state
  std::vector<ctrl::VectorND> traj_positions_;
  std::vector<ctrl::VectorND> traj_velocities_;
  std::vector<double> traj_times_;
  double traj_elapsed_{0.0};
  bool traj_active_{false};
  std::mutex traj_mutex_; ///< Guards traj_* and control_mode_.
  static constexpr double kModeHeartbeatTimeout{0.3}; ///< 300ms watchdog.

#if LOGGING
  XBot::MatLogger2::Ptr m_logger;
#endif

  // ====================================================
  // = Member variables for cartesian impedance control =
  // ====================================================
  KDL::Frame m_target_frame;
  KDL::JntArray m_null_space;
  KDL::Frame m_current_frame;
  ctrl::VectorND m_q_ns; // Null space configuration
  ctrl::Vector6D m_cart_motion_error_integral;

  // ================================================
  // = Member variables for joint impedance control =
  // ================================================
  ctrl::VectorND m_desired_joint_positions_{};
  ctrl::VectorND m_desired_joint_velocities_{};
  ctrl::VectorND m_joint_motion_error_integral{};

  // ========================================
  // = Member variables for effort blending =
  // ========================================
  ctrl::VectorND m_last_tau_task{};
  ctrl::VectorND m_blend_tau_ff_{};
  bool blend_active_{false};  ///< Protected by traj_mutex_.
  double blend_elapsed_{0.0}; ///< Seconds since blend started.
  static constexpr double kBlendTimeConstant{
      0.05}; ///< 50 ms exponential decay.

  // ===========================
  // = Common member variables =
  // ===========================
  ctrl::Vector6D m_ft_sensor_wrench;
  std::string m_ft_sensor_ref_link;
  KDL::Frame m_ft_sensor_transform;

  ctrl::MatrixND m_identity;

  bool m_compensate_dJdq = false;
  bool m_debug_topics = false;

  enum class ControlMode {
    CARTESIAN,
    JOINT_TRAJECTORY,
  };
  ControlMode control_mode;

  enum class StateInterfaces {
    ROBOT_MODE = 12,
    SAFETY_MODE = 13,
    PROGRAM_RUNNING = 14,
  };

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

  enum class MimicRobotMode {
    UNKNOWN = 0u,
    IDLE = 1,
    MOVE = 2,
    USER_STOPPED = 3,
  };

  static const char *toString(RobotMode mode);
  static const char *toString(SafetyMode mode);
  static const char *toString(ProgramMode mode);
  void updateRobotState();

  RobotMode robot_mode;
  SafetyMode safety_mode;
  ProgramMode program_mode;
  MimicRobotMode mimic_robot_mode = MimicRobotMode::UNKNOWN;

  enum ControllerState { RUNNING, WAITING, STOPPED };

  ControllerState controller_state{ControllerState::STOPPED};
  struct FrozenPose {
    KDL::Frame pose;
  };
  FrozenPose frozen_pose;
  std::atomic<bool> is_safe{true}; ///< Safety flag (atomic for thread safety).
  rclcpp::Time
      last_heartbeat_time;    ///< Timestamp of the last received heartbeat.
  std::mutex heartbeat_mutex; ///< Mutex to protect last_heartbeat_time_ access.
  std::atomic<bool> initial_heartbeat_received{
      false}; ///< Flag to indicate if the first heartbeat was received (atomic
              ///< for thread safety).

  // External program auto-restart
  enum class ProgramRestartState {
    IDLE,             ///< Not attempting restart.
    LOADING,          ///< load_program service call in flight.
    WAITING_FOR_PLAY, ///< Load succeeded, about to call play.
    PLAYING,          ///< play service call in flight.
  };
  ProgramRestartState program_restart_state_{ProgramRestartState::IDLE};
  rclcpp::Client<ur_dashboard_msgs::srv::Load>::SharedPtr load_program_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr play_client_;
  rclcpp::Time last_program_restart_attempt_;
  static constexpr double kProgramRestartCooldown{
      3.0};                     ///< Seconds between retry attempts.
  std::string ur_program_name_; ///< UR program filename to load (e.g.
                                ///< "ext_control.urp").
  std::string
      dashboard_prefix_; ///< Service namespace prefix for dashboard client.
  void tryRestartExternalProgram();
};

} // namespace combined_impedance_controller

#endif
