#ifndef EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED
#define EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <mutex>

#include <effort_controller_base/effort_controller_base.h>

#include "controller_interface/controller_interface.hpp"
#include "debug_msg/msg/debug.hpp"
#include "effort_controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

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

  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

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
  const ctrl::MatrixND m_joint_stiffness;
  const ctrl::MatrixND m_joint_damping;
  const ctrl::MatrixND m_joint_integral_gain;

  // ===========================
  // = Common config variables =
  // ===========================
  double m_max_impendance_force;
  ctrl::Vector6D m_target_wrench;
  std::string tf_prefix;
 private:
  void targetWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void ftSensorWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void targetFrameCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr target);
  void targetJointsCallback(
      const sensor_msgs::msg::JointState::SharedPtr target);
  void heartbeatCallback(const std_msgs::msg::Bool::SharedPtr msg);
  ctrl::Vector6D computeCartMotionError();
  ctrl::VectorND computeJointMotionError();
  void freezeDesiredPoses();

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      m_heartbeat_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_target_wrench_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      m_target_joints_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_ft_sensor_subscriber;
  rclcpp::Publisher<debug_msg::msg::Debug>::SharedPtr m_data_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_data_impedance_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_robot_mode_publisher;

  // Debug publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_goal_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
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
  ctrl::VectorND m_target_joints;

  // ===========================
  // = Common member variables =
  // ===========================
  ctrl::Vector6D m_ft_sensor_wrench;
  std::string m_ft_sensor_ref_link;
  KDL::Frame m_ft_sensor_transform;

  ctrl::MatrixND m_identity;

  bool m_compensate_dJdq = false;
  bool m_debug_topics = false;

  enum class ControlMode
  {
    CARTESIAN,
    JOINT,
  };
  ControlMode control_mode;

  enum class StateInterfaces
  {
    ROBOT_MODE = 12,
    SAFETY_MODE = 13,
    PROGRAM_RUNNING = 14,
  };

  enum class RobotMode {
    NO_CONTROLLER=-1,
    DISCONNECTED=0,
    CONFIRM_SAFETY=1,
    BOOTING=2,
    POWER_OFF=3,
    POWER_ON=4,
    IDLE=5,
    BACKDRIVE=6,
    RUNNING=7,
    UPDATING_FIRMWARE=8,
  };

  enum class SafetyMode {
    NORMAL=1u,
    REDUCED=2,
    PROTECTIVE_STOP=3,
    RECOVERY=4,
    SAFEGUARD_STOP=5,
    SYSTEM_EMERGENCY_STOP=6,
    ROBOT_EMERGENCY_STOP=7,
    VIOLATION=8,
    FAULT=9,
    VALIDATE_JOINT_ID=10,
    UNDEFINED_SAFETY_MODE=11,
    AUTOMATIC_MODE_SAFEGUARD_STOP=12,
    SYSTEM_THREE_POSITION_ENABLING_STOP=13,
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

  static const char* toString(RobotMode mode);
  static const char* toString(SafetyMode mode);
  static const char* toString(ProgramMode mode);
  void updateRobotState();

  RobotMode robot_mode;
  SafetyMode safety_mode;
  ProgramMode program_mode;
  MimicRobotMode mimic_robot_mode = MimicRobotMode::UNKNOWN;

  enum ControllerState {
    RUNNING,
    WAITING,
    STOPPED
  };

  ControllerState controller_state{ControllerState::STOPPED};
  struct FrozenPose {
    KDL::Frame pose;
  };
  FrozenPose frozen_pose;
  std::atomic<bool> is_safe{true}; ///< Safety flag (atomic for thread safety).
  rclcpp::Time last_heartbeat_time;     ///< Timestamp of the last received heartbeat.
  std::mutex heartbeat_mutex; ///< Mutex to protect last_heartbeat_time_ access.
  std::atomic<bool> initial_heartbeat_received{false}; ///< Flag to indicate if the first heartbeat was received (atomic for thread safety).
};

}  // namespace combined_impedance_controller

#endif