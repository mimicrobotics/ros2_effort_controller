#ifndef COMBINED_IMPEDANCE_CONTROLLER_H_INCLUDED
#define COMBINED_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <effort_controller_base/effort_controller_base.h>

#include "controller_interface/controller_interface.hpp"
#include "debug_msg/msg/debug.hpp"
#include "effort_controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "combined_impedance_controller/robot_monitor.h"

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
  ~CombinedImpedanceController() override;

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

  using Base = effort_controller_base::EffortControllerBase;

private:
  ctrl::VectorND computeTorque(double dt);

  void targetWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void ftSensorWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void
  targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target);
  void jointTrajectoryCallback(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr target);
  ctrl::Vector6D
  computeCartMotionError(const KDL::Frame &target_frame_snapshot);
  ctrl::VectorND computeJointMotionError();
  ctrl::VectorND computeJointTrajectoryTaskTorque(const ctrl::VectorND &q_dot);
  ctrl::VectorND computeCartesianTaskTorque(
      const ctrl::MatrixND &jac, const ctrl::VectorND &q_dot,
      const ctrl::Matrix6D &Lambda, const KDL::Frame &target_frame_snapshot,
      const KDL::Frame &current_frame, double dt);
  struct DebugSnapshot; // forward-declared, defined below
  void publishDebugTopics(const DebugSnapshot &snap);
  void freezeDesiredPoses();
  void updateNextTrajectoryPoint(const rclcpp::Duration &period);
  static ctrl::Vector6D toVector6D(const geometry_msgs::msg::Wrench &w);

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_target_wrench_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      m_target_joint_trajectory_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_ft_sensor_subscriber;

  // Publishers
  rclcpp::Publisher<debug_msg::msg::Debug>::SharedPtr m_data_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      m_data_impedance_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_current_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_next_goal_pose_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_angle_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      m_tau_stiffness_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      m_tau_damping_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      m_tau_total_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      m_tau_commanded_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      m_tau_velocity_limit_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      m_tau_integral_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_control_mode_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_trajectory_ack_pub;

  enum class ControlMode {
    CARTESIAN,
    JOINT_TRAJECTORY,
  };

  // Debug state cached by computeCartMotionError for publishDebugTopics
  KDL::Frame m_debug_target_frame;
  KDL::Frame m_debug_next_goal_frame;
  double m_debug_angle{0.0};
  bool m_debug_cart_valid{false};

  // Async debug publish thread (throttled to 10 Hz)
  struct DebugSnapshot {
    ctrl::VectorND tau_stiffness;
    ctrl::VectorND tau_damping;
    ctrl::VectorND tau_integral;
    ctrl::VectorND tau_velocity_limit;
    ctrl::VectorND tau_total;
    ctrl::VectorND tau_commanded;
    KDL::Frame target_frame;
    KDL::Frame current_frame;
    KDL::Frame next_goal_frame;
    double angle{0.0};
    bool cart_valid{false};
    ControlMode control_mode{ControlMode::CARTESIAN};
  };
  std::mutex m_debug_mutex;
  std::condition_variable m_debug_cv;
  DebugSnapshot m_debug_snapshot;
  bool m_debug_snapshot_ready{false};
  std::atomic<bool> m_debug_thread_running{false};
  std::thread m_debug_thread;
  void debugPublishLoop();

  // Controller mode service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_mode_switch_srv;
  void modeSwitchCallback(std_srvs::srv::SetBool::Request::SharedPtr req,
                          std_srvs::srv::SetBool::Response::SharedPtr res);

  // Mode heartbeat (received from Python while in JOINT_TRAJECTORY)
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_mode_heartbeat_sub;
  void modeHeartbeatCallback(const std_msgs::msg::Empty::SharedPtr msg);
  rclcpp::Time m_last_mode_heartbeat_time;
  std::mutex m_mode_heartbeat_mutex;
  std::atomic<bool> m_mode_heartbeat_received{false};

  // Trajectory execution state
  std::vector<ctrl::VectorND> m_traj_positions;
  std::vector<ctrl::VectorND> m_traj_velocities;
  std::vector<double> m_traj_times;
  double m_traj_elapsed{0.0};
  bool m_traj_active{false};
  std::mutex m_traj_mutex; ///< Guards traj state and control mode transitions.
  /// Stamp of the last accepted trajectory, used to dedup Python-side retries
  /// that republish the same trajectory when an ack is delayed.
  rclcpp::Time m_last_accepted_traj_stamp{0, 0, RCL_ROS_TIME};
  static constexpr double kModeHeartbeatTimeout{0.3}; ///< 300ms watchdog.

#if LOGGING
  XBot::MatLogger2::Ptr m_logger;
#endif

  // Config variables for cartesian impedance control
  ctrl::Matrix6D m_cartesian_stiffness;
  ctrl::Matrix6D m_cartesian_damping;
  ctrl::Matrix6D m_cartesian_integral_gain;
  double m_null_space_stiffness;
  double m_null_space_damping;
  double m_damping_ratio_trans;
  double m_damping_ratio_rot;

  // Config variables for joint impedance control
  ctrl::MatrixND m_joint_stiffness;
  ctrl::MatrixND m_joint_damping;
  ctrl::MatrixND m_joint_integral_gain;

  // Member variables for cartesian impedance control
  KDL::Frame m_target_frame;
  KDL::Frame m_current_frame;
  ctrl::VectorND m_q_ns; // Null space configuration
  ctrl::Vector6D m_cart_motion_error_integral;
  KDL::Frame
      m_prev_target_frame; ///< Previous target for goal-change detection.
  double m_integral_activation_threshold_lin{0.02};  ///< meters
  double m_integral_activation_threshold_rot{0.05};  ///< radians
  double m_integral_goal_change_threshold_lin{0.01}; ///< meters
  double m_integral_goal_change_threshold_rot{0.03}; ///< radians

  // Member variables for joint impedance control
  ctrl::VectorND m_desired_joint_positions{};
  ctrl::VectorND m_desired_joint_velocities{};
  ctrl::VectorND m_joint_motion_error_integral{};

  // Member variables for effort blending
  ctrl::VectorND m_last_tau_task{};
  ctrl::VectorND m_blend_tau_ff{};
  bool m_blend_active{false};  ///< Protected by m_traj_mutex.
  double m_blend_elapsed{0.0}; ///< Seconds since blend started.
  static constexpr double kBlendTimeConstant{
      0.05}; ///< 50 ms exponential decay.

  // Common member variables
  ctrl::Vector6D m_target_wrench;
  ctrl::Vector6D m_ft_sensor_wrench;
  std::string m_ft_sensor_ref_link;
  KDL::Frame m_ft_sensor_transform;
  std::string m_tf_prefix;

  ctrl::MatrixND m_identity;

  bool m_compensate_dJdq = false;
  bool m_debug_topics = false;
  ctrl::VectorND m_last_stiffness_torque;
  ctrl::VectorND m_last_damping_torque;
  ctrl::VectorND m_last_integral_torque;

  // Per-joint velocity limits (rad/s). Zero means no limit for that joint.
  ctrl::VectorND m_joint_velocity_limits;
  double m_velocity_limit_damping{
      50.0}; ///< Braking gain when over speed limit.
  ctrl::VectorND applyJointVelocityLimits(ctrl::VectorND &tau);

  std::atomic<ControlMode> m_control_mode{ControlMode::CARTESIAN};

  /// Protects m_target_wrench, m_ft_sensor_wrench, and m_target_frame
  /// (written from subscription callbacks, read from update thread).
  std::mutex m_input_mutex;

  std::unique_ptr<RobotMonitor> m_robot_monitor;
  mutable size_t m_monitor_state_iface_offset{0};
  mutable size_t m_monitor_state_iface_count{0};
  KDL::Frame m_frozen_pose;

  // Update frequency tracking
  unsigned long m_freq_call_count{0};
  rclcpp::Time m_freq_last_report_time;
  bool m_freq_initialized{false};

  // Cycle-time profiling (throttled debug output every 5 s)
  struct TimingStats {
    double sum{0.0};
    double max{0.0};
    unsigned long count{0};
    void record(double us) {
      sum += us;
      ++count;
      if (us > max)
        max = us;
    }
    void reset() {
      sum = 0.0;
      max = 0.0;
      count = 0;
    }
    double avg() const { return count > 0 ? sum / count : 0.0; }
  };
  TimingStats m_timing_update_joint_states;
  TimingStats m_timing_update_state;
  TimingStats m_timing_monitor_update;
  TimingStats m_timing_compute_torque;
  TimingStats m_timing_velocity_limits;
  TimingStats m_timing_effort_cmds;
  TimingStats m_timing_write_cmds;
  TimingStats m_timing_trajectory;
  TimingStats m_timing_debug_publish;
  TimingStats m_timing_total;
  std::chrono::steady_clock::time_point m_timing_last_report{
      std::chrono::steady_clock::now()};
};

} // namespace combined_impedance_controller

#endif
