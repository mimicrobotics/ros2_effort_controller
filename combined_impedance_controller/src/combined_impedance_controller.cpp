#include <combined_impedance_controller/combined_impedance_controller.h>
#include <combined_impedance_controller/ur_robot_monitor.h>
#include <exception>

namespace combined_impedance_controller {

CombinedImpedanceController::CombinedImpedanceController()
    : Base::EffortControllerBase() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CombinedImpedanceController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  auto_declare<std::string>("tf_prefix", "");
  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);
  auto_declare<double>("nullspace_stiffness", 0.0);
  auto_declare<bool>("compensate_dJdq", false);
  auto_declare<bool>("debug_topics",
                     false); // Publish additional topics for debugging
  auto_declare<std::vector<double>>("nullspace_desired_configuration",
                                    std::vector<double>());

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;
  auto_declare<double>("cart_stiffness.trans_x", default_lin_stiff);
  auto_declare<double>("cart_stiffness.trans_y", default_lin_stiff);
  auto_declare<double>("cart_stiffness.trans_z", default_lin_stiff);
  auto_declare<double>("cart_stiffness.rot_x", default_rot_stiff);
  auto_declare<double>("cart_stiffness.rot_y", default_rot_stiff);
  auto_declare<double>("cart_stiffness.rot_z", default_rot_stiff);
  auto_declare<std::vector<double>>("joint_stiffness", std::vector<double>());

  // Disable integral gain by default to avoid windup issues, can be enabled
  // with parameters
  constexpr double default_lin_integral = 0.0;
  constexpr double default_rot_integral = 0.0;
  auto_declare<double>("cart_integral_gain.trans_x", default_lin_integral);
  auto_declare<double>("cart_integral_gain.trans_y", default_lin_integral);
  auto_declare<double>("cart_integral_gain.trans_z", default_lin_integral);
  auto_declare<double>("cart_integral_gain.rot_x", default_rot_integral);
  auto_declare<double>("cart_integral_gain.rot_y", default_rot_integral);
  auto_declare<double>("cart_integral_gain.rot_z", default_rot_integral);
  auto_declare<double>("cart_damping_ratio", std::sqrt(2.0) / 2.0);
  auto_declare<std::vector<double>>("joint_integral_gain",
                                    std::vector<double>());

  // Integral conditional-activation thresholds: the integral term only
  // accumulates when the Cartesian error is below these values, keeping it
  // inactive during large transient motions.
  auto_declare<double>("integral_activation_threshold_lin", 0.02); // meters
  auto_declare<double>("integral_activation_threshold_rot", 0.05); // radians
  // Goal-change thresholds: the integral is reset when a new target differs
  // from the previous one by more than these values.
  auto_declare<double>("integral_goal_change_threshold_lin", 0.01); // meters
  auto_declare<double>("integral_goal_change_threshold_rot", 0.03); // radians

  // Define upper limit for impedance forces
  auto_declare<double>("max_impedance_force", 70.0);

  // Per-joint velocity limits (rad/s). Empty or all-zero = no limiting.
  auto_declare<std::vector<double>>("joint_velocity_limits",
                                    std::vector<double>());
  auto_declare<double>("velocity_limit_damping", 50.0);

  // External program auto-restart parameters (declared here for the UR monitor)
  auto_declare<std::string>("ur_program_name", "ext_control.urp");
  auto_declare<std::string>("dashboard_prefix", "/dashboard_client");

  m_robot_monitor = std::make_unique<UrRobotMonitor>(get_node());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CombinedImpedanceController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  m_tf_prefix = get_node()->get_parameter("tf_prefix").as_string();

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link =
      get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!Base::robotChainContains(m_ft_sensor_ref_link)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link
                            << " is not part of the kinematic chain from "
                            << Base::m_robot_base_link << " to "
                            << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  // Set cartesian stiffness
  ctrl::Vector6D tmp;
  tmp[0] = get_node()->get_parameter("cart_stiffness.trans_x").as_double();
  tmp[1] = get_node()->get_parameter("cart_stiffness.trans_y").as_double();
  tmp[2] = get_node()->get_parameter("cart_stiffness.trans_z").as_double();
  tmp[3] = get_node()->get_parameter("cart_stiffness.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("cart_stiffness.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("cart_stiffness.rot_z").as_double();

  m_cartesian_stiffness = tmp.asDiagonal();

  // Set cartesian damping
  tmp[0] = 2 * sqrt(tmp[0]);
  tmp[1] = 2 * sqrt(tmp[1]);
  tmp[2] = 2 * sqrt(tmp[2]);
  tmp[3] = 2 * sqrt(tmp[3]);
  tmp[4] = 2 * sqrt(tmp[4]);
  tmp[5] = 2 * sqrt(tmp[5]);

  m_cartesian_damping = tmp.asDiagonal();

  // Set cartesian integral gain
  tmp[0] = get_node()->get_parameter("cart_integral_gain.trans_x").as_double();
  tmp[1] = get_node()->get_parameter("cart_integral_gain.trans_y").as_double();
  tmp[2] = get_node()->get_parameter("cart_integral_gain.trans_z").as_double();
  tmp[3] = get_node()->get_parameter("cart_integral_gain.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("cart_integral_gain.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("cart_integral_gain.rot_z").as_double();

  m_cartesian_integral_gain = tmp.asDiagonal();
  m_damping_ratio = get_node()->get_parameter("cart_damping_ratio").as_double();

  // Set joint stiffness
  const std::vector<double> joint_stiffness =
      get_node()->get_parameter("joint_stiffness").as_double_array();
  if (joint_stiffness.size() != Base::m_joint_number) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "joint_stiffness configuration size does not match joint number: "
        "%zu != %zu",
        joint_stiffness.size(), Base::m_joint_number);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  m_joint_stiffness = ctrl::MatrixND::Zero(m_joint_number, m_joint_number);
  m_joint_damping = ctrl::MatrixND::Zero(m_joint_number, m_joint_number);
  for (size_t i = 0; i < Base::m_joint_number; ++i) {
    m_joint_stiffness(i, i) = joint_stiffness[i];
    m_joint_damping(i, i) = 1.5 * std::sqrt(joint_stiffness[i]);
  }
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Joint stiffness: " << m_joint_stiffness.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Joint damping: " << m_joint_damping.transpose());

  // Set joint integral gain
  const std::vector<double> joint_integral_gain =
      get_node()->get_parameter("joint_integral_gain").as_double_array();
  if (joint_integral_gain.size() != Base::m_joint_number) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "joint_integral_gain gain configuration size does not match "
                 "joint number: "
                 "%zu != %zu",
                 joint_stiffness.size(), Base::m_joint_number);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }
  m_joint_integral_gain = ctrl::MatrixND::Zero(m_joint_number, m_joint_number);
  for (size_t i = 0; i < Base::m_joint_number; ++i) {
    m_joint_integral_gain(i, i) = joint_integral_gain[i];
  }
  RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Joint integral gain: " << m_joint_integral_gain.transpose());

  // Set nullspace stiffness
  m_null_space_stiffness =
      get_node()->get_parameter("nullspace_stiffness").as_double();
  if (m_null_space_stiffness > 0.0) {
    // Set nullspace configuration
    std::vector<double> nullspace_config =
        get_node()
            ->get_parameter("nullspace_desired_configuration")
            .as_double_array();
    if (nullspace_config.empty()) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Null space configuration is empty, zeroing null space stiffness");
      m_null_space_stiffness = 0.0;
    } else if (nullspace_config.size() != Base::m_joint_number) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Null space configuration size does not match joint number: "
                   "%zu != %zu",
                   nullspace_config.size(), Base::m_joint_number);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::ERROR;
    }
    m_q_ns = ctrl::VectorND::Zero(Base::m_joint_number);
    for (size_t i = 0; i < Base::m_joint_number; ++i) {
      m_q_ns(i) = nullspace_config[i];
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Postural task stiffness: " << m_null_space_stiffness
                                                   << " for configuration: "
                                                   << m_q_ns.transpose());
  }
  m_compensate_dJdq = get_node()->get_parameter("compensate_dJdq").as_bool();
  RCLCPP_INFO(get_node()->get_logger(), "Compensate dJdq: %d",
              m_compensate_dJdq);

  // Integral activation and goal-change thresholds
  m_integral_activation_threshold_lin =
      get_node()
          ->get_parameter("integral_activation_threshold_lin")
          .as_double();
  m_integral_activation_threshold_rot =
      get_node()
          ->get_parameter("integral_activation_threshold_rot")
          .as_double();
  m_integral_goal_change_threshold_lin =
      get_node()
          ->get_parameter("integral_goal_change_threshold_lin")
          .as_double();
  m_integral_goal_change_threshold_rot =
      get_node()
          ->get_parameter("integral_goal_change_threshold_rot")
          .as_double();
  RCLCPP_INFO(get_node()->get_logger(),
              "Integral activation thresholds: lin=%.4f m, rot=%.4f rad",
              m_integral_activation_threshold_lin,
              m_integral_activation_threshold_rot);
  RCLCPP_INFO(get_node()->get_logger(),
              "Integral goal-change thresholds: lin=%.4f m, rot=%.4f rad",
              m_integral_goal_change_threshold_lin,
              m_integral_goal_change_threshold_rot);
  // Set nullspace damping
  m_null_space_damping = 2 * sqrt(m_null_space_stiffness);

  // Set per-joint velocity limits
  const std::vector<double> vel_limits =
      get_node()->get_parameter("joint_velocity_limits").as_double_array();
  m_joint_velocity_limits = ctrl::VectorND::Zero(Base::m_joint_number);
  if (!vel_limits.empty()) {
    if (vel_limits.size() != Base::m_joint_number) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "joint_velocity_limits size does not match joint number: "
                   "%zu != %zu",
                   vel_limits.size(), Base::m_joint_number);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < Base::m_joint_number; ++i) {
      m_joint_velocity_limits(i) = vel_limits[i];
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Joint velocity limits (rad/s): "
                           << m_joint_velocity_limits.transpose());
  }
  m_velocity_limit_damping =
      get_node()->get_parameter("velocity_limit_damping").as_double();

  // Set the identity matrix with dimension of the joint space
  m_identity = ctrl::MatrixND::Identity(m_joint_number, m_joint_number);

  m_target_wrench_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          get_node()->get_name() + std::string("/target_wrench"), 10,
          std::bind(&CombinedImpedanceController::targetWrenchCallback, this,
                    std::placeholders::_1));

  m_ft_sensor_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          get_node()->get_name() + std::string("/ft_sensor_wrench"), 10,
          std::bind(&CombinedImpedanceController::ftSensorWrenchCallback, this,
                    std::placeholders::_1));

  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 1,
          std::bind(&CombinedImpedanceController::targetFrameCallback, this,
                    std::placeholders::_1));

  m_target_joint_trajectory_subscriber =
      get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          std::string("/target_joint_trajectory"), 1,
          std::bind(&CombinedImpedanceController::jointTrajectoryCallback, this,
                    std::placeholders::_1));

  m_data_publisher = get_node()->create_publisher<debug_msg::msg::Debug>(
      get_node()->get_name() + std::string("/data"), 1);

  m_data_impedance_publisher =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          get_node()->get_name() + std::string("/data_impedance"), 1);

  // Service for controller mode switching (replaces topic-based mode command)
  m_mode_switch_srv = get_node()->create_service<std_srvs::srv::SetBool>(
      get_node()->get_name() + std::string("/controller_mode_switch"),
      std::bind(&CombinedImpedanceController::modeSwitchCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(get_node()->get_logger(),
              "CombinedImpedanceController: Mode switch service ready.");

  // Heartbeat subscriber for JOINT_TRAJECTORY watchdog
  m_mode_heartbeat_sub = get_node()->create_subscription<std_msgs::msg::Empty>(
      std::string("/mode_heartbeat"), 1,
      std::bind(&CombinedImpedanceController::modeHeartbeatCallback, this,
                std::placeholders::_1));

  // Get debug topics parameter and create publishers
  m_debug_topics = get_node()->get_parameter("debug_topics").as_bool();
  RCLCPP_INFO(get_node()->get_logger(), "Publishing debug topics: %d",
              m_debug_topics);

  if (m_debug_topics) {
    // Publish current target frame
    m_target_pose_pub =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            get_node()->get_name() + std::string("/debug_target_frame"), 10);

    // Publish current end-effector frame
    m_current_pose_pub =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            get_node()->get_name() + std::string("/debug_current_frame"), 10);

    // Publish clamped goal frame that the controller is actually trying to
    // achieve (after error clamping)
    m_next_goal_pose_pub =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            get_node()->get_name() + std::string("/debug_next_goal_frame"), 10);

    // Publish orientation error angle
    m_angle_pub = get_node()->create_publisher<std_msgs::msg::Float64>(
        get_node()->get_name() + std::string("/debug_orientation_error_angle"),
        10);

    // Publish overall tau
    m_tau_damping_pub =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            get_node()->get_name() + std::string("/debug_tau_damping"), 10);

    m_tau_stiffness_pub =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            get_node()->get_name() + std::string("/debug_tau_stiffness"), 10);

    m_tau_total_pub =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            get_node()->get_name() + std::string("/debug_tau_total"), 10);

    m_tau_commanded_pub =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            get_node()->get_name() + std::string("/debug_tau_commanded"), 10);

    m_tau_velocity_limit_pub =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            get_node()->get_name() + std::string("/debug_tau_velocity_limit"),
            10);

    m_tau_integral_pub =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            get_node()->get_name() + std::string("/debug_tau_integral"), 10);

    // Publish control mode
    m_control_mode_pub = get_node()->create_publisher<std_msgs::msg::Int32>(
        get_node()->get_name() + std::string("/debug_control_mode"), 10);
  }

  // Configure robot monitor (heartbeat, mode publisher, robot-specific setup)
  m_robot_monitor->configure(get_node()->get_name());

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CombinedImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf =
      EffortControllerBase::state_interface_configuration();
  m_monitor_state_iface_offset = conf.names.size();
  auto monitor_ifaces = m_robot_monitor->requiredStateInterfaces(m_tf_prefix);
  m_monitor_state_iface_count = monitor_ifaces.size();
  for (auto &name : monitor_ifaces) {
    conf.names.emplace_back(std::move(name));
  }
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CombinedImpedanceController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  Base::on_activate(previous_state);

  // Update joint states
  Base::updateJointStates();

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Set the target frame to the current frame, same for target joints
  m_target_frame = m_current_frame;
  m_desired_joint_positions = Base::m_joint_positions.data;
  m_desired_joint_velocities = ctrl::VectorND::Zero(Base::m_joint_number);
  m_blend_tau_ff = ctrl::VectorND::Zero(Base::m_joint_number);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");

  // Reset integral terms and target joints
  m_cart_motion_error_integral = ctrl::Vector6D::Zero();
  m_joint_motion_error_integral = ctrl::VectorND::Zero(Base::m_joint_number);

  m_target_wrench = ctrl::Vector6D::Zero();
  m_ft_sensor_wrench = ctrl::Vector6D::Zero();

  m_robot_monitor->activate();

  // Default to cartesian mode on activation
  m_control_mode.store(ControlMode::CARTESIAN);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CombinedImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  // Stop drifting by sending zero joint torques
  Base::computeJointEffortCmds(ctrl::VectorND::Zero(Base::m_joint_number));
  Base::writeJointEffortCmds();
  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type
CombinedImpedanceController::update(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) {
  // Update joint states
  Base::updateJointStates();

  // Update robot monitor (heartbeat check, state machine, mode publish)
  {
    std::vector<double> monitor_values(m_monitor_state_iface_count);
    for (size_t i = 0; i < m_monitor_state_iface_count; ++i) {
      monitor_values[i] =
          state_interfaces_[m_monitor_state_iface_offset + i].get_value();
    }
    m_robot_monitor->updateState(monitor_values);
  }
  if (m_robot_monitor->update()) {
    freezeDesiredPoses();
  }

  ctrl::VectorND tau_tot = computeTorque(period.seconds());

  // Enforce per-joint velocity limits
  ctrl::VectorND tau_vel_limit = applyJointVelocityLimits(tau_tot);

  // Saturation of the torque
  Base::computeJointEffortCmds(tau_tot);

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();

  if (m_control_mode.load() == ControlMode::JOINT_TRAJECTORY) {
    updateNextTrajectoryPoint(period);
  }

  // Compute the task torque
  if (m_debug_topics) {
    publishDebugTopics(m_last_stiffness_torque, m_last_damping_torque,
                       m_last_integral_torque, tau_vel_limit, tau_tot,
                       m_efforts);
  }

  return controller_interface::return_type::OK;
}

geometry_msgs::msg::PoseStamped toPoseStamped(const KDL::Frame &frame,
                                              const std::string &frame_id,
                                              const rclcpp::Time &stamp) {
  geometry_msgs::msg::PoseStamped msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.pose.position.x = frame.p.x();
  msg.pose.position.y = frame.p.y();
  msg.pose.position.z = frame.p.z();

  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);

  msg.pose.orientation.x = x;
  msg.pose.orientation.y = y;
  msg.pose.orientation.z = z;
  msg.pose.orientation.w = w;

  return msg;
}

void CombinedImpedanceController::updateNextTrajectoryPoint(
    const rclcpp::Duration &period) {
  RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Updating trajectory point. Elapsed time: %f seconds", m_traj_elapsed);
  std::lock_guard<std::mutex> lock(m_traj_mutex);
  if (!m_traj_active || m_robot_monitor->controllerState() !=
                            RobotMonitor::ControllerState::RUNNING) {
    return;
  }
  m_traj_elapsed += period.seconds();

  if (m_traj_elapsed >= m_traj_times.back()) {
    // Trajectory complete -- hold final waypoint
    m_desired_joint_positions = m_traj_positions.back();
    m_desired_joint_velocities = ctrl::VectorND::Zero(Base::m_joint_number);
    m_traj_active = false;
    RCLCPP_INFO(get_node()->get_logger(),
                "CombinedImpedanceController: Homing trajectory complete, "
                "holding final position.");
  } else {
    // Find bounding segment: idx0 is the waypoint just before m_traj_elapsed,
    // idx1 is the waypoint just after it.
    auto it = std::upper_bound(m_traj_times.begin(), m_traj_times.end(),
                               m_traj_elapsed);
    const size_t idx1 =
        static_cast<size_t>(std::distance(m_traj_times.begin(), it));
    const size_t idx0 = idx1 - 1;
    const double alpha = (m_traj_elapsed - m_traj_times[idx0]) /
                         (m_traj_times[idx1] - m_traj_times[idx0]);

    // Linear interpolation between the two bounding waypoints
    m_desired_joint_positions =
        m_traj_positions[idx0] +
        alpha * (m_traj_positions[idx1] - m_traj_positions[idx0]);
    m_desired_joint_velocities =
        m_traj_velocities[idx0] +
        alpha * (m_traj_velocities[idx1] - m_traj_velocities[idx0]);

    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         1000, "Setting waypoint:  %lu - %lu", idx0, idx1);
  }
}

void CombinedImpedanceController::publishDebugTopics(
    const ctrl::VectorND &tau_stiffness, const ctrl::VectorND &tau_damping,
    const ctrl::VectorND &tau_integral,
    const ctrl::VectorND &tau_velocity_limit, const ctrl::VectorND &tau_total,
    const ctrl::VectorND &tau_commanded) {
  // Cartesian-mode debug (target/current/next_goal poses + angle)
  if (m_debug_cart_valid) {
    const auto stamp = get_node()->now();
    m_target_pose_pub->publish(
        toPoseStamped(m_debug_target_frame, Base::m_robot_base_link, stamp));
    m_current_pose_pub->publish(
        toPoseStamped(m_current_frame, Base::m_robot_base_link, stamp));
    m_next_goal_pose_pub->publish(
        toPoseStamped(m_debug_next_goal_frame, Base::m_robot_base_link, stamp));

    std_msgs::msg::Float64 angle_msg;
    angle_msg.data = m_debug_angle;
    m_angle_pub->publish(angle_msg);

    m_debug_cart_valid = false;
  }

  // Tau and control mode (always published in debug mode)
  if (m_tau_stiffness_pub) {
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.assign(tau_stiffness.data(),
                        tau_stiffness.data() + tau_stiffness.size());
    m_tau_stiffness_pub->publish(tau_msg);
  }
  if (m_tau_damping_pub) {
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.assign(tau_damping.data(),
                        tau_damping.data() + tau_damping.size());
    m_tau_damping_pub->publish(tau_msg);
  }
  if (m_tau_velocity_limit_pub) {
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.assign(tau_velocity_limit.data(),
                        tau_velocity_limit.data() + tau_velocity_limit.size());
    m_tau_velocity_limit_pub->publish(tau_msg);
  }
  if (m_tau_integral_pub) {
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.assign(tau_integral.data(),
                        tau_integral.data() + tau_integral.size());
    m_tau_integral_pub->publish(tau_msg);
  }
  if (m_tau_total_pub) {
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.assign(tau_total.data(), tau_total.data() + tau_total.size());
    m_tau_total_pub->publish(tau_msg);
  }
  if (m_tau_commanded_pub) {
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.assign(tau_commanded.data(),
                        tau_commanded.data() + tau_commanded.size());
    m_tau_commanded_pub->publish(tau_msg);
  }

  if (m_control_mode_pub) {
    std_msgs::msg::Int32 mode_msg;
    mode_msg.data = static_cast<int32_t>(m_control_mode.load());
    m_control_mode_pub->publish(mode_msg);
  }
}

ctrl::Vector6D
CombinedImpedanceController::toVector6D(const geometry_msgs::msg::Wrench &w) {
  ctrl::Vector6D v;
  v << w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z;
  return v;
}

void CombinedImpedanceController::freezeDesiredPoses() {
  // freeze arm pose with desired pose
  m_frozen_pose = m_current_frame;
  {
    std::lock_guard<std::mutex> lock(m_input_mutex);
    m_target_frame = m_current_frame;
  }
  m_cart_motion_error_integral = ctrl::Vector6D::Zero();
  m_desired_joint_positions = Base::m_joint_positions.data;
  m_blend_tau_ff = ctrl::VectorND::Zero(Base::m_joint_number);
  m_desired_joint_velocities = ctrl::VectorND::Zero(Base::m_joint_number);
  m_joint_motion_error_integral = ctrl::VectorND::Zero(Base::m_joint_number);
}

ctrl::Vector6D CombinedImpedanceController::computeCartMotionError(
    const KDL::Frame &target_frame_snapshot) {
  // Compute the cartesian error between the current and the target frame
  KDL::Frame target_frame;
  if (m_robot_monitor->controllerState() ==
      RobotMonitor::ControllerState::RUNNING) {
    target_frame = target_frame_snapshot;
  } else { // STOPPED or WAITING, use frozen poses
    target_frame = m_frozen_pose;
  }

  // Transformation from target -> current corresponds to error = target -
  // current
  KDL::Frame error_kdl;
  error_kdl.M = target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle = error_kdl.M.GetRotAngle(rot_axis); // rot_axis is normalized
  double distance = error_kdl.p.Normalize();

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.
  // Note that this is also the maximal offset that the
  // cartesian_compliance_controller can use to build up a restoring stiffness
  // wrench.
  const double max_angle = 0.1;
  const double max_distance = 0.1;
  double angle_clamped = std::clamp(angle, -max_angle, max_angle);
  distance = std::clamp(distance, 0.0, max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle_clamped;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  ctrl::Vector6D error;
  error.head<3>() << error_kdl.p.x(), error_kdl.p.y(), error_kdl.p.z();
  error.tail<3>() << rot_axis(0), rot_axis(1), rot_axis(2);

  if (m_debug_topics) {
    m_debug_target_frame = target_frame;
    m_debug_next_goal_frame.M =
        KDL::Rotation::Rot(rot_axis, rot_axis.Norm()) * m_current_frame.M;
    m_debug_next_goal_frame.p = error_kdl.p + m_current_frame.p;
    m_debug_angle = angle;
    m_debug_cart_valid = true;
  }

  return error;
}

ctrl::VectorND CombinedImpedanceController::computeJointMotionError() {
  // Compute the joint space error between the current and the target
  // configuration
  ctrl::VectorND error =
      m_desired_joint_positions - Base::m_joint_positions.data;

  // Clamp the error to avoid excessive torques
  const double max_joint_error = 0.3; // TODO: tune this parameter
  for (auto i = 0; i < error.size(); ++i) {
    error(i) = std::clamp(error(i), -max_joint_error, max_joint_error);
  }
  return error;
}

ctrl::VectorND CombinedImpedanceController::computeJointTrajectoryTaskTorque(
    const ctrl::VectorND &q_dot) {
  // to avoid deadlock (this function is called with m_traj_mutex held).

  // Compute the motion error
  const ctrl::VectorND motion_error = computeJointMotionError();
  RCLCPP_DEBUG_STREAM_THROTTLE(get_node()->get_logger(),
                               *get_node()->get_clock(), 250,
                               "Motion error: \n"
                                   << motion_error.transpose() << "\n");

  // Compute the stiffness and damping in the joint space
  const ctrl::MatrixND K_d = m_joint_stiffness;
  const ctrl::MatrixND D_d = m_joint_damping;
  const ctrl::MatrixND K_i = m_joint_integral_gain;

  // Anti-windup: clamp the integral error to prevent excessive torques
  m_joint_motion_error_integral
      << (m_joint_motion_error_integral + 0.1 * motion_error)
             .cwiseMax(-0.1)
             .cwiseMin(0.1);

  const ctrl::VectorND stiffness_torque = K_d * motion_error;
  const ctrl::VectorND damping_torque =
      D_d * (m_desired_joint_velocities - q_dot);
  const ctrl::VectorND integral_torque = K_i * m_joint_motion_error_integral;

  m_last_stiffness_torque = stiffness_torque;
  m_last_damping_torque = damping_torque;
  m_last_integral_torque = integral_torque;

  // Compute the task torque
  return stiffness_torque + damping_torque + integral_torque;
}

ctrl::VectorND CombinedImpedanceController::computeCartesianTaskTorque(
    const ctrl::MatrixND &jac, const ctrl::VectorND &q_dot,
    const ctrl::Matrix6D &Lambda, const KDL::Frame &target_frame_snapshot,
    double dt) {
  // Compute the motion error
  const ctrl::Vector6D motion_error =
      computeCartMotionError(target_frame_snapshot);

  // Compute the stiffness and damping in the base link
  const auto base_link_stiffness =
      Base::displayInBaseLink(m_cartesian_stiffness, Base::m_end_effector_link);

  const ctrl::Matrix6D K_d = base_link_stiffness;
  const ctrl::Matrix6D D_d =
      compute_correct_damping(Lambda, K_d, m_damping_ratio);
  const ctrl::Matrix6D K_i = m_cartesian_integral_gain;

  // Conditional integration: only accumulate when close to the target so that
  // the integral term does not interfere with transient/dynamic behaviour.
  const double lin_error_norm = motion_error.head(3).norm();
  const double rot_error_norm = motion_error.tail(3).norm();
  if (lin_error_norm < m_integral_activation_threshold_lin &&
      rot_error_norm < m_integral_activation_threshold_rot) {
    // Anti-windup: integrate with dt and clamp to prevent excessive torques
    m_cart_motion_error_integral.head(3)
        << (m_cart_motion_error_integral.head(3) + dt * motion_error.head(3))
               .cwiseMax(-0.1)
               .cwiseMin(0.1);
    m_cart_motion_error_integral.tail(3)
        << (m_cart_motion_error_integral.tail(3) + dt * motion_error.tail(3))
               .cwiseMax(-0.05)
               .cwiseMin(0.05);
  }

  const ctrl::Vector6D stiffness_torque =
      jac.transpose() * (K_d * motion_error);
  const ctrl::Vector6D damping_torque =
      jac.transpose() * (D_d * (-jac * q_dot));
  const ctrl::Vector6D integral_torque =
      jac.transpose() * (K_i * m_cart_motion_error_integral);

  m_last_stiffness_torque = stiffness_torque;
  m_last_damping_torque = damping_torque;
  m_last_integral_torque = integral_torque;

  return stiffness_torque + damping_torque + integral_torque;
}

ctrl::VectorND
CombinedImpedanceController::applyJointVelocityLimits(ctrl::VectorND &tau) {
  constexpr double kBufferRatio = 0.2;
  ctrl::VectorND tau_vel_limit = ctrl::VectorND::Zero(tau.size());

  for (Eigen::Index i = 0; i < tau.size(); ++i) {
    const double limit = m_joint_velocity_limits(i);
    if (limit <= 0.0) {
      continue; // No limit configured for this joint
    }

    const double vel = Base::m_joint_velocities(i);
    const double abs_vel = std::abs(vel);
    const double buffer_start = (1.0 - kBufferRatio) * limit;
    const double tau_before = tau(i);

    if (abs_vel > limit) {
      // Hard clamp: joint exceeded the limit — override with braking torque
      if (vel > 0.0) {
        tau(i) = std::min(tau(i), -m_velocity_limit_damping * (vel - limit));
      } else {
        tau(i) = std::max(tau(i), -m_velocity_limit_damping * (vel + limit));
      }
    } else if (abs_vel > buffer_start) {
      // Soft buffer zone: smoothly ramp up a braking torque as velocity
      // approaches the limit. Uses smoothstep (3t²-2t³) for C1 continuity.
      const double t = (abs_vel - buffer_start) / (limit - buffer_start);
      const double alpha = t * t * (3.0 - 2.0 * t);
      const double sign = (vel > 0.0) ? 1.0 : -1.0;
      tau(i) -=
          alpha * m_velocity_limit_damping * sign * (abs_vel - buffer_start);
    }

    tau_vel_limit(i) = tau(i) - tau_before;
  }

  return tau_vel_limit;
}

ctrl::VectorND CombinedImpedanceController::computeTorque(double dt) {
  // computeJointTrajectoryTaskTorque (which is called with m_traj_mutex held).
  // Lock ordering: m_mode_heartbeat_mutex first, then m_traj_mutex.
  if (m_control_mode.load() == ControlMode::JOINT_TRAJECTORY) {
    bool mode_hb_timed_out = false;
    {
      std::lock_guard<std::mutex> mhb_lock(m_mode_heartbeat_mutex);
      if (m_mode_heartbeat_received.load()) {
        const auto time = get_node()->get_clock()->now();
        const double mode_hb_diff =
            (time - m_last_mode_heartbeat_time).seconds();
        if (mode_hb_diff > kModeHeartbeatTimeout) {
          mode_hb_timed_out = true;
        }
      }
    }
    if (mode_hb_timed_out) {
      std::lock_guard<std::mutex> tlock(m_traj_mutex);
      m_control_mode.store(ControlMode::CARTESIAN);
      m_mode_heartbeat_received.store(false);
      RCLCPP_ERROR(get_node()->get_logger(),
                   "CombinedImpedanceController: Mode heartbeat timeout. "
                   "Auto-switching to CARTESIAN.");
      freezeDesiredPoses();
    }
  }

  RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                        2000, "Compute torque in mode: %s",
                        m_control_mode.load() == ControlMode::JOINT_TRAJECTORY
                            ? "JOINT"
                            : "CARTESIAN");

  // Log average update frequency every 5 seconds
  {
    const auto now = get_node()->get_clock()->now();
    if (!m_freq_initialized) {
      m_freq_last_report_time = now;
      m_freq_call_count = 0;
      m_freq_initialized = true;
    }
    ++m_freq_call_count;
    const double elapsed = (now - m_freq_last_report_time).seconds();
    if (elapsed >= 5.0) {
      const double avg_hz = m_freq_call_count / elapsed;
      RCLCPP_INFO(get_node()->get_logger(),
                  "computeTorque avg update rate: %.1f Hz", avg_hz);
      m_freq_call_count = 0;
      m_freq_last_report_time = now;
    }
  }

  // Redefine joints velocities in Eigen format
  ctrl::VectorND q = Base::m_joint_positions.data;
  ctrl::VectorND q_dot = Base::m_joint_velocities.data;

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Compute the jacobian
  Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions,
                                      Base::m_jacobian);

  ctrl::MatrixND jac = Base::m_jacobian.data;

  KDL::JntSpaceInertiaMatrix M(Base::m_joint_number);
  m_dyn_solver->JntToMass(Base::m_joint_positions, M);
  ctrl::Matrix6D Lambda = (jac * M.data.inverse() * jac.transpose()).inverse();

  KDL::Frame target_frame_snapshot;
  ctrl::Vector6D target_wrench_snapshot;
  ctrl::Vector6D ft_sensor_wrench_snapshot;
  {
    std::lock_guard<std::mutex> lock(m_input_mutex);
    target_frame_snapshot = m_target_frame;
    target_wrench_snapshot = m_target_wrench;
    ft_sensor_wrench_snapshot = m_ft_sensor_wrench;
  }

  // Initialize the torque vectors
  ctrl::VectorND tau_null(Base::m_joint_number), tau_ext(Base::m_joint_number),
      tau(Base::m_joint_number);

  // init tau to zero
  tau.setZero();
  tau_ext.setZero();
  tau_null.setZero();

  {
    std::lock_guard<std::mutex> lock(m_traj_mutex);
    if (m_control_mode.load() == ControlMode::JOINT_TRAJECTORY) {
      const auto tau_joint = computeJointTrajectoryTaskTorque(q_dot);
      tau += tau_joint;

      // Apply optional blending
      if (m_blend_active) {
        // Compute the blending feedforward torque and apply blending if needed
        const double blend_gain =
            std::exp(-m_blend_elapsed / kBlendTimeConstant);
        tau += blend_gain * (m_blend_tau_ff - tau_joint);

        m_blend_elapsed += dt;
        if (m_blend_elapsed >
            5.0 * kBlendTimeConstant) { // ~250 ms -- gain < 0.7%
          m_blend_active = false;
          RCLCPP_INFO(get_node()->get_logger(), "Torque blend complete.");
        }
      }
    } else if (m_control_mode.load() == ControlMode::CARTESIAN) {
      const auto tau_task = computeCartesianTaskTorque(
          jac, q_dot, Lambda, target_frame_snapshot, dt);
      // Save the last task torque for blending
      m_last_tau_task = tau_task;
      tau += tau_task;
    } else {
      // Don't throw in a real-time control loop; fail safe instead.
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unknown control mode! Sending zero torques.");
      tau.setZero();
    }
  }
  KDL::JntArray tau_coriolis(Base::m_joint_number);
  if (m_compensate_coriolis) {
    Base::m_dyn_solver->JntToCoriolis(Base::m_joint_positions,
                                      Base::m_joint_velocities, tau_coriolis);
    tau += tau_coriolis.data;
  }
  // Computes the Jacobian derivative * q_dot, negligible for most of the robot
  if (m_compensate_dJdq) {
    KDL::JntArrayVel q_in(Base::m_joint_positions, Base::m_joint_velocities);
    KDL::Twist jac_dot_q_dot;
    Base::m_jnt_to_jac_dot_solver->JntToJacDot(q_in, jac_dot_q_dot);
    // convert KDL::Twist to Eigen::VectorXd
    Eigen::VectorXd jac_dot_q_dot_eigen(6);
    jac_dot_q_dot_eigen.head(3) << jac_dot_q_dot.vel.x(), jac_dot_q_dot.vel.y(),
        jac_dot_q_dot.vel.z();
    jac_dot_q_dot_eigen.tail(3) << jac_dot_q_dot.rot.x(), jac_dot_q_dot.rot.y(),
        jac_dot_q_dot.rot.z();
    tau += jac.transpose() * Lambda * jac_dot_q_dot_eigen;
  }

  // In JOINT_TRAJECTORY mode, the Cartesian null-space projector conflicts
  // with joint-space PD control.
  if (m_control_mode.load() == ControlMode::CARTESIAN &&
      m_null_space_stiffness > 1e-6) {
    // Compute dynamically consistent null space projector
    tau_null =
        (m_identity - jac.transpose() * Lambda * jac * M.data.inverse()) *
        (m_null_space_stiffness * (-q + m_q_ns) - m_null_space_damping * q_dot);
  }

  double k_p = 1.0;
  // Compute the torque to achieve the desired force
  if (target_wrench_snapshot.norm() > 0.1) {
    tau_ext = jac.transpose() *
              (target_wrench_snapshot +
               k_p * (target_wrench_snapshot + ft_sensor_wrench_snapshot));
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "External wrench desired: \n"
            << target_wrench_snapshot << "\n"
            << "Measured wrench: \n"
            << ft_sensor_wrench_snapshot << "\n"
            << "Torque ext: \n"
            << tau_ext.transpose() << "\n"
            << "Feedforward target wrench: \n"
            << (target_wrench_snapshot +
                k_p * (target_wrench_snapshot + ft_sensor_wrench_snapshot))
                   .transpose()
            << "\n");
  }
  // Sum up remaining torques
  // tau += tau_null + tau_ext;

  return tau;
}

void CombinedImpedanceController::targetWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  auto w = toVector6D(wrench->wrench);
  if (wrench->header.frame_id != Base::m_robot_base_link) {
    w = Base::displayInBaseLink(w, wrench->header.frame_id);
  }
  std::lock_guard<std::mutex> lock(m_input_mutex);
  m_target_wrench = w;
}

void CombinedImpedanceController::ftSensorWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  auto v = toVector6D(wrench->wrench);

  if (v.hasNaN()) {
    RCLCPP_WARN_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 3000,
        "NaN detected in force-torque sensor wrench. Ignoring input.");
    return;
  }

  if (wrench->header.frame_id != Base::m_robot_base_link) {
    v = Base::displayInBaseLink(v, wrench->header.frame_id);
  }

  std::lock_guard<std::mutex> lock(m_input_mutex);
  m_ft_sensor_wrench = v;
}

void CombinedImpedanceController::targetFrameCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr target) {
  if (m_robot_monitor->controllerState() !=
      RobotMonitor::ControllerState::RUNNING) {
    return; // Don't accept new poses while in a non-normal state
  }
  if (m_control_mode.load() != ControlMode::CARTESIAN) {
    return; // Don't accept new poses if not in cartesian mode
  }

  if (target->header.frame_id != Base::m_robot_base_link) {
    auto &clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), clock, 3000,
        "Got target pose in wrong reference frame. Expected: %s but got %s",
        Base::m_robot_base_link.c_str(), target->header.frame_id.c_str());
    return;
  }

  auto frame =
      KDL::Frame(KDL::Rotation::Quaternion(
                     target->pose.orientation.x, target->pose.orientation.y,
                     target->pose.orientation.z, target->pose.orientation.w),
                 KDL::Vector(target->pose.position.x, target->pose.position.y,
                             target->pose.position.z));

  // Reset integral when the goal changes significantly to avoid torque bias
  // from the previous target causing overshoot toward the new one.
  const KDL::Vector dp = frame.p - m_prev_target_frame.p;
  const double lin_change = dp.Norm();
  // Rotation change as angle of the relative rotation
  const KDL::Rotation dR = m_prev_target_frame.M.Inverse() * frame.M;
  const KDL::Vector rot_axis = dR.GetRot(); // axis * angle (Rodrigues)
  const double rot_change = rot_axis.Norm();
  if (lin_change > m_integral_goal_change_threshold_lin ||
      rot_change > m_integral_goal_change_threshold_rot) {
    m_cart_motion_error_integral = ctrl::Vector6D::Zero();
  }
  m_prev_target_frame = frame;

  std::lock_guard<std::mutex> lock(m_input_mutex);
  m_target_frame = frame;
}

// was silently discarded.
void CombinedImpedanceController::modeSwitchCallback(
    std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {

  // Lock ordering: always m_mode_heartbeat_mutex before m_traj_mutex (or
  // acquire them sequentially, never nest the other way) to match the
  // ordering in computeTorque() and avoid deadlocks.

  if (req->data) {
    {
      std::lock_guard<std::mutex> lock(m_traj_mutex);
      // Request JOINT_TRAJECTORY
      if (m_control_mode.load() == ControlMode::JOINT_TRAJECTORY) {
        res->success = true;
        res->message = "Already in JOINT_TRAJECTORY mode.";
        RCLCPP_INFO(get_node()->get_logger(), "modeSwitchCallback done.");
        return;
      }
      m_blend_tau_ff = m_last_tau_task;
      RCLCPP_INFO_STREAM(get_node()->get_logger(),
                         "m_last_tau_task: " << m_last_tau_task.transpose()
                                             << "\n");
      m_blend_elapsed = 0.0;
      m_blend_active = true;
      m_traj_active =
          false; // No trajectory yet -- just holding current position.
      freezeDesiredPoses();
      // Switch CARTESIAN -> JOINT_TRAJECTORY
      m_control_mode.store(ControlMode::JOINT_TRAJECTORY);
    }
    // Start the watchdog clock (m_traj_mutex released first to respect
    // ordering).
    {
      std::lock_guard<std::mutex> hb_lock(m_mode_heartbeat_mutex);
      m_last_mode_heartbeat_time = get_node()->get_clock()->now();
      m_mode_heartbeat_received.store(true);
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "CombinedImpedanceController: Switched to JOINT_TRAJECTORY "
                "mode (via service).");
    res->success = true;
    res->message = "Switched to JOINT_TRAJECTORY mode.";
  } else {
    std::lock_guard<std::mutex> lock(m_traj_mutex);
    // Switch JOINT -> CARTESIAN
    m_control_mode.store(ControlMode::CARTESIAN);
    m_mode_heartbeat_received.store(false);
    RCLCPP_INFO(get_node()->get_logger(),
                "CombinedImpedanceController: Switched to CARTESIAN mode "
                "(via service).");
    freezeDesiredPoses();
    res->success = true;
    res->message = "Switched to CARTESIAN mode.";
  }
}

void CombinedImpedanceController::jointTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  // Empty trajectory = cancel active trajectory (hold current joint positions).
  // Mode switching is handled by the controller_mode_switch service.
  if (msg->points.empty()) {
    std::lock_guard<std::mutex> lock(m_traj_mutex);
    RCLCPP_INFO(get_node()->get_logger(),
                "CombinedImpedanceController: Cancelling active trajectory.");
    return;
  }

  // Reject trajectory if not in JOINT_TRAJECTORY mode
  {
    std::lock_guard<std::mutex> lock(m_traj_mutex);
    if (m_control_mode.load() != ControlMode::JOINT_TRAJECTORY) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "CombinedImpedanceController: Trajectory rejected "
                  "-- not in JOINT_TRAJECTORY mode.");
      return;
    }
  }

  // Build an index map from this controller's joints to the message joints.
  // For each joint in controller_joint_names we find the matching index in
  // msg->joint_names so we can extract the correct positions.
  const auto controller_joint_names = Base::m_joint_names;
  std::vector<int> msg_index_for_controller_joint(Base::m_joint_number, -1);
  for (size_t i = 0; i < controller_joint_names.size(); ++i) {
    for (size_t j = 0; j < msg->joint_names.size(); ++j) {
      if (msg->joint_names[j] == controller_joint_names[i]) {
        msg_index_for_controller_joint[i] = static_cast<int>(j);
        break;
      }
    }
    if (msg_index_for_controller_joint[i] < 0) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "CombinedImpedanceController: Joint '%s' not found in "
                  "trajectory message. Ignoring trajectory.",
                  controller_joint_names[i].c_str());
      return;
    }
  }

  std::lock_guard<std::mutex> lock(m_traj_mutex);
  const size_t n = msg->points.size();
  m_traj_positions.resize(n);
  m_traj_velocities.resize(n);
  m_traj_times.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const auto &pt = msg->points[i];

    if (pt.positions.size() < msg->joint_names.size()) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "CombinedImpedanceController: Trajectory point %zu has "
                  "insufficient positions (%zu < %zu). Ignoring trajectory.",
                  i, pt.positions.size(), msg->joint_names.size());
      m_traj_active = false;
      return;
    }

    const bool has_vel = pt.velocities.size() >= msg->joint_names.size();
    m_traj_times[i] = rclcpp::Duration(pt.time_from_start).seconds();
    m_traj_positions[i] = ctrl::VectorND::Zero(Base::m_joint_number);
    m_traj_velocities[i] = ctrl::VectorND::Zero(Base::m_joint_number);
    for (size_t j = 0; j < Base::m_joint_number; ++j) {
      m_traj_positions[i](j) = pt.positions[msg_index_for_controller_joint[j]];
      m_traj_velocities[i](j) =
          has_vel ? pt.velocities[msg_index_for_controller_joint[j]] : 0.0;
    }
  }

  // Seed desired positions from the ACTUAL robot state rather than from
  // m_traj_positions[0]. m_traj_positions[0] comes from the Python-side
  // joint-state snapshot which was taken several milliseconds before this
  // callback fires (planning time + bridge latency). Seeding from the live
  // hardware state guarantees zero initial PD error, eliminating the torque
  // spike (and resulting jerk) that would otherwise scale with the P gain.
  m_desired_joint_positions = Base::m_joint_positions.data;
  m_desired_joint_velocities = ctrl::VectorND::Zero(Base::m_joint_number);
  m_traj_elapsed = 0.0;
  m_traj_active = true;

  RCLCPP_INFO(get_node()->get_logger(),
              "CombinedImpedanceController: Accepted trajectory with %zu "
              "points. Target set to last waypoint.",
              msg->points.size());
}

void CombinedImpedanceController::modeHeartbeatCallback(
    const std_msgs::msg::Empty::SharedPtr /*msg*/) {
  std::lock_guard<std::mutex> lock(m_mode_heartbeat_mutex);
  m_last_mode_heartbeat_time = get_node()->get_clock()->now();

  if (!m_mode_heartbeat_received.load()) {
    m_mode_heartbeat_received.store(true);
  }
}

} // namespace combined_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    combined_impedance_controller::CombinedImpedanceController,
    controller_interface::ControllerInterface)
