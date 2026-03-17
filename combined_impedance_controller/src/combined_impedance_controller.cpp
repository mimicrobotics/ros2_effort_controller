#include <combined_impedance_controller/combined_impedance_controller.h>
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

  // Define upper limit for impedance forces
  auto_declare<double>("max_impedance_force", 70.0);

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

  tf_prefix = get_node()->get_parameter("tf_prefix").as_string();

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
    m_joint_damping(i, i) = 2.0 * std::sqrt(joint_stiffness[i]);
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
  // Set nullspace damping
  m_null_space_damping = 2 * sqrt(m_null_space_stiffness);

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

  // Subscribe to heartbeat topic
  m_heartbeat_subscriber = get_node()->create_subscription<std_msgs::msg::Bool>(
      "collision_detection_heartbeat", 1,
      std::bind(&CombinedImpedanceController::heartbeatCallback, this,
                std::placeholders::_1));

  // Publisher for robot mode
  m_robot_mode_publisher = get_node()->create_publisher<std_msgs::msg::Int32>(
      get_node()->get_name() + std::string("/robot_mode"), 1);

  // Service for controller mode switching (replaces topic-based mode command)
  mode_switch_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
      get_node()->get_name() + std::string("/controller_mode_switch"),
      std::bind(&CombinedImpedanceController::modeSwitchCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(get_node()->get_logger(),
              "CombinedImpedanceController: Mode switch service ready.");

  // Heartbeat subscriber for JOINT_TRAJECTORY watchdog
  mode_heartbeat_sub_ = get_node()->create_subscription<std_msgs::msg::Empty>(
      std::string("/mode_heartbeat"), 1,
      std::bind(&CombinedImpedanceController::modeHeartbeatCallback, this,
                std::placeholders::_1));

  // Get debug topics parameter and create publishers
  m_debug_topics = get_node()->get_parameter("debug_topics").as_bool();
  RCLCPP_INFO(get_node()->get_logger(), "Publishing debug topics: %d",
              m_debug_topics);

  if (m_debug_topics) {
    // Publish current target frame
    target_pose_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            get_node()->get_name() + std::string("/debug_target_frame"), 10);

    // Publish current end-effector frame
    current_pose_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            get_node()->get_name() + std::string("/debug_current_frame"), 10);

    // Publish clamped goal frame that the controller is actually trying to
    // achieve (after error clamping)
    next_goal_pose_pub_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            get_node()->get_name() + std::string("/debug_next_goal_frame"), 10);

    // Publish orientation error angle
    angle_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>(
        get_node()->get_name() + std::string("/debug_orientation_error_angle"),
        10);

    // Publish overall tau
    tau_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        get_node()->get_name() + std::string("/debug_tau"), 10);

    // Publish control mode
    control_mode_pub_ = get_node()->create_publisher<std_msgs::msg::Int32>(
        get_node()->get_name() + std::string("/debug_control_mode"), 10);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CombinedImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf =
      EffortControllerBase::state_interface_configuration();
  conf.names.emplace_back(tf_prefix + "gpio/robot_mode");
  conf.names.emplace_back(tf_prefix + "gpio/safety_mode");
  conf.names.emplace_back(tf_prefix + "gpio/program_running");
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
  m_desired_joint_positions_ = Base::m_joint_positions.data;
  m_desired_joint_velocities_ = ctrl::VectorND::Zero(Base::m_joint_number);
  m_blend_tau_ff_ = ctrl::VectorND::Zero(Base::m_joint_number);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");

  // Reset integral terms and target joints
  m_cart_motion_error_integral = ctrl::Vector6D::Zero();
  m_joint_motion_error_integral = ctrl::VectorND::Zero(Base::m_joint_number);

  m_target_wrench = ctrl::Vector6D::Zero();
  m_ft_sensor_wrench = ctrl::Vector6D::Zero();

  {
    std::lock_guard<std::mutex> lock(heartbeat_mutex);
    last_heartbeat_time = get_node()->get_clock()->now();
  }

  // initialize controller state
  controller_state = ControllerState::RUNNING;
  control_mode =
      ControlMode::CARTESIAN; // default to cartesian mode on activation
  mimic_robot_mode = MimicRobotMode::MOVE;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CombinedImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  // Stop drifting by sending zero joint velocities
  Base::computeJointEffortCmds(ctrl::Vector6D::Zero());
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

  // Update controller state based on heartbeat and collision detection
  updateControllerState();

  // Update current program / safety / arm state from the controller
  updateRobotState();

  // Compute the torque to applay at the joints
  ctrl::VectorND tau_tot = computeTorque();

  // Saturation of the torque
  Base::computeJointEffortCmds(tau_tot);

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();

  if (control_mode == ControlMode::JOINT_TRAJECTORY) {
    updateNextTrajectoryPoint(period);
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
      "Updating trajectory point. Elapsed time: %f seconds", traj_elapsed_);
  std::lock_guard<std::mutex> lock(traj_mutex_);
  if (!traj_active_ || controller_state != ControllerState::RUNNING) {
    return;
  }
  traj_elapsed_ += 0.6 * period.seconds();

  if (traj_elapsed_ >= traj_times_.back()) {
    // Trajectory complete — hold final waypoint
    m_desired_joint_positions_ = traj_positions_.back();
    m_desired_joint_velocities_ = ctrl::VectorND::Zero(Base::m_joint_number);
    traj_active_ = false;
    RCLCPP_INFO(get_node()->get_logger(),
                "CombinedImpedanceController: Homing trajectory complete, "
                "holding final position.");
  } else {
    // Find bounding segment: idx0 is the waypoint just before traj_elapsed_,
    // idx1 is the waypoint just after it.
    auto it =
        std::upper_bound(traj_times_.begin(), traj_times_.end(), traj_elapsed_);
    const size_t idx1 =
        static_cast<size_t>(std::distance(traj_times_.begin(), it));
    const size_t idx0 = idx1 - 1;
    const double alpha = (traj_elapsed_ - traj_times_[idx0]) /
                         (traj_times_[idx1] - traj_times_[idx0]);

    // Linear interpolation between the two bounding waypoints
    m_desired_joint_positions_ =
        traj_positions_[idx0] +
        alpha * (traj_positions_[idx1] - traj_positions_[idx0]);
    m_desired_joint_velocities_ =
        traj_velocities_[idx0] +
        alpha * (traj_velocities_[idx1] - traj_velocities_[idx0]);

    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         1000, "Setting waypoint:  %lu - %lu", idx0, idx1);
  }
}

void CombinedImpedanceController::updateControllerState() {
  rclcpp::Time current_last_heartbeat_time;
  bool initial_heartbeat_was_received = false;
  const auto time = get_node()->get_clock()->now();

  {
    // Read the flag and the time under the same lock to avoid race condition
    std::lock_guard<std::mutex> lock(heartbeat_mutex);
    current_last_heartbeat_time = last_heartbeat_time;
    initial_heartbeat_was_received =
        initial_heartbeat_received.load(); // atomic read
  }
  // Check heartbeat only if the initial one has been received
  if (initial_heartbeat_was_received) { // Use the value read under the lock
    double time_diff = (time - current_last_heartbeat_time).seconds();
    if (time_diff > 0.5 && is_safe.load()) {
      RCLCPP_INFO_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Heartbeat timed out. Setting controller to UNSAFE. Current time: "
          "%f, Last heartbeat: %f",
          time.seconds(), current_last_heartbeat_time.seconds());
      is_safe.store(false); // atomic write
      initial_heartbeat_received.store(false);
    }
  }
  if (controller_state == ControllerState::RUNNING) {
    if (!is_safe.load()) {
      controller_state = ControllerState::STOPPED;
      RCLCPP_INFO(
          get_node()->get_logger(),
          "Collision detected! Freezing current pose. Recycle e-stops and move "
          "arms into a non collision config to continue operation.");
    } else if (robot_mode != RobotMode::RUNNING) {
      controller_state = ControllerState::STOPPED;
      RCLCPP_INFO(get_node()->get_logger(), "Robot not running!");
    } else if (safety_mode != SafetyMode::NORMAL) {
      controller_state = ControllerState::STOPPED;
      RCLCPP_INFO(get_node()->get_logger(), "Safety mode not normal!");
    } else if (program_mode != ProgramMode::PLAYING) {
      controller_state = ControllerState::STOPPED;
      RCLCPP_INFO(get_node()->get_logger(), "Program not playing!");
    }
  }

  if (!is_safe.load() || robot_mode != RobotMode::RUNNING ||
      safety_mode != SafetyMode::NORMAL ||
      program_mode != ProgramMode::PLAYING) {
    freezeDesiredPoses();
    mimic_robot_mode = MimicRobotMode::USER_STOPPED;
  }

  // if  collision had occurred, we now enter a pending state to wait for
  // recovery to finish.
  if (is_safe.load() && controller_state == ControllerState::STOPPED) {
    controller_state = ControllerState::WAITING;
  }

  if (controller_state == ControllerState::WAITING && is_safe.load() &&
      robot_mode == RobotMode::RUNNING && safety_mode == SafetyMode::NORMAL &&
      program_mode == ProgramMode::PLAYING) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Robot in back in safe remote control state. Resuming...");
    controller_state = ControllerState::RUNNING;
    mimic_robot_mode = MimicRobotMode::MOVE;
  }

  // Publish mimic robot mode for high level components as well
  std_msgs::msg::Int32 mode_msg;
  mode_msg.data = static_cast<int>(mimic_robot_mode);
  m_robot_mode_publisher->publish(mode_msg);
}

void CombinedImpedanceController::updateRobotState() {
  const auto robot_mode_new = static_cast<RobotMode>(
      state_interfaces_[static_cast<uint32_t>(StateInterfaces::ROBOT_MODE)]
          .get_value());
  if (robot_mode_new != robot_mode) {
    robot_mode = robot_mode_new;
    RCLCPP_INFO(get_node()->get_logger(), "Robot mode switched to: %s",
                toString(robot_mode));
  }
  const auto safety_mode_new = static_cast<SafetyMode>(
      state_interfaces_[static_cast<uint32_t>(StateInterfaces::SAFETY_MODE)]
          .get_value());
  if (safety_mode_new != safety_mode) {
    safety_mode = safety_mode_new;
    RCLCPP_INFO(get_node()->get_logger(), "Safety mode switched to: %s",
                toString(safety_mode));
  }
  const auto program_mode_new = static_cast<ProgramMode>(
      state_interfaces_[static_cast<uint32_t>(StateInterfaces::PROGRAM_RUNNING)]
          .get_value());
  if (program_mode_new != program_mode) {
    program_mode = program_mode_new;
    RCLCPP_INFO(get_node()->get_logger(), "Program mode switched to: %s",
                toString(program_mode));
  }
}

void CombinedImpedanceController::freezeDesiredPoses() {
  // freeze arm pose with desired pose
  frozen_pose.pose = m_current_frame;
  m_target_frame = m_current_frame;
  m_cart_motion_error_integral = ctrl::Vector6D::Zero();
  m_desired_joint_positions_ = Base::m_joint_positions.data;
  m_blend_tau_ff_ = ctrl::VectorND::Zero(Base::m_joint_number);
  m_desired_joint_velocities_ = ctrl::VectorND::Zero(Base::m_joint_number);
  m_joint_motion_error_integral = ctrl::VectorND::Zero(Base::m_joint_number);
}

ctrl::Vector6D CombinedImpedanceController::computeCartMotionError() {
  // Compute the cartesian error between the current and the target frame
  KDL::Frame target_frame;
  if (controller_state == ControllerState::RUNNING) {
    target_frame = m_target_frame;
  } else { // STOPPED or WAITING, use frozen poses
    target_frame = frozen_pose.pose;
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
  distance = std::clamp(distance, -max_distance, max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle_clamped;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  ctrl::Vector6D error;
  error.head<3>() << error_kdl.p.x(), error_kdl.p.y(), error_kdl.p.z();
  error.tail<3>() << rot_axis(0), rot_axis(1), rot_axis(2);

  if (m_debug_topics) {
    KDL::Frame next_goal_frame;
    next_goal_frame.M =
        KDL::Rotation::Rot(rot_axis, rot_axis.Norm()) * m_current_frame.M;
    next_goal_frame.p = error_kdl.p + m_current_frame.p;

    // Publish the target frame, current frame, and next goal frame for
    // debugging
    target_pose_pub_->publish(toPoseStamped(
        target_frame, Base::m_robot_base_link, get_node()->now()));

    current_pose_pub_->publish(toPoseStamped(
        m_current_frame, Base::m_robot_base_link, get_node()->now()));

    next_goal_pose_pub_->publish(toPoseStamped(
        next_goal_frame, Base::m_robot_base_link, get_node()->now()));

    std_msgs::msg::Float64 angle_msg;
    angle_msg.data = angle;
    angle_pub_->publish(angle_msg);
  }

  return error;
}

ctrl::VectorND CombinedImpedanceController::computeJointMotionError() {
  // Compute the joint space error between the current and the target
  // configuration
  ctrl::VectorND error =
      m_desired_joint_positions_ - Base::m_joint_positions.data;

  // Clamp the error to avoid excessive torques
  const double max_joint_error = 0.3; // TODO: tune this parameter
  for (auto i = 0; i < error.size(); ++i) {
    error(i) = std::clamp(error(i), -max_joint_error, max_joint_error);
  }
  return error;
}

ctrl::VectorND CombinedImpedanceController::computeJointTrajectoryTaskTorque(
    const ctrl::VectorND &q_dot) {
  // ── Mode heartbeat watchdog ──────────────────────────────────────────
  // If in JOINT and heartbeat is lost for >200ms, auto-revert
  // to CARTESIAN. Lock ordering: mode_heartbeat_mutex_ first, then
  // traj_mutex_ (never nested the other way) to avoid deadlock.
  bool mode_hb_timed_out = false;
  {
    std::lock_guard<std::mutex> mhb_lock(mode_heartbeat_mutex_);
    if (mode_heartbeat_received_.load()) {
      const auto time = get_node()->get_clock()->now();
      const double mode_hb_diff = (time - last_mode_heartbeat_time_).seconds();
      if (mode_hb_diff > kModeHeartbeatTimeout) {
        mode_hb_timed_out = true;
      }
    }
  }
  if (mode_hb_timed_out) {
    std::lock_guard<std::mutex> tlock(traj_mutex_);
    control_mode = ControlMode::CARTESIAN;
    mode_heartbeat_received_.store(false);
    RCLCPP_ERROR(get_node()->get_logger(),
                 "CombinedImpedanceController: Mode heartbeat timeout. "
                 "Auto-switching to CARTESIAN.");
    freezeDesiredPoses();
  }

  // Compute the motion error
  const ctrl::VectorND motion_error = computeJointMotionError();
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
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
  const ctrl::VectorND damping_torque = D_d * (-q_dot);
  const ctrl::VectorND integral_torque = K_i * m_joint_motion_error_integral;

  // Compute the task torque
  return stiffness_torque + damping_torque;
}

ctrl::VectorND CombinedImpedanceController::computeCartesianTaskTorque(
    const ctrl::MatrixND &jac, const ctrl::VectorND &q_dot,
    const ctrl::Matrix6D &Lambda) {
  // Compute the motion error
  const ctrl::Vector6D motion_error = computeCartMotionError();

  // Compute the stiffness and damping in the base link
  const auto base_link_stiffness =
      Base::displayInBaseLink(m_cartesian_stiffness, Base::m_end_effector_link);

  const ctrl::Matrix6D K_d = base_link_stiffness;
  // Eigen::VectorXd damping_correction = 3.0 * Eigen::VectorXd::Ones(6);
  const ctrl::Matrix6D D_d =
      compute_correct_damping(Lambda, K_d, m_damping_ratio);
  const ctrl::Matrix6D K_i = m_cartesian_integral_gain;

  // Anti-windup: clamp the integral error to prevent excessive torques
  m_cart_motion_error_integral.head(3)
      << (m_cart_motion_error_integral.head(3) + 0.1 * motion_error.head(3))
             .cwiseMax(-0.1)
             .cwiseMin(0.1);
  m_cart_motion_error_integral.tail(3)
      << (m_cart_motion_error_integral.tail(3) + 0.1 * motion_error.tail(3))
             .cwiseMax(-0.05)
             .cwiseMin(0.05);

  const ctrl::Vector6D stiffness_torque =
      jac.transpose() * (K_d * motion_error);
  const ctrl::Vector6D damping_torque =
      jac.transpose() * (D_d * (-jac * q_dot));
  const ctrl::Vector6D integral_torque =
      jac.transpose() * (K_i * m_cart_motion_error_integral);

  // Compute the task torque
  return stiffness_torque + damping_torque + integral_torque;
}

ctrl::VectorND CombinedImpedanceController::computeTorque() {
  RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Compute torque in mode: %s, controller state: %s, robot mode: %s, "
      "safety mode: %s",
      control_mode == ControlMode::JOINT_TRAJECTORY ? "JOINT" : "CARTESIAN",
      controller_state == ControllerState::RUNNING ? "RUNNING" : "STOPPED",
      robot_mode == RobotMode::RUNNING ? "RUNNING" : "STOPPED",
      safety_mode == SafetyMode::NORMAL ? "SAFE" : "UNSAFE");
  // Redefine joints velocities in Eigen format
  ctrl::VectorND q = Base::m_joint_positions.data;
  ctrl::VectorND q_dot = Base::m_joint_velocities.data;
  ctrl::VectorND q_null_space(Base::m_joint_number);

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  debug_msg::msg::Debug debug_msg;

  // Compute the jacobian
  Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions,
                                      Base::m_jacobian);

  // Compute the pseudo-inverse of the jacobian
  ctrl::MatrixND jac = Base::m_jacobian.data;
  ctrl::MatrixND jac_tran_pseudo_inverse;

  pseudoInverse(jac.transpose(), &jac_tran_pseudo_inverse);

  KDL::JntSpaceInertiaMatrix M(Base::m_joint_number);
  m_dyn_solver->JntToMass(Base::m_joint_positions, M);
  ctrl::Matrix6D Lambda = (jac * M.data.inverse() * jac.transpose()).inverse();

  // Initialize the torque vectors
  ctrl::VectorND tau_null(Base::m_joint_number), tau_ext(Base::m_joint_number),
      tau(Base::m_joint_number);

  // init tau to zero
  tau.setZero();
  tau_ext.setZero();
  tau_null.setZero();

  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (control_mode == ControlMode::JOINT_TRAJECTORY) {
      const auto tau_joint = computeJointTrajectoryTaskTorque(q_dot);
      tau += tau_joint;

      // Apply optional blending
      if (blend_active_) {
        // Compute the blending feedforward torque and apply blending if needed
        const double blend_gain =
            std::exp(-blend_elapsed_ / kBlendTimeConstant);
        tau += blend_gain * (m_blend_tau_ff_ - tau_joint);

        RCLCPP_INFO_STREAM(get_node()->get_logger(),
                           << "Tau: " << tau.transpose() << "\n"
                           << "tau_joint: " << tau_joint.transpose() << "\n");

        // Advance blend timer after arm has been updated.
        blend_elapsed_ += 0.001; // 1 kHz control loop
        if (blend_elapsed_ >
            5.0 * kBlendTimeConstant) { // ~250 ms — gain < 0.7%
          blend_active_ = false;
          RCLCPP_INFO(get_node()->get_logger(), "Torque blend complete.");
        }
      }
    } else if (control_mode == ControlMode::CARTESIAN) {
      const auto tau_task = computeCartesianTaskTorque(jac, q_dot, Lambda);
      // Save the last task torque for blending
      m_last_tau_task = tau_task;
      tau += tau_task;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Unknown control mode!");
      throw std::runtime_error("Unknown control mode!");
    }
  }
  KDL::JntArray tau_coriolis(Base::m_joint_number),
      tau_gravity(Base::m_joint_number);
  if (m_compensate_coriolis) {
    Base::m_dyn_solver->JntToCoriolis(Base::m_joint_positions,
                                      Base::m_joint_velocities, tau_coriolis);
    tau += tau_coriolis.data;
  }
  // Computes the Jacobian derivative * q_dot, negligible for most of the robot
  Eigen::VectorXd j_tran_lambda_jdot_qdot =
      Eigen::VectorXd::Zero(Base::m_joint_number);
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
    Eigen::VectorXd j_tran_lambda_jdot_qdot =
        jac.transpose() * Lambda * jac_dot_q_dot_eigen;
    tau = tau + j_tran_lambda_jdot_qdot;
  }

  // Compute the null space torque
  if (m_null_space_stiffness > 1e-6) {
    // Compute dynamically consistent null space projector
    tau_null =
        (m_identity - jac.transpose() * Lambda * jac * M.data.inverse()) *
        (m_null_space_stiffness * (-q + m_q_ns) - m_null_space_damping * q_dot);
  } else {
    tau_null = ctrl::VectorND::Zero(Base::m_joint_number);
  }

  double k_p = 1.0;
  // Compute the torque to achieve the desired force
  if (m_target_wrench.norm() > 0.1) {
    tau_ext = jac.transpose() *
              (m_target_wrench + k_p * (m_target_wrench + m_ft_sensor_wrench));
    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "External wrench desired: \n"
            << m_target_wrench << "\n"
            << "Measured wrench: \n"
            << m_ft_sensor_wrench << "\n"
            << "Torque ext: \n"
            << tau_ext.transpose() << "\n"
            << "Feedforward target wrench: \n"
            << (m_target_wrench + k_p * (m_target_wrench + m_ft_sensor_wrench))
                   .transpose()
            << "\n");
  } else {
    tau_ext = ctrl::VectorND::Zero(Base::m_joint_number);
  }
  // Sum up remaining torques
  tau += tau_null + tau_ext;

  if (m_debug_topics && tau_pub_) {
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.assign(tau.data(), tau.data() + tau.size());
    tau_pub_->publish(tau_msg);
  }

  if (m_debug_topics && control_mode_pub_) {
    std_msgs::msg::Int32 mode_msg;
    mode_msg.data = static_cast<int32_t>(control_mode);
    control_mode_pub_->publish(mode_msg);
  }

  return tau;
}

void CombinedImpedanceController::targetWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  // Parse the target wrench
  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;

  // Check if the wrench is given in the base frame
  if (wrench->header.frame_id != Base::m_robot_base_link) {
    // Transform the wrench to the base frame
    m_target_wrench =
        Base::displayInBaseLink(m_target_wrench, wrench->header.frame_id);
  }
}

void CombinedImpedanceController::ftSensorWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {

  if (std::isnan(wrench->wrench.force.x) ||
      std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) ||
      std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) ||
      std::isnan(wrench->wrench.torque.z)) {
    auto &clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(
        get_node()->get_logger(), clock, 3000,
        "NaN detected in force-torque sensor wrench. Ignoring input.");
    return;
  }

  m_ft_sensor_wrench[0] = wrench->wrench.force.x;
  m_ft_sensor_wrench[1] = wrench->wrench.force.y;
  m_ft_sensor_wrench[2] = wrench->wrench.force.z;
  m_ft_sensor_wrench[3] = wrench->wrench.torque.x;
  m_ft_sensor_wrench[4] = wrench->wrench.torque.y;
  m_ft_sensor_wrench[5] = wrench->wrench.torque.z;

  // Check if the wrench is given in the base frame
  if (wrench->header.frame_id != Base::m_robot_base_link) {
    // Transform the wrench to the base frame
    m_ft_sensor_wrench =
        Base::displayInBaseLink(m_ft_sensor_wrench, wrench->header.frame_id);
  }
}

void CombinedImpedanceController::targetFrameCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr target) {
  if (controller_state != ControllerState::RUNNING) {
    return; // Don't accept new poses while in a non-normal state
  }
  if (control_mode != ControlMode::CARTESIAN) {
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

  m_target_frame =
      KDL::Frame(KDL::Rotation::Quaternion(
                     target->pose.orientation.x, target->pose.orientation.y,
                     target->pose.orientation.z, target->pose.orientation.w),
                 KDL::Vector(target->pose.position.x, target->pose.position.y,
                             target->pose.position.z));
}

void CombinedImpedanceController::heartbeatCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  bool is_now_safe = msg->data;

  {
    std::lock_guard<std::mutex> lock(heartbeat_mutex);
    last_heartbeat_time = get_node()->get_clock()->now();

    if (!initial_heartbeat_received.load()) { // atomic read
      initial_heartbeat_received.store(true); // atomic write
      RCLCPP_INFO(get_node()->get_logger(),
                  "Initial collision detection heartbeat received. "
                  "Controller operational.");
    }
  }

  // atomically set is_safe value and get the previous value back.
  bool was_safe = is_safe.exchange(is_now_safe);

  if (is_now_safe != was_safe) {
    if (is_now_safe) {
      RCLCPP_INFO(get_node()->get_logger(),
                  "Controller state changed to SAFE (no collision).");
    } else {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Controller state changed to UNSAFE (collision detected).");
    }
  }
}

bool CombinedImpedanceController::modeSwitchCallback(
    std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {

  // Lock ordering: always mode_heartbeat_mutex_ before traj_mutex_ (or
  // acquire them sequentially, never nest the other way) to match the
  // ordering in computeTorque() and avoid deadlocks.

  if (req->data) {
    {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      // Request JOINT_TRAJECTORY
      if (control_mode == ControlMode::JOINT_TRAJECTORY) {
        res->success = true;
        res->message = "Already in JOINT_TRAJECTORY mode.";
        RCLCPP_INFO(get_node()->get_logger(), "modeSwitchCallback done.");
        return true;
      }
      m_blend_tau_ff_ = m_last_tau_task;
      RCLCPP_INFO_STREAM(get_node()->get_logger(),
                         << "m_last_tau_task: " << m_last_tau_task.transpose()
                         << "\n");
      blend_elapsed_ = 0.0;
      blend_active_ = true;
      traj_active_ =
          false; // No trajectory yet — just holding current position.
      freezeDesiredPoses();
      // Switch CARTESIAN -> JOINT_TRAJECTORY
      control_mode = ControlMode::JOINT_TRAJECTORY;
    }
    // Start the watchdog clock (traj_mutex_ released first to respect
    // ordering).
    {
      std::lock_guard<std::mutex> hb_lock(mode_heartbeat_mutex_);
      last_mode_heartbeat_time_ = get_node()->get_clock()->now();
      mode_heartbeat_received_.store(true);
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "CombinedImpedanceController: Switched to JOINT_TRAJECTORY "
                "mode (via service).");
    res->success = true;
    res->message = "Switched to JOINT_TRAJECTORY mode.";
  } else {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    // Switch JOINT -> CARTESIAN
    control_mode = ControlMode::CARTESIAN;
    mode_heartbeat_received_.store(false);
    RCLCPP_INFO(get_node()->get_logger(),
                "CombinedImpedanceController: Switched to CARTESIAN mode "
                "(via service).");
    freezeDesiredPoses();
    res->success = true;
    res->message = "Switched to CARTESIAN mode.";
  }

  return true;
}

void CombinedImpedanceController::jointTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  // Empty trajectory = cancel active trajectory (hold current joint positions).
  // Mode switching is handled by the controller_mode_switch service.
  if (msg->points.empty()) {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    RCLCPP_INFO(get_node()->get_logger(),
                "CombinedImpedanceController: Cancelling active trajectory.");
    return;
  }

  // Reject trajectory if not in JOINT_TRAJECTORY mode
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (control_mode != ControlMode::JOINT_TRAJECTORY) {
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

  std::lock_guard<std::mutex> lock(traj_mutex_);
  const size_t n = msg->points.size();
  traj_positions_.resize(n);
  traj_velocities_.resize(n);
  traj_times_.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const auto &pt = msg->points[i];
    const bool has_vel = pt.velocities.size() >= msg->joint_names.size();
    traj_times_[i] = rclcpp::Duration(pt.time_from_start).seconds();
    traj_positions_[i] = ctrl::VectorND::Zero(Base::m_joint_number);
    traj_velocities_[i] = ctrl::VectorND::Zero(Base::m_joint_number);
    for (size_t j = 0; j < Base::m_joint_number; ++j) {
      traj_positions_[i](j) = pt.positions[msg_index_for_controller_joint[j]];
      traj_velocities_[i](j) =
          has_vel ? pt.velocities[msg_index_for_controller_joint[j]] : 0.0;
    }
  }

  // Seed desired positions from the ACTUAL robot state rather than from
  // traj_positions_[0]. traj_positions_[0] comes from the Python-side
  // joint-state snapshot which was taken several milliseconds before this
  // callback fires (planning time + bridge latency). Seeding from the live
  // hardware state guarantees zero initial PD error, eliminating the torque
  // spike (and resulting jerk) that would otherwise scale with the P gain.
  m_desired_joint_positions_ = Base::m_joint_positions.data;
  m_desired_joint_velocities_ = ctrl::VectorND::Zero(Base::m_joint_number);
  traj_elapsed_ = 0.0;
  traj_active_ = true;

  RCLCPP_INFO(get_node()->get_logger(),
              "CombinedImpedanceController: Accepted trajectory with %zu "
              "points. Target set to last waypoint.",
              msg->points.size());
}

void CombinedImpedanceController::modeHeartbeatCallback(
    const std_msgs::msg::Empty::SharedPtr /*msg*/) {
  std::lock_guard<std::mutex> lock(mode_heartbeat_mutex_);
  last_mode_heartbeat_time_ = get_node()->get_clock()->now();

  if (!mode_heartbeat_received_.load()) {
    mode_heartbeat_received_.store(true);
  }
}

const char *CombinedImpedanceController::toString(RobotMode mode) {
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

const char *CombinedImpedanceController::toString(SafetyMode mode) {
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

const char *CombinedImpedanceController::toString(ProgramMode mode) {
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
} // namespace combined_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    combined_impedance_controller::CombinedImpedanceController,
    controller_interface::ControllerInterface)
