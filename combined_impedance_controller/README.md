# Combined Impedance Controller

A ROS 2 control plugin that combines cartesian and joint-space impedance control for 6-DOF manipulators. It supports compliant end-effector pose tracking, force control via external wrenches, joint trajectory execution, and automatic mode switching — making it well suited for teleoperation with contacts.

Built on top of `effort_controller_base` and loaded as a `controller_interface::ControllerInterface` plugin.

## Control Modes

The controller operates in two mutually exclusive modes, switchable at runtime via a service call:

| Mode | Description |
|------|-------------|
| **CARTESIAN** (default) | PD + optional integral control in task space. Accepts a target end-effector pose and computes joint torques through the Jacobian. Includes error clamping and null-space configuration control. |
| **JOINT_TRAJECTORY** | PD + optional integral control in joint space. Accepts joint trajectories with interpolation between waypoints. Requires a heartbeat (`/mode_heartbeat`) and auto-reverts to CARTESIAN after a 300 ms timeout. |

Smooth blending (50 ms exponential decay) is applied when switching between modes.

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `~/target_frame` | `geometry_msgs/PoseStamped` | Target end-effector pose (cartesian mode) |
| `~/target_wrench` | `geometry_msgs/WrenchStamped` | Desired task-space force/torque |
| `~/ft_sensor_wrench` | `geometry_msgs/WrenchStamped` | Measured F/T sensor readings |
| `/target_joint_trajectory` | `trajectory_msgs/JointTrajectory` | Target joint trajectory (joint mode) |
| `/mode_heartbeat` | `std_msgs/Empty` | Watchdog heartbeat for joint trajectory mode |
| `collision_detection_heartbeat` | `std_msgs/Bool` | Collision detection status |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `~/data` | `debug_msg/Debug` | Main debug output |
| `~/data_impedance` | `std_msgs/Float64MultiArray` | Impedance control debug data |
| `~/robot_mode` | `std_msgs/Int32` | Current robot mode (from robot monitor) |

When `debug_topics` is enabled, additional topics are published:

| Topic | Type | Description |
|-------|------|-------------|
| `~/debug_target_frame` | `geometry_msgs/PoseStamped` | Target frame visualization |
| `~/debug_current_frame` | `geometry_msgs/PoseStamped` | Current end-effector frame |
| `~/debug_next_goal_frame` | `geometry_msgs/PoseStamped` | Clamped goal frame after error limiting |
| `~/debug_orientation_error_angle` | `std_msgs/Float64` | Orientation error magnitude |
| `~/debug_tau` | `std_msgs/Float64MultiArray` | Commanded joint torques |
| `~/debug_control_mode` | `std_msgs/Int32` | Active control mode (0 = CARTESIAN, 1 = JOINT_TRAJECTORY) |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `~/controller_mode_switch` | `std_srvs/SetBool` | `true` switches to JOINT_TRAJECTORY, `false` switches to CARTESIAN |

## Parameters

### Robot Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tf_prefix` | string | `""` | Prefix for TF frames |
| `end_effector_link` | string | — | End-effector link name |
| `robot_base_link` | string | — | Robot base link name |
| `ft_sensor_ref_link` | string | — | F/T sensor reference frame |
| `joints` | string[] | — | List of controlled joint names |

### Cartesian Impedance

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cart_stiffness.trans_x/y/z` | double | 500.0 | Translational stiffness (N/m) |
| `cart_stiffness.rot_x/y/z` | double | 50.0 | Rotational stiffness (Nm/rad) |
| `cart_integral_gain.trans_x/y/z` | double | 0.0 | Translational integral gain |
| `cart_integral_gain.rot_x/y/z` | double | 0.0 | Rotational integral gain |
| `cart_damping_ratio` | double | sqrt(2)/2 | Damping ratio (critically damped) |
| `max_impedance_force` | double | 70.0 | Maximum task force (N) |
| `hand_frame_control` | bool | true | Control in end-effector frame |

### Joint Impedance

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joint_stiffness` | double[] | — | Per-joint stiffness gains |
| `joint_integral_gain` | double[] | — | Per-joint integral gains |

### Null Space

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `nullspace_stiffness` | double | 0.0 | Null-space task stiffness |
| `nullspace_desired_configuration` | double[] | — | Target joint configuration for null space |

### Dynamics Compensation

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `compensate_dJdq` | bool | false | Compensate Jacobian derivative term |

### Debug & UR-Specific

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `debug_topics` | bool | false | Publish debug visualization topics |
| `ur_program_name` | string | `"ext_control.urp"` | UR program to auto-load |
| `dashboard_prefix` | string | `"/dashboard_client"` | UR dashboard service prefix |

## Example Configuration

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    combined_impedance_controller:
      type: combined_impedance_controller/CombinedImpedanceController

combined_impedance_controller:
  ros__parameters:
    end_effector_link: "tool0"
    robot_base_link: "base_link"
    ft_sensor_ref_link: "sensor_link"
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint

    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity

    cart_stiffness:
      trans_x: 500.0
      trans_y: 500.0
      trans_z: 500.0
      rot_x: 50.0
      rot_y: 50.0
      rot_z: 50.0

    cart_damping_ratio: 0.707

    joint_stiffness: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
    joint_integral_gain: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    nullspace_stiffness: 1.0
    nullspace_desired_configuration: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

    hand_frame_control: true
    compensate_dJdq: false
    debug_topics: false

    # UR-specific
    ur_program_name: "ext_control.urp"
    dashboard_prefix: "/dashboard_client"
```

## UR Robot Integration

The controller includes a `URRobotMonitor` that interfaces with the UR dashboard to handle recovery scenarios automatically:

- Monitors robot mode, safety mode, and program state via GPIO state interfaces
- Auto-closes safety popups and releases protective stops
- Restarts the external control program if it stops unexpectedly
- Freezes commanded poses during recovery to prevent jumps on resume

## Utilities

- **`scripts/plot_tau.py`** — Records `debug_tau` and `debug_control_mode` topics and plots joint torques over time with mode transitions marked. Requires `debug_topics: true`.

## Dependencies

- `rclcpp`, `controller_interface`, `effort_controller_base`
- `geometry_msgs`, `trajectory_msgs`, `std_msgs`, `std_srvs`
- `ur_dashboard_msgs`, `debug_msg`
- Eigen3 (>= 3.3)
- MatLogger2 (optional, enabled via CMake `LOGGING` option)
