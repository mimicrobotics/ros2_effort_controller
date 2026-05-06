from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("dual_arm_coordinator"), "config", "dual_arm.yaml"]
        ),
        description="Path to a coordinator parameter YAML.",
    )

    coordinator = Node(
        package="dual_arm_coordinator",
        executable="dual_arm_coordinator",
        name="dual_arm_coordinator",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, coordinator])
