"""Launch file for the Plato Pod virtual simulation node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("tick_rate_hz", default_value="50"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.2"),
        DeclareLaunchArgument("max_angular_speed", default_value="2.0"),
        DeclareLaunchArgument("acceleration_limit", default_value="0.0"),
        DeclareLaunchArgument("velocity_noise_stddev", default_value="0.0"),
        DeclareLaunchArgument("pose_drift_stddev", default_value="0.0"),

        Node(
            package="plato_pod",
            executable="virtual_sim_node",
            name="virtual_sim_node",
            parameters=[{
                "tick_rate_hz": LaunchConfiguration("tick_rate_hz"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                "acceleration_limit": LaunchConfiguration("acceleration_limit"),
                "velocity_noise_stddev": LaunchConfiguration("velocity_noise_stddev"),
                "pose_drift_stddev": LaunchConfiguration("pose_drift_stddev"),
            }],
            output="screen",
        ),
    ])
