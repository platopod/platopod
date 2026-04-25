"""Launch file for the Plato Pod API gateway node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.2"),
        DeclareLaunchArgument("max_angular_speed", default_value="2.0"),
        DeclareLaunchArgument("watchdog_timeout_ms", default_value="500"),
        DeclareLaunchArgument("control_timeout_sec", default_value="30"),
        DeclareLaunchArgument("pose_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("lookahead_dt", default_value="0.1"),

        Node(
            package="plato_pod",
            executable="api_gateway_node",
            name="api_gateway_node",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                "watchdog_timeout_ms": LaunchConfiguration("watchdog_timeout_ms"),
                "control_timeout_sec": LaunchConfiguration("control_timeout_sec"),
                "pose_rate_hz": LaunchConfiguration("pose_rate_hz"),
                "lookahead_dt": LaunchConfiguration("lookahead_dt"),
            }],
            output="screen",
        ),
    ])
