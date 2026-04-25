"""Launch file for the Plato Pod registry and robot bridge nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("udp_port", default_value="9999"),
        DeclareLaunchArgument("heartbeat_interval_sec", default_value="2.0"),
        DeclareLaunchArgument("heartbeat_timeout_sec", default_value="6.0"),
        DeclareLaunchArgument("default_radius", default_value="0.028"),
        DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.3"),
        DeclareLaunchArgument("max_angular_speed", default_value="2.0"),

        Node(
            package="plato_pod",
            executable="registry_node",
            name="registry_node",
            parameters=[{
                "default_radius": LaunchConfiguration("default_radius"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            }],
            output="screen",
        ),

        Node(
            package="plato_pod",
            executable="robot_bridge_node",
            name="robot_bridge_node",
            parameters=[{
                "udp_port": LaunchConfiguration("udp_port"),
                "heartbeat_interval_sec": LaunchConfiguration("heartbeat_interval_sec"),
                "heartbeat_timeout_sec": LaunchConfiguration("heartbeat_timeout_sec"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_speed": LaunchConfiguration("max_angular_speed"),
            }],
            output="screen",
        ),
    ])
