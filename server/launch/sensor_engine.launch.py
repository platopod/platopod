"""Launch file for the Plato Pod sensor engine node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("tick_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("default_preset", default_value="minimal"),
        DeclareLaunchArgument("exercise_file", default_value=""),
        DeclareLaunchArgument("sensor_source", default_value="python"),

        Node(
            package="plato_pod",
            executable="sensor_engine_node",
            name="sensor_engine_node",
            parameters=[{
                "tick_rate_hz": LaunchConfiguration("tick_rate_hz"),
                "default_preset": LaunchConfiguration("default_preset"),
                "exercise_file": LaunchConfiguration("exercise_file"),
                "sensor_source": LaunchConfiguration("sensor_source"),
            }],
            output="screen",
        ),
    ])
