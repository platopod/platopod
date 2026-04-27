"""Launch file for the OPFOR node — autonomous virtual hostile units."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("exercise_file", default_value=""),
        DeclareLaunchArgument("tick_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("default_team", default_value="red"),

        Node(
            package="plato_pod",
            executable="opfor_node",
            name="opfor_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
                "tick_rate_hz": LaunchConfiguration("tick_rate_hz"),
                "default_team": LaunchConfiguration("default_team"),
            }],
            output="screen",
        ),
    ])
