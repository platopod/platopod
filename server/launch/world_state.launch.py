"""Launch file for world_state_node — single source of truth for the world."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("exercise_file", default_value=""),

        Node(
            package="plato_pod",
            executable="world_state_node",
            name="world_state_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
            }],
            output="screen",
        ),
    ])
