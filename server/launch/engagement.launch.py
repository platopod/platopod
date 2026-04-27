"""Launch file for the engagement node — central fire evaluator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("exercise_file", default_value=""),
        DeclareLaunchArgument("rng_seed", default_value="0"),

        Node(
            package="plato_pod",
            executable="engagement_node",
            name="engagement_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
                "rng_seed": LaunchConfiguration("rng_seed"),
            }],
            output="screen",
        ),
    ])
