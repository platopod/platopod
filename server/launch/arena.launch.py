"""Launch file for the Plato Pod arena model node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("exercise_file", default_value=""),
        DeclareLaunchArgument("boundary_staleness_timeout", default_value="10.0"),
        DeclareLaunchArgument("circle_vertices", default_value="16"),
        DeclareLaunchArgument("origin_tag", default_value="101"),

        Node(
            package="plato_pod",
            executable="arena_model_node",
            name="arena_model_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
                "boundary_staleness_timeout": LaunchConfiguration(
                    "boundary_staleness_timeout"
                ),
                "circle_vertices": LaunchConfiguration("circle_vertices"),
                "origin_tag": LaunchConfiguration("origin_tag"),
            }],
            output="screen",
        ),
    ])
