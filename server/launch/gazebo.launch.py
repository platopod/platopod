"""Launch file for Gazebo simulation mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("exercise_file", default_value=""),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("physics_step_size", default_value="0.001"),
        DeclareLaunchArgument("real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("model_path", default_value="models"),

        # Gazebo bridge (generates world, launches gz-sim, bridges topics)
        Node(
            package="plato_pod",
            executable="gazebo_bridge_node",
            name="gazebo_bridge_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
                "headless": LaunchConfiguration("headless"),
                "physics_step_size": LaunchConfiguration("physics_step_size"),
                "real_time_factor": LaunchConfiguration("real_time_factor"),
                "model_path": LaunchConfiguration("model_path"),
            }],
            output="screen",
        ),

        # Arena model (unchanged — loads exercise YAML for boundary/obstacles)
        Node(
            package="plato_pod",
            executable="arena_model_node",
            name="arena_model_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
            }],
            output="screen",
        ),

        # Registry (unchanged)
        Node(
            package="plato_pod",
            executable="registry_node",
            name="registry_node",
            output="screen",
        ),

        # API Gateway (unchanged)
        Node(
            package="plato_pod",
            executable="api_gateway_node",
            name="api_gateway_node",
            output="screen",
        ),
    ])
