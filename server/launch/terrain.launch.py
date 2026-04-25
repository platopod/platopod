"""Launch file for Gazebo terrain mode with real DEM heightmaps.

Starts the full platform with Gazebo world from real terrain data,
sensor bridging, ghost models for physical robots, and ATAK output.

Usage:
    ros2 launch plato_pod terrain.launch.py \
      exercise_file:=/ros2_ws/config/exercises/adfa-terrain-patrol.yaml \
      target_host:=192.168.1.42 \
      scale_factor:=1000.0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_dir = os.path.dirname(os.path.abspath(__file__))

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("exercise_file", default_value=""),
        DeclareLaunchArgument("target_host", default_value="192.168.1.100"),
        DeclareLaunchArgument("target_port", default_value="4242"),
        DeclareLaunchArgument("scale_factor", default_value="1.0"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("model_path", default_value="models"),
        DeclareLaunchArgument("default_preset", default_value="full_suite"),
        DeclareLaunchArgument("geo_origin_lat", default_value="-35.2975"),
        DeclareLaunchArgument("geo_origin_lon", default_value="149.1012"),

        # Gazebo bridge (world from DEM terrain, sensor bridging, ghost models)
        Node(
            package="plato_pod",
            executable="gazebo_bridge_node",
            name="gazebo_bridge_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
                "headless": LaunchConfiguration("headless"),
                "model_path": LaunchConfiguration("model_path"),
                "bridge_sensors": True,
                "scale_factor": LaunchConfiguration("scale_factor"),
                "ghost_update_rate_hz": 10.0,
            }],
            output="screen",
        ),

        # Arena model
        Node(
            package="plato_pod",
            executable="arena_model_node",
            name="arena_model_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
            }],
            output="screen",
        ),

        # Registry + robot bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "registry.launch.py")
            ),
        ),

        # Sensor engine (Gazebo sensor source)
        Node(
            package="plato_pod",
            executable="sensor_engine_node",
            name="sensor_engine_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
                "default_preset": LaunchConfiguration("default_preset"),
                "sensor_source": "gazebo",
            }],
            output="screen",
        ),

        # API gateway
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "gateway.launch.py")
            ),
        ),

        # CoT/ATAK bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "cot_bridge.launch.py")
            ),
            launch_arguments={
                "target_host": LaunchConfiguration("target_host"),
                "target_port": LaunchConfiguration("target_port"),
                "scale_factor": LaunchConfiguration("scale_factor"),
                "geo_origin_lat": LaunchConfiguration("geo_origin_lat"),
                "geo_origin_lon": LaunchConfiguration("geo_origin_lon"),
            }.items(),
        ),
    ])
