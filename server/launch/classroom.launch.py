"""Launch file for physical classroom exercises.

Starts all nodes needed for physical robots on a desktop arena with
AprilTag camera localisation. Optionally includes ATAK output.

This is the primary launch file for classroom use with real robots.

Usage:
    # Basic — camera + robots + dashboard
    ros2 launch plato_pod classroom.launch.py camera_device:=4

    # With exercise config
    ros2 launch plato_pod classroom.launch.py \
      camera_device:=4 \
      exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml

    # With ATAK output
    ros2 launch plato_pod classroom.launch.py \
      camera_device:=4 \
      exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml \
      target_host:=192.168.1.42
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
        DeclareLaunchArgument("camera_device", default_value="0"),
        DeclareLaunchArgument("exercise_file", default_value=""),
        DeclareLaunchArgument("target_host", default_value=""),
        DeclareLaunchArgument("default_preset", default_value="basic_scout"),

        # Vision node (AprilTag detection + MJPEG stream)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "vision.launch.py")
            ),
            launch_arguments={
                "camera_device": LaunchConfiguration("camera_device"),
                "debug_overlay": "true",
            }.items(),
        ),

        # Arena model (boundary from YAML or detected tags)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "arena.launch.py")
            ),
            launch_arguments={
                "exercise_file": LaunchConfiguration("exercise_file"),
            }.items(),
        ),

        # Registry + robot bridge (ESP32 UDP)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "registry.launch.py")
            ),
        ),

        # Virtual sim (for spawning virtual robots alongside physical ones)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "virtual_sim.launch.py")
            ),
        ),

        # Sensor engine
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "sensor_engine.launch.py")
            ),
            launch_arguments={
                "exercise_file": LaunchConfiguration("exercise_file"),
                "default_preset": LaunchConfiguration("default_preset"),
            }.items(),
        ),

        # API gateway (dashboard + WebSocket + REST)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "gateway.launch.py")
            ),
        ),
    ])
