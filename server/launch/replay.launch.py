"""Launch file for exercise replay mode.

Replays recorded GPS tracks on real terrain. Can drive desktop robots
along the recorded paths or run in pure simulation.

Usage:
    ros2 launch plato_pod replay.launch.py \
      exercise_file:=/ros2_ws/config/exercises/replay-exercise.yaml \
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
        DeclareLaunchArgument("exercise_file", default_value=""),
        DeclareLaunchArgument("target_host", default_value="192.168.1.100"),
        DeclareLaunchArgument("target_port", default_value="4242"),
        DeclareLaunchArgument("scale_factor", default_value="1000.0"),
        DeclareLaunchArgument("playback_speed", default_value="1.0"),
        DeclareLaunchArgument("geo_origin_lat", default_value="-35.2975"),
        DeclareLaunchArgument("geo_origin_lon", default_value="149.1012"),

        # Replay node (drives virtual robots along recorded GPS tracks)
        Node(
            package="plato_pod",
            executable="replay_node",
            name="replay_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
                "playback_speed": LaunchConfiguration("playback_speed"),
                "scale_factor": LaunchConfiguration("scale_factor"),
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

        # Registry
        Node(
            package="plato_pod",
            executable="registry_node",
            name="registry_node",
            output="screen",
        ),

        # Sensor engine (Python mode for replay)
        Node(
            package="plato_pod",
            executable="sensor_engine_node",
            name="sensor_engine_node",
            parameters=[{
                "exercise_file": LaunchConfiguration("exercise_file"),
                "sensor_source": "python",
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
