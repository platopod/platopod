"""Launch file for Plato Pod simulation — mode-switching entry point.

Selects between simulation modes based on the `mode` parameter.

Usage:
    ros2 launch plato_pod simulation.launch.py mode:=lightweight
    ros2 launch plato_pod simulation.launch.py mode:=gazebo
    ros2 launch plato_pod simulation.launch.py mode:=gazebo_terrain \
      exercise_file:=/ros2_ws/config/exercises/adfa-terrain-patrol.yaml
    ros2 launch plato_pod simulation.launch.py mode:=replay \
      exercise_file:=/ros2_ws/config/exercises/replay-exercise.yaml

Modes:
    lightweight    — Python kinematics at 50 Hz (default, no GPU needed)
    gazebo         — Gazebo flat arena with 3D physics
    gazebo_terrain — Gazebo with real DEM terrain heightmaps + sensor bridging
    replay         — GPS track replay on terrain
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    mode = LaunchConfiguration("mode")
    pkg_dir = os.path.dirname(os.path.abspath(__file__))

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="lightweight",
            description="Simulation mode: lightweight, gazebo, gazebo_terrain, or replay",
        ),
        DeclareLaunchArgument("exercise_file", default_value=""),
        DeclareLaunchArgument("target_host", default_value="192.168.1.100"),
        DeclareLaunchArgument("scale_factor", default_value="1.0"),

        # Lightweight mode: virtual_sim_node (Python kinematics)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "virtual_sim.launch.py")
            ),
            condition=IfCondition(
                PythonExpression(["'", mode, "' == 'lightweight'"])
            ),
        ),

        # Gazebo mode: flat arena with 3D physics
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "gazebo.launch.py")
            ),
            launch_arguments={
                "exercise_file": LaunchConfiguration("exercise_file"),
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", mode, "' == 'gazebo'"])
            ),
        ),

        # Gazebo terrain mode: real DEM heightmaps + sensors + ghosts + ATAK
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "terrain.launch.py")
            ),
            launch_arguments={
                "exercise_file": LaunchConfiguration("exercise_file"),
                "target_host": LaunchConfiguration("target_host"),
                "scale_factor": LaunchConfiguration("scale_factor"),
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", mode, "' == 'gazebo_terrain'"])
            ),
        ),

        # Replay mode: GPS track playback on terrain
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, "replay.launch.py")
            ),
            launch_arguments={
                "exercise_file": LaunchConfiguration("exercise_file"),
                "target_host": LaunchConfiguration("target_host"),
                "scale_factor": LaunchConfiguration("scale_factor"),
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", mode, "' == 'replay'"])
            ),
        ),
    ])
