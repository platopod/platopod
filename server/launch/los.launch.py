"""Launch file for the LoS service.

Pick a backend with the `backend` argument:
  * backend:=python  → los_python_node (default; lightweight, no Gazebo)
  * backend:=gazebo  → los_gazebo_node (requires Gazebo + ros_gz_bridge)

Both nodes serve the same `~/evaluate_los` service contract, so consumers
(engagement_node, sensor plugins) work with either.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context):
    backend = LaunchConfiguration("backend").perform(context)
    if backend == "gazebo":
        return [Node(
            package="plato_pod",
            executable="los_gazebo_node",
            name="los_gazebo_node",
            parameters=[{
                "laser_scan_topic": LaunchConfiguration("laser_scan_topic"),
            }],
            output="screen",
        )]
    return [Node(
        package="plato_pod",
        executable="los_python_node",
        name="los_python_node",
        output="screen",
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="python"),
        DeclareLaunchArgument(
            "laser_scan_topic", default_value="/world/los_probe/scan",
        ),
        OpaqueFunction(function=_make_node),
    ])
