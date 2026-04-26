"""Launch file for full ATAK interoperability testing.

Starts all nodes needed for a complete exercise with CoT output:
  registry + robot_bridge → arena_model → virtual_sim → sensor_engine
  → api_gateway → cot_bridge

Usage:
  ros2 launch plato_pod atak_test.launch.py \
    target_host:=192.168.1.100 \
    exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml

Then spawn virtual robots via the dashboard or REST API:
  curl -X POST http://localhost:8080/robots/spawn \
    -H 'Content-Type: application/json' -d '{"x":0.4,"y":0.3}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("plato_pod")
    launch_dir = os.path.join(pkg_share, "launch")

    # Top-level arguments
    exercise_file_arg = DeclareLaunchArgument(
        "exercise_file", default_value="",
        description="Path to exercise YAML file",
    )
    target_host_arg = DeclareLaunchArgument(
        "target_host", default_value="192.168.1.100",
        description="ATAK device IP address",
    )
    target_port_arg = DeclareLaunchArgument(
        "target_port", default_value="4242",
        description="ATAK CoT receive port (4242 = ATAK default, 8087 = FreeTAKServer TCP)",
    )
    transport_arg = DeclareLaunchArgument(
        "transport", default_value="udp_unicast",
        description="CoT transport: udp_unicast, udp_multicast, or tcp",
    )
    geo_origin_lat_arg = DeclareLaunchArgument(
        "geo_origin_lat", default_value="-35.2975",
        description="Geographic origin latitude",
    )
    geo_origin_lon_arg = DeclareLaunchArgument(
        "geo_origin_lon", default_value="149.1012",
        description="Geographic origin longitude",
    )
    sensor_preset_arg = DeclareLaunchArgument(
        "default_preset", default_value="minimal",
        description="Default sensor preset for spawned robots",
    )
    gateway_port_arg = DeclareLaunchArgument(
        "gateway_port", default_value="8080",
        description="API gateway port (change if 8080 conflicts with FreeTAKServer)",
    )
    inbound_port_arg = DeclareLaunchArgument(
        "inbound_port", default_value="4242",
        description="UDP port to listen for inbound CoT events from ATAK",
    )
    scale_factor_arg = DeclareLaunchArgument(
        "scale_factor", default_value="1.0",
        description="Arena scale factor (1.0=outdoor, 1000.0=classroom)",
    )

    # Registry + robot bridge
    registry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "registry.launch.py")
        ),
    )

    # Arena model
    arena = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "arena.launch.py")
        ),
        launch_arguments={
            "exercise_file": LaunchConfiguration("exercise_file"),
        }.items(),
    )

    # Virtual simulation (lightweight mode)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "virtual_sim.launch.py")
        ),
    )

    # Sensor engine
    sensor_engine = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sensor_engine.launch.py")
        ),
        launch_arguments={
            "exercise_file": LaunchConfiguration("exercise_file"),
            "default_preset": LaunchConfiguration("default_preset"),
        }.items(),
    )

    # API gateway
    gateway = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "gateway.launch.py")
        ),
        launch_arguments={
            "port": LaunchConfiguration("gateway_port"),
        }.items(),
    )

    # CoT/ATAK bridge
    cot_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "cot_bridge.launch.py")
        ),
        launch_arguments={
            "target_host": LaunchConfiguration("target_host"),
            "target_port": LaunchConfiguration("target_port"),
            "transport": LaunchConfiguration("transport"),
            "geo_origin_lat": LaunchConfiguration("geo_origin_lat"),
            "geo_origin_lon": LaunchConfiguration("geo_origin_lon"),
            "scale_factor": LaunchConfiguration("scale_factor"),
            "exercise_file": LaunchConfiguration("exercise_file"),
            "inbound_port": LaunchConfiguration("inbound_port"),
        }.items(),
    )

    return LaunchDescription([
        exercise_file_arg,
        target_host_arg,
        target_port_arg,
        transport_arg,
        geo_origin_lat_arg,
        geo_origin_lon_arg,
        sensor_preset_arg,
        scale_factor_arg,
        gateway_port_arg,
        inbound_port_arg,
        registry,
        arena,
        simulation,
        sensor_engine,
        gateway,
        cot_bridge,
    ])
