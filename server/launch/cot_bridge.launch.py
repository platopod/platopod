"""Launch file for the Plato Pod CoT/ATAK bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("target_host", default_value="192.168.1.100"),
        DeclareLaunchArgument("target_port", default_value="6969"),
        DeclareLaunchArgument("transport", default_value="udp_unicast"),
        DeclareLaunchArgument("geo_origin_lat", default_value="-35.2975"),
        DeclareLaunchArgument("geo_origin_lon", default_value="149.1012"),
        DeclareLaunchArgument("geo_origin_alt", default_value="580.0"),
        DeclareLaunchArgument("geo_rotation_deg", default_value="0.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("stale_seconds", default_value="30.0"),
        DeclareLaunchArgument("arena_republish_seconds", default_value="60.0"),
        DeclareLaunchArgument("inbound_port", default_value="4242"),
        DeclareLaunchArgument("default_vehicle_role", default_value="recon"),
        DeclareLaunchArgument("scale_factor", default_value="1.0"),

        Node(
            package="plato_pod",
            executable="cot_bridge_node",
            name="cot_bridge_node",
            parameters=[{
                "target_host": LaunchConfiguration("target_host"),
                "target_port": LaunchConfiguration("target_port"),
                "transport": LaunchConfiguration("transport"),
                "geo_origin_lat": LaunchConfiguration("geo_origin_lat"),
                "geo_origin_lon": LaunchConfiguration("geo_origin_lon"),
                "geo_origin_alt": LaunchConfiguration("geo_origin_alt"),
                "geo_rotation_deg": LaunchConfiguration("geo_rotation_deg"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "stale_seconds": LaunchConfiguration("stale_seconds"),
                "arena_republish_seconds": LaunchConfiguration("arena_republish_seconds"),
                "inbound_port": LaunchConfiguration("inbound_port"),
                "default_vehicle_role": LaunchConfiguration("default_vehicle_role"),
                "scale_factor": LaunchConfiguration("scale_factor"),
            }],
            output="screen",
        ),
    ])
