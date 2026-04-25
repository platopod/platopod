"""Launch file for the Plato Pod vision node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("camera_device", default_value="0"),
        DeclareLaunchArgument("camera_width", default_value="1920"),
        DeclareLaunchArgument("camera_height", default_value="1080"),
        DeclareLaunchArgument("camera_fps", default_value="30"),
        DeclareLaunchArgument("calibration_file",
                              default_value="config/camera_calibration.yaml"),
        DeclareLaunchArgument("apriltag_config",
                              default_value="config/apriltag_settings.yaml"),
        DeclareLaunchArgument("debug_overlay", default_value="false"),
        DeclareLaunchArgument("mjpeg_quality", default_value="80"),
        DeclareLaunchArgument("mjpeg_port", default_value="8081"),

        Node(
            package="plato_pod",
            executable="vision_node",
            name="vision_node",
            parameters=[{
                "camera_device": LaunchConfiguration("camera_device"),
                "camera_width": LaunchConfiguration("camera_width"),
                "camera_height": LaunchConfiguration("camera_height"),
                "camera_fps": LaunchConfiguration("camera_fps"),
                "calibration_file": LaunchConfiguration("calibration_file"),
                "apriltag_config": LaunchConfiguration("apriltag_config"),
                "debug_overlay": LaunchConfiguration("debug_overlay"),
                "mjpeg_quality": LaunchConfiguration("mjpeg_quality"),
                "mjpeg_port": LaunchConfiguration("mjpeg_port"),
            }],
            output="screen",
        ),
    ])
