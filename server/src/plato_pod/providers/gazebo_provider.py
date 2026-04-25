"""Gazebo localization provider.

Extracts robot poses from Gazebo model states via ros_gz_bridge.
Produces RobotPose objects with source=GAZEBO_SIM.
"""

from __future__ import annotations

import math
import time

from plato_pod.pose import PoseSource, RobotPose


class GazeboProvider:
    """Localization provider that reads poses from Gazebo simulation.

    Poses are received via ROS2 topics (bridged from Gazebo gz-transport
    by ros_gz_bridge) and stored. get_poses() returns the latest batch.
    """

    name = "gazebo"
    source = PoseSource.GAZEBO_SIM

    def __init__(self) -> None:
        self._poses: dict[int, RobotPose] = {}

    def update_pose(
        self,
        robot_id: int,
        x: float, y: float, z: float,
        qx: float, qy: float, qz: float, qw: float,
        localization_id: str = "",
    ) -> None:
        """Update a robot's pose from Gazebo quaternion data.

        Converts quaternion to 2D heading (yaw) for the platform's 2D model.

        Args:
            robot_id: Robot identifier.
            x: X position in metres.
            y: Y position in metres.
            z: Z position in metres (ignored for 2D).
            qx, qy, qz, qw: Orientation quaternion.
            localization_id: Provider-specific ID.
        """
        theta = quaternion_to_yaw(qx, qy, qz, qw)
        self._poses[robot_id] = RobotPose(
            robot_id=robot_id,
            x=x, y=y, theta=theta,
            timestamp=time.time(),
            source=PoseSource.GAZEBO_SIM,
            confidence=1.0,
            localization_id=localization_id or str(robot_id),
        )

    def get_poses(self) -> list[RobotPose]:
        """Return latest poses for all Gazebo-simulated robots."""
        return list(self._poses.values())

    def remove_robot(self, robot_id: int) -> None:
        """Remove a robot from tracking."""
        self._poses.pop(robot_id, None)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw (heading) angle in radians.

    Extracts the Z-axis rotation from a quaternion. Assumes the robot
    operates primarily in the XY plane.

    Args:
        qx, qy, qz, qw: Quaternion components.

    Returns:
        Yaw angle in radians [-pi, pi].
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)
