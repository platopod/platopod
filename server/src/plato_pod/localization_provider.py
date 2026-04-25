"""Pluggable localization provider interface.

Each provider implements the LocalizationProvider protocol, producing
RobotPose objects from its specific sensing mechanism (camera + AprilTags,
GPS + IMU, RF anchors, etc.). The rest of the platform consumes only
RobotPose and never knows which provider produced it.
"""

from __future__ import annotations

from typing import Protocol

from plato_pod.pose import PoseSource, RobotPose


class LocalizationProvider(Protocol):
    """Protocol for localization backends.

    Each implementation detects/localizes robots using its own mechanism
    and returns poses in the unified arena coordinate frame.
    """

    name: str
    source: PoseSource

    def get_poses(self) -> list[RobotPose]:
        """Return current poses for all detected robots.

        Called periodically by the hosting node. Returns poses in arena
        coordinates (metres relative to arena origin).
        """
        ...
