"""Unified robot pose representation for the Plato Pod platform.

RobotPose is the canonical pose type consumed by all downstream components.
The source field indicates which localization backend produced the pose.
Consumers never need to know the backend — they only see (x, y, theta).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class PoseSource(Enum):
    """Localization backend that produced a pose."""
    CAMERA_ARTAG = "camera_artag"
    GPS_IMU = "gps_imu"
    RF_ANCHOR = "rf_anchor"
    VIRTUAL_SIM = "virtual_sim"
    GAZEBO_SIM = "gazebo_sim"
    UNKNOWN = "unknown"


@dataclass(frozen=True, slots=True)
class RobotPose:
    """A robot's position and orientation in the arena frame.

    This is the single interface between localization backends and the rest
    of the platform. All coordinates are in metres relative to the arena
    origin.
    """
    robot_id: int
    x: float                # metres, arena-local frame
    y: float                # metres, arena-local frame
    theta: float            # radians, 0 = arena +x axis
    timestamp: float        # seconds since epoch
    source: PoseSource      # which backend produced this
    confidence: float       # 0.0 to 1.0, method-dependent
    localization_id: str    # provider-specific ID (tag "3", gps "rover-1")
