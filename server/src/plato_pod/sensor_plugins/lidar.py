"""Lidar sensor plugin — Python fallback returning max-range readings.

Returns max-range data when Gazebo lidar is not available.
When Gazebo is running, GazeboLidarBridge provides real ray-cast data.
"""

from __future__ import annotations

import math

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    SensorConfig,
)


class LidarSensor:
    """2D Lidar — stub returning max-range readings."""

    name = "lidar_2d"

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        **kwargs,
    ) -> dict:
        """Return max-range readings (no obstacles detected — stub)."""
        range_max = config.params.get("range_max", 2.0)
        resolution_deg = config.params.get("angular_resolution_deg", 1.0)
        n_rays = int(360.0 / resolution_deg)

        return {
            "ranges": [range_max] * n_rays,
            "angle_min": 0.0,
            "angle_max": 2.0 * math.pi,
            "angle_increment": math.radians(resolution_deg),
            "range_max": range_max,
        }

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=10.0,
            noise_stddev=0.005,
            params={"range_max": 2.0, "angular_resolution_deg": 1.0},
        )
