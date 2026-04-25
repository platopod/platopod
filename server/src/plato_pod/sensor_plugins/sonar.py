"""Sonar sensor plugin — Python fallback returning max-range readings.

Returns max-range data. A future Gazebo sonar bridge could replace this
with physics-based sonar simulation.
"""

from __future__ import annotations

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    SensorConfig,
)


class SonarSensor:
    """Sonar — stub returning max-range readings."""

    name = "sonar"

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        **kwargs,
    ) -> dict:
        """Return max-range readings (stub)."""
        range_max = config.params.get("range_max", 1.5)
        segment_angle = config.params.get("segment_angle_deg", 10.0)
        n_segments = int(360.0 / segment_angle)

        return {
            "ranges": [range_max] * n_segments,
            "segment_angle_deg": segment_angle,
            "range_max": range_max,
        }

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=10.0,
            noise_stddev=0.02,
            params={"range_max": 1.5, "segment_angle_deg": 10.0},
        )
