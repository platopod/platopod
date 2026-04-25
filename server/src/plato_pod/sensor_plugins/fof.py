"""Friend-or-Foe sensor plugin — returns detected friendly/hostile units.

Currently returns empty detections. Planned: team-aware detection using
robot positions and team assignments from the exercise configuration.
"""

from __future__ import annotations

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    SensorConfig,
)


class FofSensor:
    """Friend-or-Foe detection — stub returning no detections."""

    name = "fof"

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        **kwargs,
    ) -> dict:
        """Return empty detection list (stub)."""
        return {
            "detections": [],
            "range_max": config.params.get("range_max", 1.0),
        }

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=5.0,
            noise_stddev=0.0,
            params={"range_max": 1.0, "fov_deg": 360.0},
        )
