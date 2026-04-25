"""GPS sensor plugin — returns robot position with optional noise.

The simplest sensor: reads the robot's pose directly and adds configurable
Gaussian noise. No ray casting needed.
"""

from __future__ import annotations

from plato_pod.kinematics import normalize_angle
from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    SensorConfig,
    apply_noise,
)


class GpsSensor:
    """GPS sensor — position and heading with noise."""

    name = "gps"

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        **kwargs,
    ) -> dict:
        """Return robot pose with optional Gaussian noise."""
        pos_noise = config.noise_stddev
        heading_noise = config.params.get("heading_noise_stddev", pos_noise * 10)

        x = apply_noise(robot.x, pos_noise)
        y = apply_noise(robot.y, pos_noise)
        theta = normalize_angle(apply_noise(robot.theta, heading_noise))

        return {
            "x": x,
            "y": y,
            "theta": theta,
            "origin_tag_id": 101,
        }

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=10.0,
            noise_stddev=0.001,  # 1mm position noise
            params={"heading_noise_stddev": 0.01},  # ~0.6 degree heading noise
        )
