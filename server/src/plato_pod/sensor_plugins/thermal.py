"""Thermal imager — detects warm units within FOV, attenuated by weather/cover.

Returns one detection per visible unit weighted by its `thermal_signature`
(0=cold, 1=hot/engine running). Detections include bearing, range, and an
intensity score reflecting LoS attenuation and weather visibility.
"""

from __future__ import annotations

import math

from plato_pod.line_of_sight import has_line_of_sight
from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    EnvironmentContext,
    SensorConfig,
)
from plato_pod.weather import CLEAR


class ThermalSensor:
    """Thermal imager digital twin."""

    name = "thermal"

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        environment: EnvironmentContext | None = None,
        state: dict | None = None,
        **kwargs,
    ) -> dict:
        max_range = float(config.params.get("max_range_m", 800.0))
        fov_deg = float(config.params.get("fov_deg", 60.0))
        min_signature = float(config.params.get("min_signature", 0.05))
        cover_polygons = config.params.get("cover_polygons", []) or []
        weather = config.params.get("weather", CLEAR) or CLEAR

        half_fov = math.radians(fov_deg) / 2.0
        detections: list[dict] = []

        for other in other_robots:
            if not other.is_operational():
                continue
            if other.thermal_signature < min_signature:
                continue
            dx = other.x - robot.x
            dy = other.y - robot.y
            dist = math.hypot(dx, dy)
            if dist == 0 or dist > max_range:
                continue
            bearing = math.atan2(dy, dx)
            if abs(_wrap(bearing - robot.theta)) > half_fov:
                continue

            los = has_line_of_sight(
                (robot.x, robot.y, 1.0),
                (other.x, other.y, 1.0),
                cover_polygons=cover_polygons,
                weather=weather,
            )
            if not los.visible:
                continue

            intensity = other.thermal_signature * los.attenuation
            if intensity < min_signature:
                continue

            detections.append({
                "robot_id": other.robot_id,
                "bearing_rad": bearing,
                "range_m": dist,
                "intensity": intensity,
            })

        detections.sort(key=lambda d: d["range_m"])
        return {
            "detections": detections,
            "max_range_m": max_range,
            "fov_deg": fov_deg,
        }

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=5.0,
            noise_stddev=0.0,
            params={
                "max_range_m": 800.0,
                "fov_deg": 60.0,
                "min_signature": 0.05,
            },
        )


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
