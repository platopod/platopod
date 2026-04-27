"""Rangefinder sensor — distance to a designated target with LoS check.

Picks the nearest other-team unit within the configured cone-of-view (or
nearest of any other unit when `team_filter` is empty), checks line of
sight against cover polygons and weather visibility, and returns range,
target id, and accuracy.
"""

from __future__ import annotations

import math

from plato_pod.line_of_sight import CoverPolygon, has_line_of_sight
from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    EnvironmentContext,
    SensorConfig,
    apply_noise,
)
from plato_pod.weather import CLEAR


class RangefinderSensor:
    """Laser rangefinder digital twin."""

    name = "rangefinder"

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
        max_range = float(config.params.get("max_range_m", 1500.0))
        fov_deg = float(config.params.get("fov_deg", 5.0))
        team_filter = config.params.get("enemy_teams", [])
        accuracy_m = float(config.params.get("accuracy_m", 0.5))

        cover_polygons: list[CoverPolygon] = config.params.get("cover_polygons", []) or []
        weather = config.params.get("weather", CLEAR) or CLEAR

        half_fov = math.radians(fov_deg) / 2.0
        candidates: list[tuple[Robot, float]] = []
        for other in other_robots:
            if not other.is_operational():
                continue
            if team_filter and other.team not in team_filter:
                continue
            dx = other.x - robot.x
            dy = other.y - robot.y
            dist = math.hypot(dx, dy)
            if dist == 0 or dist > max_range:
                continue
            bearing = math.atan2(dy, dx)
            heading_err = _wrap(bearing - robot.theta)
            if abs(heading_err) > half_fov:
                continue
            candidates.append((other, dist))

        if not candidates:
            return {
                "range_m": None, "target_id": None, "accuracy_m": accuracy_m,
                "reason": "no_target",
            }

        candidates.sort(key=lambda c: c[1])
        for target, dist in candidates:
            los = has_line_of_sight(
                (robot.x, robot.y, 1.0),
                (target.x, target.y, 1.0),
                cover_polygons=cover_polygons,
                weather=weather,
            )
            if los.visible:
                noisy = max(0.0, apply_noise(dist, accuracy_m))
                return {
                    "range_m": noisy, "target_id": target.robot_id,
                    "accuracy_m": accuracy_m, "reason": "ok",
                }

        return {
            "range_m": None, "target_id": None, "accuracy_m": accuracy_m,
            "reason": "los_blocked",
        }

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=2.0,
            noise_stddev=0.0,
            params={
                "max_range_m": 1500.0,
                "fov_deg": 5.0,
                "accuracy_m": 0.5,
                "enemy_teams": [],
            },
        )


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
