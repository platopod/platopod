"""UAV camera — overhead FOV sweep returning detected ground units.

Treats the host `Robot` as a UAV (its (x, y) is the ground projection of
the camera footprint centre). Returns every other operational unit whose
position lies inside the rectangular footprint, attenuated by weather but
not by terrain (overhead view).

Footprint dimensions are config params; the UAV's heading rotates the
rectangle.
"""

from __future__ import annotations

import math

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    EnvironmentContext,
    SensorConfig,
)
from plato_pod.weather import CLEAR, visibility_factor


class UavCameraSensor:
    """Overhead camera digital twin for a virtual UAV unit."""

    name = "uav_camera"

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
        footprint_w = float(config.params.get("footprint_width_m", 200.0))
        footprint_h = float(config.params.get("footprint_height_m", 150.0))
        team_filter = config.params.get("ignore_teams", [])
        weather = config.params.get("weather", CLEAR) or CLEAR

        cos_h = math.cos(-robot.theta)
        sin_h = math.sin(-robot.theta)
        half_w = footprint_w / 2.0
        half_h = footprint_h / 2.0

        detections: list[dict] = []
        for other in other_robots:
            if other.robot_id == robot.robot_id:
                continue
            if not other.is_operational():
                continue
            if team_filter and other.team in team_filter:
                continue
            dx = other.x - robot.x
            dy = other.y - robot.y
            # Rotate into UAV frame
            local_x = dx * cos_h - dy * sin_h
            local_y = dx * sin_h + dy * cos_h
            if abs(local_x) > half_w or abs(local_y) > half_h:
                continue

            dist = math.hypot(dx, dy)
            confidence = visibility_factor(weather, dist)
            if confidence <= 0.0:
                continue

            detections.append({
                "robot_id": other.robot_id,
                "team": other.team,
                "vehicle_role": other.vehicle_role,
                "position": (other.x, other.y),
                "local_position": (local_x, local_y),
                "confidence": confidence,
            })

        return {
            "detections": detections,
            "footprint_width_m": footprint_w,
            "footprint_height_m": footprint_h,
            "uav_position": (robot.x, robot.y),
            "uav_heading_rad": robot.theta,
        }

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=4.0,
            noise_stddev=0.0,
            params={
                "footprint_width_m": 200.0,
                "footprint_height_m": 150.0,
                "ignore_teams": [],
            },
        )
