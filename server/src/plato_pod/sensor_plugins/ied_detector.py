"""IED detector — point-source proximity sensor with detection radius.

Reads `environment.point_sources["ied"]` and returns any source within the
configured detection radius. Each source dict must include `position`
(tuple) and may include `detectability_radius_m` to override the default.
"""

from __future__ import annotations

import math

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    EnvironmentContext,
    SensorConfig,
    apply_noise,
)


class IedDetectorSensor:
    """Counter-IED detector digital twin."""

    name = "ied_detector"

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
        default_radius = float(config.params.get("default_detection_radius_m", 5.0))
        position_noise_m = float(config.params.get("position_noise_m", 0.5))

        sources: list[dict] = []
        if environment is not None:
            sources = environment.point_sources.get("ied", []) or []

        detections: list[dict] = []
        for src in sources:
            pos = src.get("position")
            if pos is None:
                continue
            sx, sy = float(pos[0]), float(pos[1])
            radius = float(src.get("detectability_radius_m", default_radius))
            dist = math.hypot(sx - robot.x, sy - robot.y)
            if dist > radius:
                continue
            confidence = max(0.0, min(1.0, 1.0 - dist / radius))
            detections.append({
                "position": (
                    apply_noise(sx, position_noise_m),
                    apply_noise(sy, position_noise_m),
                ),
                "true_position": (sx, sy),
                "distance_m": dist,
                "confidence": confidence,
                "label": src.get("label", "ied"),
            })

        detections.sort(key=lambda d: d["distance_m"])
        return {
            "detections": detections,
            "default_detection_radius_m": default_radius,
        }

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=2.0,
            noise_stddev=0.0,
            params={
                "default_detection_radius_m": 5.0,
                "position_noise_m": 0.5,
            },
        )
