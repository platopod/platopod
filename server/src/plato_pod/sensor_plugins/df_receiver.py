"""Direction-finding receiver — bearings to RF emitters with noise.

Reads `environment.point_sources["ew_emitters"]` (each with `position` and
optional `frequency_mhz`, `signal_strength`, `label`). Returns one bearing
per emitter within the configured detection range, with bearing noise that
grows with distance (1/√strength behaviour).

This is a passive sensor: range is implicit (signal-strength surrogate)
rather than measured.
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


class DfReceiverSensor:
    """Direction-finding (RDF) receiver digital twin."""

    name = "df_receiver"

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
        max_range = float(config.params.get("max_range_m", 5000.0))
        bearing_noise_deg = float(config.params.get("bearing_noise_deg", 2.0))
        min_signal = float(config.params.get("min_signal_strength", 0.05))

        sources: list[dict] = []
        if environment is not None:
            sources = environment.point_sources.get("ew_emitters", []) or []

        bearings: list[dict] = []
        for src in sources:
            pos = src.get("position")
            if pos is None:
                continue
            sx, sy = float(pos[0]), float(pos[1])
            dist = math.hypot(sx - robot.x, sy - robot.y)
            if dist > max_range or dist == 0:
                continue

            strength = float(src.get("signal_strength", 1.0))
            # Inverse-square attenuation
            received = strength / max(1.0, (dist / 100.0) ** 2)
            if received < min_signal:
                continue

            # Bearing noise scales with 1/sqrt(received)
            noise_deg = bearing_noise_deg / math.sqrt(received)
            true_bearing = math.atan2(sy - robot.y, sx - robot.x)
            measured = apply_noise(math.degrees(true_bearing), noise_deg)

            bearings.append({
                "bearing_deg": measured,
                "true_bearing_deg": math.degrees(true_bearing),
                "signal_strength": received,
                "frequency_mhz": float(src.get("frequency_mhz", 0.0)),
                "label": src.get("label", "emitter"),
            })

        bearings.sort(key=lambda b: -b["signal_strength"])
        return {
            "bearings": bearings,
            "max_range_m": max_range,
        }

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=2.0,
            noise_stddev=0.0,
            params={
                "max_range_m": 5000.0,
                "bearing_noise_deg": 2.0,
                "min_signal_strength": 0.05,
            },
        )
