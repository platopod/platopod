"""Atmospheric and visibility state.

Weather affects sensor range, gas plume direction, and visual detection.
Configurable per exercise; can be updated at runtime via inject_event.

Pure dataclass + helper functions. No ROS2 dependency.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class WeatherState:
    """Current atmospheric conditions.

    Used by line_of_sight (visibility), engagement (long-range accuracy),
    and gas plume models (wind speed/direction).
    """
    visibility_m: float = 1000.0     # max sight range in current conditions
    wind_speed: float = 0.0          # m/s
    wind_direction: float = 0.0      # radians, direction wind blows TO
    fog_density: float = 0.0         # 0=clear, 1=opaque
    time_of_day: float = 12.0        # hours since midnight (affects sensor performance)


CLEAR = WeatherState(
    visibility_m=10000.0,
    wind_speed=0.0,
    wind_direction=0.0,
    fog_density=0.0,
    time_of_day=12.0,
)


def visibility_factor(weather: WeatherState, distance_m: float) -> float:
    """Compute the visibility scaling at a given distance under current weather.

    Returns a value in [0.0, 1.0]:
        1.0 = perfect visibility (clear weather, short distance)
        0.0 = no visibility (beyond visibility_m or in dense fog at distance)

    Uses an exponential decay model: visibility drops faster as distance
    approaches the maximum visibility range, and fog accelerates the decay.
    """
    if distance_m <= 0:
        return 1.0
    if distance_m >= weather.visibility_m:
        return 0.0
    # Linear distance falloff
    distance_factor = 1.0 - (distance_m / weather.visibility_m)
    # Fog reduces effective visibility multiplicatively
    fog_attenuation = 1.0 - weather.fog_density
    return max(0.0, distance_factor * fog_attenuation)


def from_dict(data: dict) -> WeatherState:
    """Build a WeatherState from a YAML-loaded dict."""
    return WeatherState(
        visibility_m=float(data.get("visibility_m", 1000.0)),
        wind_speed=float(data.get("wind_speed", 0.0)),
        wind_direction=float(data.get("wind_direction", 0.0)),
        fog_density=float(data.get("fog_density", 0.0)),
        time_of_day=float(data.get("time_of_day", 12.0)),
    )
