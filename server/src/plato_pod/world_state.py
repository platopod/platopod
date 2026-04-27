"""World state — single source of truth for the tactical world.

Aggregates everything that used to be re-parsed by `engagement_node`,
`opfor_node`, `cot_bridge_node`, and `sensor_engine_node` from the
exercise YAML. The `world_state_node` ROS2 shell publishes this to
language-neutral latched topics so any consumer (Python, C++, MATLAB)
can subscribe and stay in sync.

Pure Python — testable without ROS2.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from plato_pod.comms import (
    DeadZonePolygon, JammingZone,
    dead_zone_from_dict, jamming_zone_from_dict,
)
from plato_pod.engagement import WeaponSpec, weapon_from_dict
from plato_pod.line_of_sight import CoverPolygon
from plato_pod.roe import ROERules, roe_from_dict
from plato_pod.weather import CLEAR, WeatherState, from_dict as weather_from_dict


@dataclass
class WorldState:
    """All declarative tactical world data for the active exercise."""
    cover: list[CoverPolygon] = field(default_factory=list)
    civilians: list[dict] = field(default_factory=list)        # {position, label, …}
    ied_zones: list[dict] = field(default_factory=list)        # {position, detect_radius, label}
    ew_emitters: list[dict] = field(default_factory=list)      # {position, freq, signal, label}
    jamming_zones: list[JammingZone] = field(default_factory=list)
    dead_zones: list[DeadZonePolygon] = field(default_factory=list)
    weather: WeatherState = field(default_factory=lambda: CLEAR)
    roe: ROERules = field(default_factory=ROERules)
    weapons: dict[str, WeaponSpec] = field(default_factory=dict)


def world_state_from_config(config: dict) -> WorldState:
    """Build a WorldState from a parsed exercise YAML dict."""
    exercise = config.get("exercise", config)
    layers = exercise.get("virtual_layers", {}) or {}
    state = WorldState()

    # Cover polygons
    for entry in layers.get("cover_polygons", []) or []:
        try:
            state.cover.append(CoverPolygon(
                vertices=[tuple(v) for v in entry["vertices"]],
                cover_value=float(entry.get("cover_value", 1.0)),
                label=str(entry.get("label", "")),
            ))
        except (KeyError, TypeError, ValueError):
            continue

    # Civilians (kept as dicts; consumers don't need a typed dataclass)
    for entry in layers.get("civilian_population", []) or []:
        pos = entry.get("position")
        if pos is None:
            continue
        state.civilians.append({
            "position": (float(pos[0]), float(pos[1])),
            "label": str(entry.get("label", "civilian")),
            "movement": str(entry.get("movement", "stationary")),
            "count": int(entry.get("count", 1)),
            "radius_m": float(entry.get("radius_m", 0.0)),
        })

    # IED zones
    for entry in layers.get("ied_zones", []) or []:
        pos = entry.get("position")
        if pos is None:
            continue
        state.ied_zones.append({
            "position": (float(pos[0]), float(pos[1])),
            "detectability_radius_m": float(entry.get("detectability_radius_m", 5.0)),
            "label": str(entry.get("label", "ied")),
        })

    # EW emitters
    for entry in layers.get("ew_emitters", []) or []:
        pos = entry.get("position")
        if pos is None:
            continue
        state.ew_emitters.append({
            "position": (float(pos[0]), float(pos[1])),
            "frequency_mhz": float(entry.get("frequency_mhz", 0.0)),
            "signal_strength": float(entry.get("signal_strength", 1.0)),
            "label": str(entry.get("label", "emitter")),
        })

    # Jamming zones
    for entry in layers.get("jamming_zones", []) or []:
        try:
            state.jamming_zones.append(jamming_zone_from_dict(entry))
        except (KeyError, TypeError, ValueError):
            continue

    # Comms dead zones
    for entry in layers.get("comms_dead_zones", []) or []:
        try:
            state.dead_zones.append(dead_zone_from_dict(entry))
        except (KeyError, TypeError, ValueError):
            continue

    # Weather
    if "weather" in exercise:
        try:
            state.weather = weather_from_dict(exercise["weather"])
        except (KeyError, TypeError, ValueError):
            pass

    # ROE
    if "roe" in exercise:
        try:
            state.roe = roe_from_dict(exercise["roe"])
        except (KeyError, TypeError, ValueError):
            pass

    # Weapons catalog
    for name, data in (exercise.get("weapons", {}) or {}).items():
        try:
            state.weapons[name] = weapon_from_dict(name, data)
        except (KeyError, TypeError, ValueError):
            continue

    return state


def load_world_state(exercise_file: str) -> WorldState:
    """Read an exercise YAML file and return its WorldState."""
    import yaml
    with open(exercise_file) as f:
        config = yaml.safe_load(f)
    return world_state_from_config(config or {})
