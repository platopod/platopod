"""Virtual layer loader — creates EnvironmentContext from exercise YAML.

Parses the optional `virtual_layers` section from exercise YAML and
instantiates SpatialField objects and an EnvironmentContext. Pure Python,
no ROS2 dependency.
"""

from __future__ import annotations

import logging

from plato_pod.sensor_plugins.base import EnvironmentContext
from plato_pod.spatial_field import (
    GaussianPlumeField,
    ElevationField,
    UniformField,
)

logger = logging.getLogger(__name__)


def load_virtual_layers(exercise_config: dict) -> EnvironmentContext | None:
    """Load virtual layers from an exercise YAML config dict.

    Args:
        exercise_config: The top-level exercise config dict (the value of
            the 'exercise' key in the YAML, or the whole dict if no
            'exercise' wrapper).

    Returns:
        EnvironmentContext with all configured fields, or None if no
        virtual_layers section is present.

    Example YAML structure::

        exercise:
          virtual_layers:
            gas_sources:
              - name: "plume_alpha"
                x: 0.5
                y: 0.3
                release_rate: 100.0
                wind_speed: 2.0
                wind_direction: 0.0
                diffusion_coeff: 0.05
            terrain:
              grid_data: [[0, 0], [1, 1]]
              origin: [0, 0]
              resolution: 0.1
            environment:
              wind_speed: 2.0
              wind_direction: 1.57
              temperature: 25.0
    """
    exercise = exercise_config.get("exercise", exercise_config)
    layers_cfg = exercise.get("virtual_layers")
    if layers_cfg is None:
        return None

    fields: dict = {}

    # Load gas sources
    gas_sources = layers_cfg.get("gas_sources", [])
    for src in gas_sources:
        name = src.get("name", "gas")
        field = GaussianPlumeField(
            source_x=float(src.get("x", 0.0)),
            source_y=float(src.get("y", 0.0)),
            release_rate=float(src.get("release_rate", 100.0)),
            wind_speed=float(src.get("wind_speed", 2.0)),
            wind_direction=float(src.get("wind_direction", 0.0)),
            diffusion_coeff=float(src.get("diffusion_coeff", 0.05)),
        )
        fields[name] = field
        logger.info("Loaded gas source '%s' at (%.2f, %.2f)", name, field.source_x, field.source_y)

    # If multiple gas sources share the name "gas", composite them
    # (handled naturally: last one wins, or use unique names)

    # Load terrain elevation grid
    terrain_cfg = layers_cfg.get("terrain")
    if terrain_cfg:
        grid_data = terrain_cfg.get("grid_data", [])
        origin = terrain_cfg.get("origin", [0, 0])
        resolution = float(terrain_cfg.get("resolution", 0.1))
        name = terrain_cfg.get("name", "elevation")

        if grid_data:
            field = ElevationField(
                grid_data=grid_data,
                origin_x=float(origin[0]),
                origin_y=float(origin[1]),
                resolution=resolution,
                name=name,
            )
            fields[name] = field
            logger.info(
                "Loaded terrain '%s': %dx%d grid, resolution=%.3f",
                name, len(grid_data), len(grid_data[0]) if grid_data else 0,
                resolution,
            )

    # Load uniform background fields
    for bg in layers_cfg.get("background_fields", []):
        name = bg.get("name", "background")
        value = float(bg.get("value", 0.0))
        fields[name] = UniformField(value=value, name=name)
        logger.info("Loaded background field '%s': value=%.2f", name, value)

    # Environment parameters
    env_cfg = layers_cfg.get("environment", {})
    wind_speed = float(env_cfg.get("wind_speed", 0.0))
    wind_direction = float(env_cfg.get("wind_direction", 0.0))
    temperature = float(env_cfg.get("temperature", 20.0))

    # Point-source style entities (IED zones, EW emitters, civilian
    # populations). Each is a list of dicts with at minimum a `position`.
    # Plugins read these via environment.point_sources[<key>].
    point_sources: dict[str, list[dict]] = {}

    for entry in layers_cfg.get("ied_zones", []) or []:
        pos = entry.get("position")
        if pos is None:
            continue
        point_sources.setdefault("ied", []).append({
            "position": (float(pos[0]), float(pos[1])),
            "detectability_radius_m": float(
                entry.get("detectability_radius_m", 5.0)
            ),
            "label": str(entry.get("label", "ied")),
        })

    for entry in layers_cfg.get("ew_emitters", []) or []:
        pos = entry.get("position")
        if pos is None:
            continue
        point_sources.setdefault("ew_emitters", []).append({
            "position": (float(pos[0]), float(pos[1])),
            "frequency_mhz": float(entry.get("frequency_mhz", 0.0)),
            "signal_strength": float(entry.get("signal_strength", 1.0)),
            "label": str(entry.get("label", "emitter")),
        })

    for entry in layers_cfg.get("civilian_population", []) or []:
        pos = entry.get("position")
        if pos is None:
            continue
        point_sources.setdefault("civilian", []).append({
            "position": (float(pos[0]), float(pos[1])),
            "movement": str(entry.get("movement", "stationary")),
            "radius_m": float(entry.get("radius_m", 0.0)),
            "count": int(entry.get("count", 1)),
            "label": str(entry.get("label", "civilian")),
        })

    if point_sources:
        logger.info(
            "Loaded point sources: %s",
            {k: len(v) for k, v in point_sources.items()},
        )

    return EnvironmentContext(
        fields=fields,
        point_sources=point_sources,
        wind_speed=wind_speed,
        wind_direction=wind_direction,
        temperature=temperature,
        time=0.0,
    )
