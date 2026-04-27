"""Line-of-sight evaluation between two points in the arena.

Computes whether one position can see another, accounting for:
- Cover polygons (buildings, trees, other obstructions)
- Terrain elevation (ridges that block view)
- Weather visibility (fog, rain, distance attenuation)

Pure Python; reuses geometry primitives. No ROS2 dependency.

Initial implementation is 2D + height samples (terrain elevation along the
ray). Upgrade to full 3D Gazebo raycast when terrain mode is in regular use.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from plato_pod.spatial_field import ElevationField
from plato_pod.weather import CLEAR, WeatherState, visibility_factor


@dataclass
class CoverPolygon:
    """A region that obstructs or attenuates line of sight.

    cover_value in [0, 1]: 0 = fully transparent (no effect),
                          1 = fully opaque (blocks LoS completely).
    Light cover (e.g., scattered trees) might be 0.3; a building wall is 1.0.
    """
    vertices: list[tuple[float, float]]
    cover_value: float = 1.0
    label: str = ""


@dataclass
class LosResult:
    """Result of a line-of-sight evaluation.

    visible: True if the target is visible at all.
    attenuation: 0..1 multiplier on detection probability.
                 1.0 = perfect visibility, 0.0 = fully blocked.
    rationale: human-readable reason (for debugging and UI display).
    """
    visible: bool
    attenuation: float
    rationale: str


def _ray_intersects_segment(
    p0: tuple[float, float], p1: tuple[float, float],
    a: tuple[float, float], b: tuple[float, float],
) -> bool:
    """Test whether segment p0-p1 intersects segment a-b."""
    def ccw(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    d1 = ccw(a, b, p0)
    d2 = ccw(a, b, p1)
    d3 = ccw(p0, p1, a)
    d4 = ccw(p0, p1, b)
    return ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
           ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0))


def _ray_intersects_polygon(
    p0: tuple[float, float], p1: tuple[float, float],
    polygon: list[tuple[float, float]],
) -> bool:
    """Test whether segment p0-p1 crosses the boundary of a polygon."""
    n = len(polygon)
    if n < 3:
        return False
    for i in range(n):
        a = polygon[i]
        b = polygon[(i + 1) % n]
        if _ray_intersects_segment(p0, p1, a, b):
            return True
    return False


def _terrain_blocks(
    observer: tuple[float, float, float],
    target: tuple[float, float, float],
    terrain: ElevationField,
    samples: int = 20,
) -> bool:
    """Sample terrain elevation along the line; check if any sample exceeds
    the line-of-sight height at that distance.

    Returns True if terrain blocks the view.
    """
    ox, oy, oz = observer
    tx, ty, tz = target
    if samples < 2:
        return False
    for i in range(1, samples):
        t = i / samples
        sx = ox + t * (tx - ox)
        sy = oy + t * (ty - oy)
        line_z = oz + t * (tz - oz)
        terrain_z = terrain.evaluate(sx, sy, 0.0)
        # Add a small clearance (eye height); terrain must clearly exceed line
        if terrain_z > line_z + 0.01:
            return True
    return False


def has_line_of_sight(
    observer: tuple[float, float, float],
    target: tuple[float, float, float],
    terrain: ElevationField | None = None,
    cover_polygons: list[CoverPolygon] | None = None,
    weather: WeatherState | None = None,
) -> LosResult:
    """Evaluate line of sight from observer to target.

    Args:
        observer: (x, y, z) of the observer in arena/world frame.
        target: (x, y, z) of the target in same frame.
        terrain: optional ElevationField; if given, checks for terrain occlusion.
        cover_polygons: optional list of CoverPolygons; each polygon's
                        cover_value is multiplied into the attenuation.
        weather: optional WeatherState; affects visibility attenuation.
                 Defaults to CLEAR.

    Returns:
        LosResult with visible flag, attenuation factor, and rationale.
    """
    weather = weather or CLEAR
    cover_polygons = cover_polygons or []

    ox, oy, oz = observer
    tx, ty, tz = target
    distance_m = math.hypot(tx - ox, ty - oy)

    # 1. Weather visibility check
    vis = visibility_factor(weather, distance_m)
    if vis <= 0.0:
        return LosResult(False, 0.0, "out_of_visual_range")

    # 2. Terrain occlusion
    if terrain is not None and _terrain_blocks(observer, target, terrain):
        return LosResult(False, 0.0, "blocked_by_terrain")

    # 3. Cover polygons — accumulate attenuation
    p0 = (ox, oy)
    p1 = (tx, ty)
    cover_attenuation = 1.0
    blocked_by = ""
    for cover in cover_polygons:
        if _ray_intersects_polygon(p0, p1, cover.vertices):
            cover_attenuation *= (1.0 - cover.cover_value)
            blocked_by = cover.label or "cover"
            if cover_attenuation <= 0.0:
                return LosResult(False, 0.0, f"blocked_by_{blocked_by}")

    final_attenuation = vis * cover_attenuation
    if final_attenuation <= 0.0:
        return LosResult(False, 0.0, f"blocked_by_{blocked_by}" if blocked_by else "no_visibility")
    if blocked_by:
        return LosResult(True, final_attenuation, f"partial_cover_{blocked_by}")
    if vis < 1.0:
        return LosResult(True, final_attenuation, "weather_attenuated")
    return LosResult(True, final_attenuation, "clear")
