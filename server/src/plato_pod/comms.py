"""Per-unit communications state — range, terrain, jamming, dead zones.

A unit is *linked* if at least one teammate is reachable: within radio
range, line of sight clear (terrain or cover), not in a jamming or
dead-zone polygon. Reach quality decays with range and jamming strength.

Pure Python, no ROS2. Consumed by the CoT bridge to gate fresh updates
(stale symbol on ATAK when comms drop) and by the dashboard.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from plato_pod.geometry import point_in_polygon
from plato_pod.line_of_sight import CoverPolygon, has_line_of_sight
from plato_pod.robot import Robot
from plato_pod.spatial_field import ElevationField


@dataclass
class JammingZone:
    """Circular jamming zone — degrades comms quality within radius."""
    position: tuple[float, float]
    radius_m: float
    strength: float = 1.0       # 0..1; 1.0=fully jammed inside
    label: str = ""


@dataclass
class DeadZonePolygon:
    """Static polygon where comms simply do not work."""
    vertices: list[tuple[float, float]]
    label: str = ""


@dataclass
class CommsConfig:
    """Per-unit comms parameters."""
    max_range_m: float = 1000.0
    cover_attenuates: bool = True
    require_los: bool = True


@dataclass
class CommsState:
    """Computed comms state for one unit at one point in time."""
    linked: bool
    quality: float                       # 0..1
    relay_id: int | None = None          # teammate that provided the link
    rationale: str = ""
    blocked_by: list[str] = field(default_factory=list)


def _in_dead_zone(unit: Robot, dead_zones: list[DeadZonePolygon]) -> str | None:
    for dz in dead_zones:
        if point_in_polygon((unit.x, unit.y), dz.vertices):
            return dz.label or "dead_zone"
    return None


def _jamming_at(x: float, y: float, zones: list[JammingZone]) -> tuple[float, str]:
    """Return (max_strength, label) of any jamming zone containing the point."""
    worst = 0.0
    label = ""
    for z in zones:
        d = math.hypot(x - z.position[0], y - z.position[1])
        if d <= z.radius_m and z.strength > worst:
            worst = z.strength
            label = z.label or "jammer"
    return worst, label


def evaluate_comms(
    unit: Robot,
    team: list[Robot],
    config: CommsConfig | None = None,
    terrain: ElevationField | None = None,
    cover_polygons: list[CoverPolygon] | None = None,
    jamming_zones: list[JammingZone] | None = None,
    dead_zones: list[DeadZonePolygon] | None = None,
) -> CommsState:
    """Evaluate whether a unit has working comms via any teammate.

    A unit can reach exactly one teammate to be considered linked. Quality
    is the best link found (linear range falloff, multiplied by 1-jamming).
    """
    cfg = config or CommsConfig()
    cover = cover_polygons or []
    jammers = jamming_zones or []
    dz = dead_zones or []

    blocked: list[str] = []

    own_dead = _in_dead_zone(unit, dz)
    if own_dead:
        return CommsState(linked=False, quality=0.0,
                          rationale=f"dead_zone:{own_dead}",
                          blocked_by=[own_dead])

    own_jam, own_jam_label = _jamming_at(unit.x, unit.y, jammers)
    if own_jam >= 1.0:
        return CommsState(linked=False, quality=0.0,
                          rationale=f"jammed:{own_jam_label}",
                          blocked_by=[own_jam_label])

    best: CommsState | None = None
    for other in team:
        if other.robot_id == unit.robot_id:
            continue
        if not other.is_operational():
            continue
        d = math.hypot(unit.x - other.x, unit.y - other.y)
        if d > cfg.max_range_m:
            blocked.append(f"out_of_range:{other.robot_id}")
            continue

        if cfg.require_los:
            los = has_line_of_sight(
                (unit.x, unit.y, 1.0),
                (other.x, other.y, 1.0),
                terrain=terrain,
                cover_polygons=cover if cfg.cover_attenuates else [],
            )
            if not los.visible:
                blocked.append(f"los:{other.robot_id}:{los.rationale}")
                continue
            link_attenuation = los.attenuation
        else:
            link_attenuation = 1.0

        # Range falloff: quality = 1 - (d / max_range)
        range_quality = max(0.0, 1.0 - d / cfg.max_range_m)
        # Jamming at either endpoint reduces quality
        peer_jam, _ = _jamming_at(other.x, other.y, jammers)
        jamming_factor = (1.0 - own_jam) * (1.0 - peer_jam)
        quality = range_quality * link_attenuation * jamming_factor

        if quality <= 0.0:
            continue
        if best is None or quality > best.quality:
            best = CommsState(
                linked=True, quality=quality, relay_id=other.robot_id,
                rationale=f"linked_via_{other.robot_id}",
            )

    if best is not None:
        return best

    return CommsState(
        linked=False, quality=0.0,
        rationale="no_reachable_teammate",
        blocked_by=blocked,
    )


def comms_config_from_dict(data: dict) -> CommsConfig:
    return CommsConfig(
        max_range_m=float(data.get("max_range_m", 1000.0)),
        cover_attenuates=bool(data.get("cover_attenuates", True)),
        require_los=bool(data.get("require_los", True)),
    )


def jamming_zone_from_dict(data: dict) -> JammingZone:
    pos = data["position"]
    return JammingZone(
        position=(float(pos[0]), float(pos[1])),
        radius_m=float(data.get("radius_m", 100.0)),
        strength=float(data.get("strength", 1.0)),
        label=str(data.get("label", "")),
    )


def dead_zone_from_dict(data: dict) -> DeadZonePolygon:
    return DeadZonePolygon(
        vertices=[(float(v[0]), float(v[1])) for v in data["vertices"]],
        label=str(data.get("label", "")),
    )
