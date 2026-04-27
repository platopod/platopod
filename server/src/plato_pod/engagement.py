"""Weapon engagement evaluation.

Pure functions that resolve fire events into outcomes (hit/miss/blocked,
damage, ROE flags). Composes line_of_sight, weather visibility, cover
modifiers, range falloff. The caller (engagement_node) applies the
resulting effects to the registry.

Design:
- Stateless evaluation; randomness from caller-supplied Random.
- Actor may be None for indirect fire (artillery, CAS) — only target
  position and weapon spec are required in that case.
- Civilian/ROE checks are returned as flags; the caller decides whether
  to log/block/punish based on exercise rules.

No ROS2 dependency.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from random import Random

from plato_pod.health import fire_capability
from plato_pod.line_of_sight import CoverPolygon, has_line_of_sight, LosResult
from plato_pod.robot import Robot
from plato_pod.weather import CLEAR, WeatherState


@dataclass
class WeaponSpec:
    """Static description of a weapon system."""
    name: str
    max_range_m: float
    base_pok_at_100m: float           # probability of kill at 100m, undegraded
    falloff_per_100m: float = 0.15    # PoK reduction per additional 100m
    damage: float = 1.0               # health subtracted on hit (0..1)
    ammo_capacity: int = 30           # initial ammo per loadout
    suppress_radius_m: float = 0.0    # area effect (artillery) — 0 = direct fire only
    min_range_m: float = 0.0          # minimum effective range (e.g., grenade arming)


@dataclass
class Civilian:
    """A civilian entity in the world for ROE checks."""
    position: tuple[float, float]
    label: str = ""


@dataclass
class EngagementOutcome:
    """Result of evaluating a fire event."""
    fired: bool                       # was the shot actually launched
    hit: bool
    blocked: bool                     # blocked by terrain/cover/visibility
    damage: float                     # 0 if missed/blocked
    distance_m: float
    effective_pok: float              # final PoK used in roll (after all modifiers)
    rationale: str                    # short tag for UI/logs
    civilian_violation: bool = False  # ROE flag (informational)
    civilians_at_risk: list[str] = field(default_factory=list)
    suppressed_targets: list[int] = field(default_factory=list)  # area-effect side hits


def _range_modifier(distance_m: float, weapon: WeaponSpec) -> float:
    """PoK at the given range, as a multiplier of base_pok_at_100m.

    Below 100m: full PoK (we treat short range as the base case).
    Above 100m: linear falloff per 100m.
    Above max_range: 0.
    """
    if distance_m < 0:
        distance_m = 0.0
    if distance_m > weapon.max_range_m:
        return 0.0
    if distance_m < weapon.min_range_m:
        return 0.0
    if distance_m <= 100.0:
        return weapon.base_pok_at_100m
    extra_hundreds = (distance_m - 100.0) / 100.0
    pok = weapon.base_pok_at_100m - extra_hundreds * weapon.falloff_per_100m
    return max(0.0, pok)


def _is_civilian_at_risk(
    target_pos: tuple[float, float],
    civilians: list[Civilian],
    proximity_m: float,
) -> list[str]:
    """Return labels of civilians within proximity_m of the target position."""
    at_risk: list[str] = []
    for civ in civilians:
        dx = civ.position[0] - target_pos[0]
        dy = civ.position[1] - target_pos[1]
        if math.hypot(dx, dy) <= proximity_m:
            at_risk.append(civ.label or f"civilian_at_{civ.position}")
    return at_risk


def evaluate_fire(
    actor: Robot | None,
    target: Robot,
    weapon: WeaponSpec,
    cover_polygons: list[CoverPolygon] | None = None,
    weather: WeatherState | None = None,
    civilians: list[Civilian] | None = None,
    civilian_proximity_m: float = 10.0,
    rng: Random | None = None,
    other_units: list[Robot] | None = None,
) -> EngagementOutcome:
    """Evaluate a single weapon fire event.

    Args:
        actor: the firing unit. May be None for indirect fire (artillery, CAS).
        target: the unit being fired at. For area weapons, this is the aim point.
        weapon: weapon characteristics.
        cover_polygons: terrain/building cover affecting LoS.
        weather: atmospheric conditions affecting visibility.
        civilians: civilian entities for ROE check.
        civilian_proximity_m: civilians within this radius of target trigger
            an ROE flag.
        rng: random number generator (caller-supplied for determinism in tests).
        other_units: for area weapons (suppress_radius_m > 0), other units within
            radius are also affected.

    Returns:
        EngagementOutcome describing what happened.
    """
    rng = rng or Random()
    cover_polygons = cover_polygons or []
    weather = weather or CLEAR
    civilians = civilians or []
    other_units = other_units or []

    # Compute distance
    if actor is not None:
        distance = math.hypot(target.x - actor.x, target.y - actor.y)
    else:
        # Indirect fire — distance is irrelevant for accuracy; treat as 100m
        distance = 100.0

    target_pos = (target.x, target.y)

    # ROE: civilian proximity
    civs_at_risk = _is_civilian_at_risk(target_pos, civilians, civilian_proximity_m)
    civilian_violation = len(civs_at_risk) > 0

    # Range check
    range_pok = _range_modifier(distance, weapon)
    if range_pok <= 0.0:
        rationale = "out_of_range" if distance > weapon.max_range_m else "below_min_range"
        return EngagementOutcome(
            fired=True, hit=False, blocked=True, damage=0.0,
            distance_m=distance, effective_pok=0.0, rationale=rationale,
            civilian_violation=civilian_violation, civilians_at_risk=civs_at_risk,
        )

    # Line of sight (only relevant for direct fire — no actor means indirect fire,
    # which doesn't require LoS from the firing unit)
    los: LosResult | None = None
    if actor is not None:
        # Z = 1m approximation (eye/sensor height) for both endpoints
        los = has_line_of_sight(
            (actor.x, actor.y, 1.0),
            (target.x, target.y, 1.0),
            terrain=None,
            cover_polygons=cover_polygons,
            weather=weather,
        )
        if not los.visible:
            return EngagementOutcome(
                fired=True, hit=False, blocked=True, damage=0.0,
                distance_m=distance, effective_pok=0.0, rationale=los.rationale,
                civilian_violation=civilian_violation, civilians_at_risk=civs_at_risk,
            )

    # Compose final PoK from range, LoS attenuation, and actor capability
    los_attenuation = los.attenuation if los is not None else 1.0
    actor_capability = fire_capability(actor.health) if actor is not None else 1.0
    effective_pok = range_pok * los_attenuation * actor_capability
    effective_pok = max(0.0, min(1.0, effective_pok))

    # Roll for hit
    roll = rng.random()
    hit = roll < effective_pok

    # Compute area-effect side hits (suppress_radius_m > 0) — informational
    suppressed: list[int] = []
    if weapon.suppress_radius_m > 0.0:
        for unit in other_units:
            if unit.robot_id == target.robot_id:
                continue
            d = math.hypot(unit.x - target.x, unit.y - target.y)
            if d <= weapon.suppress_radius_m:
                suppressed.append(unit.robot_id)

    if not hit:
        return EngagementOutcome(
            fired=True, hit=False, blocked=False, damage=0.0,
            distance_m=distance, effective_pok=effective_pok, rationale="missed",
            civilian_violation=civilian_violation, civilians_at_risk=civs_at_risk,
            suppressed_targets=suppressed,
        )

    return EngagementOutcome(
        fired=True, hit=True, blocked=False, damage=weapon.damage,
        distance_m=distance, effective_pok=effective_pok,
        rationale="hit_clean" if (los is None or los.attenuation >= 0.99) else "hit_through_cover",
        civilian_violation=civilian_violation, civilians_at_risk=civs_at_risk,
        suppressed_targets=suppressed,
    )


def weapon_from_dict(name: str, data: dict) -> WeaponSpec:
    """Build a WeaponSpec from a YAML-loaded dict (with the weapon name)."""
    return WeaponSpec(
        name=name,
        max_range_m=float(data.get("max_range_m", 100.0)),
        base_pok_at_100m=float(data.get("base_pok_at_100m", 0.5)),
        falloff_per_100m=float(data.get("falloff_per_100m", 0.15)),
        damage=float(data.get("damage", 1.0)),
        ammo_capacity=int(data.get("ammo_capacity", 30)),
        suppress_radius_m=float(data.get("suppress_radius_m", 0.0)),
        min_range_m=float(data.get("min_range_m", 0.0)),
    )
