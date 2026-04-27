"""Per-unit consumables — fuel, ammo, water.

Logistics is a small bag of state attached to a Robot via the
`Robot.logistics` field. Pure functions return new Robot instances rather
than mutating, matching the rest of the platform's functional style.

Resupply is invoked via the `inject_event` admin pipeline; this module
just provides the consumption/check helpers used by the engagement and
movement pipelines.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from plato_pod.robot import Robot


@dataclass
class Logistics:
    """Per-unit consumables snapshot."""
    fuel: float = 1.0                       # 0..1 of full tank
    ammo: dict[str, int] = field(default_factory=dict)  # weapon → rounds
    water: float = 1.0                      # 0..1; soldiers only
    fuel_rate_per_km: float = 0.05          # fraction of tank per km travelled


def consume_fuel(robot: Robot, distance_m: float) -> Robot:
    """Apply fuel consumption for a movement of `distance_m` metres."""
    if robot.logistics is None or distance_m <= 0:
        return robot
    used = (distance_m / 1000.0) * robot.logistics.fuel_rate_per_km
    new_fuel = max(0.0, robot.logistics.fuel - used)
    new_log = replace(robot.logistics, fuel=new_fuel)
    return replace(robot, logistics=new_log)


def consume_ammo(robot: Robot, weapon: str, rounds: int = 1) -> Robot:
    """Decrement ammo for a weapon. No-op if logistics is unset."""
    if robot.logistics is None or rounds <= 0:
        return robot
    current = robot.logistics.ammo.get(weapon, 0)
    new_ammo = dict(robot.logistics.ammo)
    new_ammo[weapon] = max(0, current - rounds)
    new_log = replace(robot.logistics, ammo=new_ammo)
    return replace(robot, logistics=new_log)


def consume_water(robot: Robot, amount: float) -> Robot:
    if robot.logistics is None or amount <= 0:
        return robot
    new_water = max(0.0, robot.logistics.water - amount)
    new_log = replace(robot.logistics, water=new_water)
    return replace(robot, logistics=new_log)


def is_immobile(robot: Robot) -> bool:
    """A unit with logistics tracking and zero fuel can't move."""
    return robot.logistics is not None and robot.logistics.fuel <= 0.0


def can_fire(robot: Robot, weapon: str) -> bool:
    """Returns True if the weapon has remaining ammo (or no logistics tracked)."""
    if robot.logistics is None:
        return True
    return robot.logistics.ammo.get(weapon, 0) > 0


def resupply(robot: Robot, items: dict) -> Robot:
    """Refill specified items.

    Items dict format:
        {"fuel": 1.0}                       — set fuel to full
        {"water": 1.0}                      — set water to full
        {"ammo": {"M4_carbine": 30, ...}}   — replace ammo counts (additive)
    """
    if robot.logistics is None:
        return robot
    new_log = robot.logistics
    if "fuel" in items:
        new_log = replace(new_log, fuel=min(1.0, float(items["fuel"])))
    if "water" in items:
        new_log = replace(new_log, water=min(1.0, float(items["water"])))
    if "ammo" in items and isinstance(items["ammo"], dict):
        new_ammo = dict(new_log.ammo)
        for weapon, rounds in items["ammo"].items():
            new_ammo[weapon] = new_ammo.get(weapon, 0) + int(rounds)
        new_log = replace(new_log, ammo=new_ammo)
    return replace(robot, logistics=new_log)


def logistics_from_dict(data: dict) -> Logistics:
    """Build a Logistics from YAML-loaded dict."""
    return Logistics(
        fuel=float(data.get("fuel", 1.0)),
        ammo={str(k): int(v) for k, v in (data.get("ammo") or {}).items()},
        water=float(data.get("water", 1.0)),
        fuel_rate_per_km=float(data.get("fuel_rate_per_km", 0.05)),
    )
