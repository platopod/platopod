"""Casualty and mobility model.

Maps robot health (0.0–1.0) to operational consequences: movement speed,
fire accuracy, and status transitions. Pure functions — no ROS2 dependency.

Health thresholds:
    1.0 .. 0.5    → status=active, full mobility, full accuracy
    0.5 .. 0.0    → status=wounded, reduced mobility and accuracy
    <= 0.0        → status=destroyed, no movement, no fire
"""

from __future__ import annotations

from dataclasses import replace

from plato_pod.robot import (
    Robot,
    STATUS_ACTIVE,
    STATUS_DESTROYED,
    STATUS_INCAPACITATED,
    STATUS_WOUNDED,
)

# Thresholds
HEALTH_DESTROYED = 0.0
HEALTH_WOUNDED = 0.5      # below this, robot is "wounded"
HEALTH_CRITICAL = 0.2     # below this, fire accuracy collapses


def mobility_factor(health: float) -> float:
    """Velocity scaling factor based on health.

    1.0 → no scaling (full speed)
    0.5..1.0 → linear interpolation 1.0 → 0.5
    0.0..0.5 → linear interpolation 0.5 → 0.1 (crawling)
    <=0.0 → 0.0 (no movement)
    """
    if health <= HEALTH_DESTROYED:
        return 0.0
    if health < HEALTH_WOUNDED:
        # Linear: at h=0.5 → 0.5, at h=0.0 → 0.1
        return 0.1 + (health / HEALTH_WOUNDED) * 0.4
    # Linear: at h=1.0 → 1.0, at h=0.5 → 0.5
    return health


def fire_capability(health: float) -> float:
    """Accuracy scaling factor based on health.

    1.0 → full accuracy
    0.5..1.0 → unchanged
    0.2..0.5 → degrades 1.0 → 0.5
    0.0..0.2 → degrades 0.5 → 0.0
    <=0.0 → 0.0 (cannot fire)
    """
    if health <= HEALTH_DESTROYED:
        return 0.0
    if health < HEALTH_CRITICAL:
        # 0..0.2 → 0..0.5
        return health / HEALTH_CRITICAL * 0.5
    if health < HEALTH_WOUNDED:
        # 0.2..0.5 → 0.5..1.0
        return 0.5 + (health - HEALTH_CRITICAL) / (HEALTH_WOUNDED - HEALTH_CRITICAL) * 0.5
    return 1.0


def status_for_health(current_status: str, health: float) -> str:
    """Compute the status that corresponds to a given health value.

    Preserves non-health statuses (incapacitated, frozen) — those override
    the health-based status until explicitly cleared.
    """
    # Don't override non-health statuses
    if current_status == STATUS_INCAPACITATED:
        return STATUS_INCAPACITATED
    # Health-based transitions
    if health <= HEALTH_DESTROYED:
        return STATUS_DESTROYED
    if health < HEALTH_WOUNDED:
        return STATUS_WOUNDED
    return STATUS_ACTIVE


def apply_damage(robot: Robot, damage: float) -> Robot:
    """Return a new Robot with reduced health and updated status.

    Damage is clamped non-negative; resulting health is clamped 0..1.
    """
    if damage < 0:
        damage = 0.0
    new_health = max(HEALTH_DESTROYED, robot.health - damage)
    new_status = status_for_health(robot.status, new_health)
    return replace(robot, health=new_health, status=new_status)


def heal(robot: Robot, amount: float) -> Robot:
    """Return a new Robot with restored health and updated status.

    Used for medic actions, resupply, or scenario reset.
    """
    if amount < 0:
        amount = 0.0
    new_health = min(1.0, robot.health + amount)
    new_status = status_for_health(robot.status, new_health)
    return replace(robot, health=new_health, status=new_status)


def incapacitate(robot: Robot, reason: str = "cbrn_exposure") -> Robot:
    """Mark robot as incapacitated (e.g., from CBRN exposure).

    Distinct from kinetic damage — incapacitated robots still have nominal
    health but cannot move or fire. Reversible via heal/reset.
    """
    return replace(robot, status=STATUS_INCAPACITATED)


def gas_exposure_damage(
    concentration: float,
    threshold: float = 100.0,
    rate_per_unit_per_second: float = 0.001,
    dt: float = 0.1,
) -> float:
    """Compute incremental damage from gas concentration over a timestep.

    Below threshold: no damage. Above: damage proportional to overage.

    Args:
        concentration: gas concentration at robot position (ppm or arbitrary)
        threshold: exposure threshold (no damage below this)
        rate_per_unit_per_second: damage per (unit-of-concentration above
            threshold) per second
        dt: time step in seconds

    Returns:
        Damage to apply this step (0.0 to 1.0).
    """
    if concentration <= threshold:
        return 0.0
    overage = concentration - threshold
    return overage * rate_per_unit_per_second * dt
