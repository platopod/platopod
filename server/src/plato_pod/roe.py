"""Rules of engagement evaluator.

Pure functions checking whether a fire intent complies with the active ROE.
Returns a list of violations (may be empty). The engagement node decides
whether to permit, warn, or block based on rule severity.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from plato_pod.engagement import Civilian
from plato_pod.robot import Robot

# Standard NATO permission levels (highest to lowest restriction)
WEAPONS_HOLD = "weapons_hold"        # fire only in self-defence
WEAPONS_TIGHT = "weapons_tight"      # fire on positively identified hostiles only
WEAPONS_FREE = "weapons_free"        # fire at any non-friendly target


@dataclass
class ROERules:
    """Active rules of engagement for the exercise."""
    fire_permission: str = WEAPONS_FREE
    civilian_proximity_m: float = 10.0
    require_target_id: bool = False        # must target be in cleared_targets?
    friendly_fire_severity: str = "violation"   # "warning" | "violation" | "critical"


@dataclass(frozen=True)
class ROEViolation:
    rule: str
    severity: str           # "warning" | "violation" | "critical"
    description: str


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def check_fire_roe(
    actor: Robot | None,
    target_position: tuple[float, float],
    rules: ROERules,
    target: Robot | None = None,
    civilians: list[Civilian] | None = None,
    cleared_targets: set[int] | None = None,
) -> list[ROEViolation]:
    """Evaluate ROE compliance for a fire intent.

    Returns:
        List of ROEViolation objects (empty if fully compliant).
    """
    violations: list[ROEViolation] = []
    civs = civilians or []
    cleared = cleared_targets or set()

    # 1. Permission level
    if rules.fire_permission == WEAPONS_HOLD:
        violations.append(ROEViolation(
            rule="weapons_hold",
            severity="critical",
            description="Fire forbidden under WEAPONS HOLD (self-defence only)",
        ))

    # 2. Friendly fire
    if actor is not None and target is not None and target.team is not None:
        if target.team == actor.team:
            violations.append(ROEViolation(
                rule="friendly_fire",
                severity=rules.friendly_fire_severity,
                description=(
                    f"Target {target.robot_id} is on actor's team "
                    f"'{target.team}'"
                ),
            ))

    # 3. Civilian proximity
    for civ in civs:
        if _distance(civ.position, target_position) <= rules.civilian_proximity_m:
            violations.append(ROEViolation(
                rule="civilian_proximity",
                severity="violation",
                description=(
                    f"Civilian '{civ.label or 'unidentified'}' within "
                    f"{rules.civilian_proximity_m:.0f}m of target"
                ),
            ))

    # 4. Target identification (weapons_tight or explicit require_target_id)
    needs_clearance = (
        rules.require_target_id
        or rules.fire_permission == WEAPONS_TIGHT
    )
    if needs_clearance and target is not None:
        if target.robot_id not in cleared:
            violations.append(ROEViolation(
                rule="unconfirmed_target",
                severity="warning",
                description=(
                    f"Target {target.robot_id} not positively identified — "
                    f"clearance required under {rules.fire_permission}"
                ),
            ))

    return violations


def is_blocking(violations: list[ROEViolation]) -> bool:
    """A violation list blocks the engagement if any item is critical."""
    return any(v.severity == "critical" for v in violations)


def roe_from_dict(data: dict) -> ROERules:
    return ROERules(
        fire_permission=str(data.get("fire_permission", WEAPONS_FREE)),
        civilian_proximity_m=float(data.get("civilian_proximity_m", 10.0)),
        require_target_id=bool(data.get("require_target_id", False)),
        friendly_fire_severity=str(data.get("friendly_fire", "violation")),
    )
