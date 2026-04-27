"""Virtual unit AI — finite state machine for OPFOR units.

Each virtual hostile unit runs an instance of BehaviorTree, ticked once per
control cycle. The FSM produces Actions (movement and/or fire) based on the
unit's state and observations of the world.

States: PATROL, OBSERVE, ENGAGE, RETREAT, INCAPACITATED.

Pure Python — testable without ROS2. The opfor_node provides the runtime
loop and ROS2 plumbing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from plato_pod.engagement import WeaponSpec
from plato_pod.line_of_sight import CoverPolygon, has_line_of_sight
from plato_pod.robot import Robot
from plato_pod.weather import CLEAR, WeatherState


class BehaviorState(Enum):
    PATROL = "patrol"
    OBSERVE = "observe"
    ENGAGE = "engage"
    RETREAT = "retreat"
    INCAPACITATED = "incapacitated"


@dataclass
class BehaviorConfig:
    """Per-unit behaviour parameters loaded from YAML."""
    behavior: str = "patrol_then_defend"   # behaviour preset name
    patrol_route: list[tuple[float, float]] = field(default_factory=list)
    defense_position: tuple[float, float] | None = None
    defense_radius_m: float = 50.0
    detection_range_m: float = 200.0
    engagement_range_m: float = 150.0
    retreat_health_threshold: float = 0.4
    retreat_position: tuple[float, float] | None = None
    waypoint_arrival_radius_m: float = 5.0
    max_linear_speed: float = 2.0           # m/s
    max_angular_speed: float = 1.5          # rad/s
    enemy_teams: list[str] = field(default_factory=lambda: ["blue"])
    weapon: str | None = None               # primary weapon name (lookup in weapons dict)


@dataclass
class WorldState:
    """Snapshot of the world from a unit's perspective.

    The opfor_node populates this from /robots/status and the loaded
    exercise configuration.
    """
    all_units: list[Robot]
    cover_polygons: list[CoverPolygon] = field(default_factory=list)
    weather: WeatherState = field(default_factory=lambda: CLEAR)
    weapons: dict[str, WeaponSpec] = field(default_factory=dict)
    arena_bounds: tuple[float, float, float, float] | None = None  # (xmin, ymin, xmax, ymax)


@dataclass
class Action:
    """Action produced by a single FSM tick."""
    cmd_vel: tuple[float, float] = (0.0, 0.0)   # (linear, angular)
    fire_weapon: tuple[int, str] | None = None   # (target_id, weapon_name)
    state_change: BehaviorState | None = None
    rationale: str = ""


def _angle_to(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    """Bearing from (from) to (to), in radians, 0=east, CCW."""
    return math.atan2(to_y - from_y, to_x - from_x)


def _angle_diff(target_angle: float, current_angle: float) -> float:
    """Normalised angular difference in [-pi, pi]."""
    diff = target_angle - current_angle
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def _move_toward(unit: Robot, target_x: float, target_y: float,
                  cfg: BehaviorConfig) -> tuple[float, float]:
    """Compute (linear, angular) to move unit toward a target point.

    Simple proportional controller: turn first, then move forward.
    """
    desired_heading = _angle_to(unit.x, unit.y, target_x, target_y)
    heading_error = _angle_diff(desired_heading, unit.theta)

    # Turn first
    angular = max(-cfg.max_angular_speed, min(cfg.max_angular_speed, heading_error * 2.0))

    # Move forward only when roughly facing target
    if abs(heading_error) > math.pi / 4:
        linear = 0.0
    else:
        # Proportional to alignment
        alignment = math.cos(heading_error)
        linear = cfg.max_linear_speed * max(0.0, alignment)

    return (linear, angular)


def _move_away_from(unit: Robot, threat_x: float, threat_y: float,
                     cfg: BehaviorConfig) -> tuple[float, float]:
    """Move directly away from a threat at full speed."""
    flee_heading = _angle_to(threat_x, threat_y, unit.x, unit.y)
    heading_error = _angle_diff(flee_heading, unit.theta)
    angular = max(-cfg.max_angular_speed, min(cfg.max_angular_speed, heading_error * 2.0))
    if abs(heading_error) > math.pi / 4:
        linear = 0.0
    else:
        linear = cfg.max_linear_speed
    return (linear, angular)


def _distance(a: Robot | tuple[float, float], b: Robot | tuple[float, float]) -> float:
    ax = a.x if isinstance(a, Robot) else a[0]
    ay = a.y if isinstance(a, Robot) else a[1]
    bx = b.x if isinstance(b, Robot) else b[0]
    by = b.y if isinstance(b, Robot) else b[1]
    return math.hypot(ax - bx, ay - by)


def find_visible_enemies(
    unit: Robot, world: WorldState, cfg: BehaviorConfig,
) -> list[tuple[Robot, float]]:
    """Return enemies within detection range and with line of sight.

    Returns list of (enemy, distance) sorted by distance ascending.
    """
    candidates: list[tuple[Robot, float]] = []
    for other in world.all_units:
        if other.robot_id == unit.robot_id:
            continue
        if other.team not in cfg.enemy_teams:
            continue
        if not other.is_operational():
            continue
        d = _distance(unit, other)
        if d > cfg.detection_range_m:
            continue
        # LoS check
        los = has_line_of_sight(
            (unit.x, unit.y, 1.0),
            (other.x, other.y, 1.0),
            cover_polygons=world.cover_polygons,
            weather=world.weather,
        )
        if not los.visible:
            continue
        candidates.append((other, d))

    candidates.sort(key=lambda x: x[1])
    return candidates


class BehaviorTree:
    """FSM-based behaviour for a single virtual unit.

    One instance per OPFOR unit. State is mutable and persists across ticks.
    """

    def __init__(self, config: BehaviorConfig) -> None:
        self.config = config
        self.state: BehaviorState = BehaviorState.PATROL
        self.waypoint_index: int = 0
        self.target_id: int | None = None
        self.last_target_seen_time: float = 0.0
        self.tick_count: int = 0

    def tick(self, unit: Robot, world: WorldState) -> Action:
        """Single FSM step. Returns the action to take this tick."""
        self.tick_count += 1

        # Health-based forced transitions take precedence
        if not unit.is_operational():
            self.state = BehaviorState.INCAPACITATED
            return Action(cmd_vel=(0.0, 0.0), rationale="incapacitated_or_destroyed")

        if unit.health < self.config.retreat_health_threshold and \
           self.state not in (BehaviorState.RETREAT, BehaviorState.INCAPACITATED):
            return self._enter_retreat(unit, world)

        # Dispatch to state handler
        handlers = {
            BehaviorState.PATROL: self._tick_patrol,
            BehaviorState.OBSERVE: self._tick_observe,
            BehaviorState.ENGAGE: self._tick_engage,
            BehaviorState.RETREAT: self._tick_retreat,
            BehaviorState.INCAPACITATED: lambda _u, _w: Action(rationale="terminal_state"),
        }
        return handlers[self.state](unit, world)

    def _tick_patrol(self, unit: Robot, world: WorldState) -> Action:
        # Check for enemies — transition to OBSERVE if any
        enemies = find_visible_enemies(unit, world, self.config)
        if enemies:
            self.target_id = enemies[0][0].robot_id
            self.state = BehaviorState.OBSERVE
            return Action(cmd_vel=(0.0, 0.0), state_change=BehaviorState.OBSERVE,
                          rationale=f"enemy_spotted_id_{self.target_id}")

        # No enemies — continue patrol
        if not self.config.patrol_route:
            return Action(cmd_vel=(0.0, 0.0), rationale="no_patrol_route")

        target = self.config.patrol_route[self.waypoint_index % len(self.config.patrol_route)]
        d = _distance(unit, target)
        if d < self.config.waypoint_arrival_radius_m:
            # Advance to next waypoint
            self.waypoint_index += 1
            return Action(cmd_vel=(0.0, 0.0),
                          rationale=f"waypoint_{self.waypoint_index}_reached")

        lin, ang = _move_toward(unit, target[0], target[1], self.config)
        return Action(cmd_vel=(lin, ang), rationale=f"patrol_to_wp_{self.waypoint_index}")

    def _tick_observe(self, unit: Robot, world: WorldState) -> Action:
        enemies = find_visible_enemies(unit, world, self.config)
        if not enemies:
            # Lost sight — back to patrol
            self.target_id = None
            self.state = BehaviorState.PATROL
            return Action(cmd_vel=(0.0, 0.0), state_change=BehaviorState.PATROL,
                          rationale="enemy_lost")

        # Pick closest enemy
        target, distance = enemies[0]
        self.target_id = target.robot_id

        # Within engagement range? → ENGAGE
        if distance <= self.config.engagement_range_m:
            self.state = BehaviorState.ENGAGE
            return Action(cmd_vel=(0.0, 0.0), state_change=BehaviorState.ENGAGE,
                          rationale=f"engaging_target_{target.robot_id}")

        # Otherwise hold position, face the target
        desired_heading = _angle_to(unit.x, unit.y, target.x, target.y)
        heading_error = _angle_diff(desired_heading, unit.theta)
        angular = max(-self.config.max_angular_speed,
                      min(self.config.max_angular_speed, heading_error * 2.0))
        return Action(cmd_vel=(0.0, angular), rationale="observing_target")

    def _tick_engage(self, unit: Robot, world: WorldState) -> Action:
        # Re-acquire target
        target = next((r for r in world.all_units if r.robot_id == self.target_id), None)
        if target is None or not target.is_operational():
            # Target gone — back to patrol via observe
            self.state = BehaviorState.PATROL
            self.target_id = None
            return Action(cmd_vel=(0.0, 0.0), state_change=BehaviorState.PATROL,
                          rationale="target_eliminated_or_lost")

        distance = _distance(unit, target)

        # Lost LoS or out of range → OBSERVE
        if distance > self.config.engagement_range_m:
            self.state = BehaviorState.OBSERVE
            return Action(cmd_vel=(0.0, 0.0), state_change=BehaviorState.OBSERVE,
                          rationale="target_out_of_range")

        los = has_line_of_sight(
            (unit.x, unit.y, 1.0),
            (target.x, target.y, 1.0),
            cover_polygons=world.cover_polygons,
            weather=world.weather,
        )
        if not los.visible:
            self.state = BehaviorState.OBSERVE
            return Action(cmd_vel=(0.0, 0.0), state_change=BehaviorState.OBSERVE,
                          rationale=f"target_obscured_{los.rationale}")

        # Face target
        desired_heading = _angle_to(unit.x, unit.y, target.x, target.y)
        heading_error = _angle_diff(desired_heading, unit.theta)
        angular = max(-self.config.max_angular_speed,
                      min(self.config.max_angular_speed, heading_error * 2.0))

        # Fire when roughly aligned (otherwise just turn)
        fire = None
        if abs(heading_error) < math.pi / 6 and self.config.weapon:
            fire = (target.robot_id, self.config.weapon)

        return Action(cmd_vel=(0.0, angular), fire_weapon=fire,
                      rationale=f"engaging_{target.robot_id}")

    def _tick_retreat(self, unit: Robot, world: WorldState) -> Action:
        # If we have a designated retreat position, head there
        if self.config.retreat_position is not None:
            target = self.config.retreat_position
            d = _distance(unit, target)
            if d < self.config.waypoint_arrival_radius_m:
                # Reached safety — stay put, observe
                return Action(cmd_vel=(0.0, 0.0), rationale="retreat_complete")
            lin, ang = _move_toward(unit, target[0], target[1], self.config)
            return Action(cmd_vel=(lin, ang), rationale="retreating_to_safe_position")

        # Otherwise, flee from nearest enemy
        enemies = find_visible_enemies(unit, world, self.config)
        if not enemies:
            return Action(cmd_vel=(0.0, 0.0), rationale="retreat_complete_no_threats")

        nearest = enemies[0][0]
        lin, ang = _move_away_from(unit, nearest.x, nearest.y, self.config)
        return Action(cmd_vel=(lin, ang),
                      rationale=f"fleeing_from_{nearest.robot_id}")

    def _enter_retreat(self, unit: Robot, world: WorldState) -> Action:
        self.state = BehaviorState.RETREAT
        return Action(cmd_vel=(0.0, 0.0), state_change=BehaviorState.RETREAT,
                      rationale=f"low_health_{unit.health:.2f}_retreating")


def behavior_config_from_dict(data: dict) -> BehaviorConfig:
    """Build a BehaviorConfig from a YAML-loaded dict."""
    return BehaviorConfig(
        behavior=data.get("behavior", "patrol_then_defend"),
        patrol_route=[tuple(p) for p in data.get("patrol_route", [])],
        defense_position=(tuple(data["defense_position"])
                          if "defense_position" in data else None),
        defense_radius_m=float(data.get("defense_radius_m", 50.0)),
        detection_range_m=float(data.get("detection_range_m", 200.0)),
        engagement_range_m=float(data.get("engagement_range_m", 150.0)),
        retreat_health_threshold=float(data.get("retreat_health_threshold", 0.4)),
        retreat_position=(tuple(data["retreat_position"])
                          if "retreat_position" in data else None),
        waypoint_arrival_radius_m=float(data.get("waypoint_arrival_radius_m", 5.0)),
        max_linear_speed=float(data.get("max_linear_speed", 2.0)),
        max_angular_speed=float(data.get("max_angular_speed", 1.5)),
        enemy_teams=list(data.get("enemy_teams", ["blue"])),
        weapon=data.get("weapon"),
    )
