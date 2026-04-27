"""Tests for plato_pod.behavior — OPFOR FSM."""

from __future__ import annotations

import math

import pytest

from plato_pod.behavior import (
    Action,
    BehaviorConfig,
    BehaviorState,
    BehaviorTree,
    WorldState,
    behavior_config_from_dict,
    find_visible_enemies,
)
from plato_pod.engagement import WeaponSpec
from plato_pod.line_of_sight import CoverPolygon
from plato_pod.robot import Robot
from plato_pod.weather import CLEAR


def _opfor(robot_id: int = 100, x: float = 0.0, y: float = 0.0,
           theta: float = 0.0, health: float = 1.0,
           team: str = "red", status: str = "active") -> Robot:
    return Robot(
        robot_id=robot_id, deployment="virtual",
        x=x, y=y, theta=theta, health=health, team=team, status=status,
    )


def _blue(robot_id: int = 1, x: float = 0.0, y: float = 0.0,
          health: float = 1.0, status: str = "active") -> Robot:
    return Robot(
        robot_id=robot_id, deployment="virtual",
        x=x, y=y, health=health, team="blue", status=status,
    )


def _cfg(**overrides) -> BehaviorConfig:
    base = dict(
        patrol_route=[(50.0, 0.0), (50.0, 50.0), (0.0, 50.0)],
        detection_range_m=200.0,
        engagement_range_m=150.0,
        retreat_health_threshold=0.4,
        max_linear_speed=2.0,
        max_angular_speed=1.5,
        enemy_teams=["blue"],
        weapon="AK74",
    )
    base.update(overrides)
    return BehaviorConfig(**base)


def _world(units: list[Robot], cover=None, weather=CLEAR,
            weapons=None) -> WorldState:
    return WorldState(
        all_units=units,
        cover_polygons=cover or [],
        weather=weather,
        weapons=weapons or {"AK74": WeaponSpec(
            name="AK74", max_range_m=400, base_pok_at_100m=0.5,
            falloff_per_100m=0.1, damage=1.0, ammo_capacity=30)},
    )


class TestFindVisibleEnemies:
    def test_no_enemies(self) -> None:
        opfor = _opfor()
        world = _world([opfor])
        cfg = _cfg()
        assert find_visible_enemies(opfor, world, cfg) == []

    def test_friendly_excluded(self) -> None:
        opfor = _opfor()
        friend = _opfor(robot_id=101, x=10, y=10)  # also red
        world = _world([opfor, friend])
        assert find_visible_enemies(opfor, world, _cfg()) == []

    def test_self_excluded(self) -> None:
        opfor = _opfor()
        world = _world([opfor])
        assert find_visible_enemies(opfor, world, _cfg()) == []

    def test_enemy_in_range_visible(self) -> None:
        opfor = _opfor(x=0, y=0)
        enemy = _blue(x=50, y=0)
        world = _world([opfor, enemy])
        seen = find_visible_enemies(opfor, world, _cfg())
        assert len(seen) == 1
        assert seen[0][0].robot_id == 1
        assert seen[0][1] == pytest.approx(50.0)

    def test_enemy_out_of_range(self) -> None:
        opfor = _opfor()
        enemy = _blue(x=500, y=0)  # beyond 200m detection
        world = _world([opfor, enemy])
        assert find_visible_enemies(opfor, world, _cfg()) == []

    def test_enemy_blocked_by_cover(self) -> None:
        opfor = _opfor(x=0, y=0)
        enemy = _blue(x=100, y=0)
        cover = [CoverPolygon(
            vertices=[(40, -10), (60, -10), (60, 10), (40, 10)],
            cover_value=1.0, label="wall",
        )]
        world = _world([opfor, enemy], cover=cover)
        assert find_visible_enemies(opfor, world, _cfg()) == []

    def test_destroyed_enemy_excluded(self) -> None:
        opfor = _opfor()
        enemy = _blue(x=50, y=0, status="destroyed", health=0.0)
        world = _world([opfor, enemy])
        assert find_visible_enemies(opfor, world, _cfg()) == []

    def test_sorted_by_distance(self) -> None:
        opfor = _opfor()
        far = _blue(robot_id=1, x=100, y=0)
        near = _blue(robot_id=2, x=30, y=0)
        world = _world([opfor, far, near])
        seen = find_visible_enemies(opfor, world, _cfg())
        assert [r.robot_id for r, _ in seen] == [2, 1]


class TestBehaviorTreeInit:
    def test_starts_in_patrol(self) -> None:
        tree = BehaviorTree(_cfg())
        assert tree.state == BehaviorState.PATROL
        assert tree.target_id is None
        assert tree.waypoint_index == 0


class TestPatrolState:
    def test_no_route_idles(self) -> None:
        tree = BehaviorTree(_cfg(patrol_route=[]))
        opfor = _opfor()
        action = tree.tick(opfor, _world([opfor]))
        assert action.cmd_vel == (0.0, 0.0)
        assert "no_patrol_route" in action.rationale

    def test_moves_toward_first_waypoint(self) -> None:
        # Route: (50, 0) so unit at origin facing +x should drive forward
        tree = BehaviorTree(_cfg())
        opfor = _opfor(x=0, y=0, theta=0.0)
        action = tree.tick(opfor, _world([opfor]))
        assert action.cmd_vel[0] > 0  # linear forward

    def test_advances_on_arrival(self) -> None:
        tree = BehaviorTree(_cfg())
        # Already at first waypoint
        opfor = _opfor(x=50, y=0)
        action = tree.tick(opfor, _world([opfor]))
        assert tree.waypoint_index == 1
        assert "reached" in action.rationale

    def test_transitions_to_observe_on_enemy_sighting(self) -> None:
        tree = BehaviorTree(_cfg())
        opfor = _opfor(x=0, y=0)
        enemy = _blue(x=80, y=0)
        action = tree.tick(opfor, _world([opfor, enemy]))
        assert tree.state == BehaviorState.OBSERVE
        assert action.state_change == BehaviorState.OBSERVE
        assert tree.target_id == 1


class TestObserveState:
    def test_engages_when_in_range(self) -> None:
        tree = BehaviorTree(_cfg(engagement_range_m=100.0))
        tree.state = BehaviorState.OBSERVE
        opfor = _opfor()
        enemy = _blue(x=80, y=0)  # within 100m engagement
        action = tree.tick(opfor, _world([opfor, enemy]))
        assert tree.state == BehaviorState.ENGAGE
        assert action.state_change == BehaviorState.ENGAGE

    def test_holds_position_when_out_of_engage_range(self) -> None:
        tree = BehaviorTree(_cfg(engagement_range_m=50.0))
        tree.state = BehaviorState.OBSERVE
        opfor = _opfor(x=0, y=0, theta=0.0)
        enemy = _blue(x=0, y=100)  # north of unit; engagement range is 50
        action = tree.tick(opfor, _world([opfor, enemy]))
        # Should stay at position (linear=0) but turn toward target
        assert action.cmd_vel[0] == 0.0
        assert action.cmd_vel[1] != 0.0
        assert tree.state == BehaviorState.OBSERVE

    def test_returns_to_patrol_when_lost(self) -> None:
        tree = BehaviorTree(_cfg())
        tree.state = BehaviorState.OBSERVE
        tree.target_id = 1
        opfor = _opfor()
        action = tree.tick(opfor, _world([opfor]))  # no enemies
        assert tree.state == BehaviorState.PATROL
        assert tree.target_id is None
        assert action.state_change == BehaviorState.PATROL


class TestEngageState:
    def test_fires_when_aligned(self) -> None:
        tree = BehaviorTree(_cfg(engagement_range_m=200.0))
        tree.state = BehaviorState.ENGAGE
        tree.target_id = 1
        opfor = _opfor(x=0, y=0, theta=0.0)
        enemy = _blue(x=50, y=0)  # directly east
        action = tree.tick(opfor, _world([opfor, enemy]))
        assert action.fire_weapon == (1, "AK74")

    def test_no_fire_when_misaligned(self) -> None:
        tree = BehaviorTree(_cfg(engagement_range_m=200.0))
        tree.state = BehaviorState.ENGAGE
        tree.target_id = 1
        opfor = _opfor(x=0, y=0, theta=math.pi)  # facing west
        enemy = _blue(x=50, y=0)  # east — facing wrong way
        action = tree.tick(opfor, _world([opfor, enemy]))
        assert action.fire_weapon is None
        # but turns toward target
        assert abs(action.cmd_vel[1]) > 0

    def test_no_fire_without_weapon_configured(self) -> None:
        tree = BehaviorTree(_cfg(weapon=None, engagement_range_m=200.0))
        tree.state = BehaviorState.ENGAGE
        tree.target_id = 1
        opfor = _opfor(x=0, y=0, theta=0.0)
        enemy = _blue(x=50, y=0)
        action = tree.tick(opfor, _world([opfor, enemy]))
        assert action.fire_weapon is None

    def test_target_destroyed_returns_to_patrol(self) -> None:
        tree = BehaviorTree(_cfg())
        tree.state = BehaviorState.ENGAGE
        tree.target_id = 1
        opfor = _opfor()
        dead = _blue(x=50, y=0, status="destroyed", health=0.0)
        tree.tick(opfor, _world([opfor, dead]))
        assert tree.state == BehaviorState.PATROL
        assert tree.target_id is None

    def test_target_out_of_range_back_to_observe(self) -> None:
        tree = BehaviorTree(_cfg(engagement_range_m=50.0))
        tree.state = BehaviorState.ENGAGE
        tree.target_id = 1
        opfor = _opfor(x=0, y=0, theta=0.0)
        enemy = _blue(x=100, y=0)  # beyond engage range
        action = tree.tick(opfor, _world([opfor, enemy]))
        assert tree.state == BehaviorState.OBSERVE
        assert action.state_change == BehaviorState.OBSERVE

    def test_target_obscured_back_to_observe(self) -> None:
        tree = BehaviorTree(_cfg(engagement_range_m=200.0))
        tree.state = BehaviorState.ENGAGE
        tree.target_id = 1
        opfor = _opfor(x=0, y=0, theta=0.0)
        enemy = _blue(x=100, y=0)
        cover = [CoverPolygon(
            vertices=[(40, -10), (60, -10), (60, 10), (40, 10)],
            cover_value=1.0, label="wall",
        )]
        tree.tick(opfor, _world([opfor, enemy], cover=cover))
        assert tree.state == BehaviorState.OBSERVE


class TestRetreatTransition:
    def test_low_health_triggers_retreat(self) -> None:
        tree = BehaviorTree(_cfg(retreat_health_threshold=0.4))
        opfor = _opfor(health=0.3)
        action = tree.tick(opfor, _world([opfor]))
        assert tree.state == BehaviorState.RETREAT
        assert action.state_change == BehaviorState.RETREAT

    def test_retreat_threshold_not_crossed_continues(self) -> None:
        tree = BehaviorTree(_cfg(retreat_health_threshold=0.4))
        opfor = _opfor(health=0.5)
        tree.tick(opfor, _world([opfor]))
        assert tree.state == BehaviorState.PATROL


class TestRetreatState:
    def test_moves_to_designated_position(self) -> None:
        tree = BehaviorTree(_cfg(retreat_position=(0.0, 100.0)))
        tree.state = BehaviorState.RETREAT
        opfor = _opfor(x=0, y=0, theta=math.pi / 2, health=0.3)  # facing north
        action = tree.tick(opfor, _world([opfor]))
        assert action.cmd_vel[0] > 0  # moving forward toward (0, 100)

    def test_retreat_complete_stops(self) -> None:
        tree = BehaviorTree(_cfg(retreat_position=(0.0, 100.0)))
        tree.state = BehaviorState.RETREAT
        opfor = _opfor(x=0, y=100, health=0.3)
        action = tree.tick(opfor, _world([opfor]))
        assert action.cmd_vel == (0.0, 0.0)
        assert "retreat_complete" in action.rationale

    def test_flees_from_nearest_enemy_without_position(self) -> None:
        tree = BehaviorTree(_cfg(retreat_position=None))
        tree.state = BehaviorState.RETREAT
        # facing east; enemy to the east → flee west
        opfor = _opfor(x=0, y=0, theta=math.pi, health=0.3)
        enemy = _blue(x=50, y=0)
        action = tree.tick(opfor, _world([opfor, enemy]))
        assert action.cmd_vel[0] > 0  # moving forward (which is west)

    def test_no_threats_idle(self) -> None:
        tree = BehaviorTree(_cfg(retreat_position=None))
        tree.state = BehaviorState.RETREAT
        opfor = _opfor(health=0.3)
        action = tree.tick(opfor, _world([opfor]))
        assert action.cmd_vel == (0.0, 0.0)


class TestIncapacitatedState:
    def test_destroyed_unit_does_nothing(self) -> None:
        tree = BehaviorTree(_cfg())
        opfor = _opfor(status="destroyed", health=0.0)
        action = tree.tick(opfor, _world([opfor]))
        assert tree.state == BehaviorState.INCAPACITATED
        assert action.cmd_vel == (0.0, 0.0)
        assert action.fire_weapon is None

    def test_incapacitated_status_blocks(self) -> None:
        tree = BehaviorTree(_cfg())
        opfor = _opfor(status="incapacitated")
        tree.tick(opfor, _world([opfor]))
        assert tree.state == BehaviorState.INCAPACITATED


class TestTickCounter:
    def test_increments_each_tick(self) -> None:
        tree = BehaviorTree(_cfg())
        opfor = _opfor()
        world = _world([opfor])
        tree.tick(opfor, world)
        tree.tick(opfor, world)
        tree.tick(opfor, world)
        assert tree.tick_count == 3


class TestBehaviorConfigFromDict:
    def test_minimal(self) -> None:
        cfg = behavior_config_from_dict({})
        assert cfg.behavior == "patrol_then_defend"
        assert cfg.patrol_route == []
        assert cfg.retreat_health_threshold == 0.4

    def test_full(self) -> None:
        cfg = behavior_config_from_dict({
            "behavior": "ambush",
            "patrol_route": [[0.0, 0.0], [10.0, 10.0]],
            "defense_position": [50.0, 50.0],
            "defense_radius_m": 30,
            "detection_range_m": 250,
            "engagement_range_m": 200,
            "retreat_health_threshold": 0.3,
            "retreat_position": [100.0, 100.0],
            "max_linear_speed": 3.0,
            "max_angular_speed": 2.0,
            "enemy_teams": ["blue", "green"],
            "weapon": "PKM",
        })
        assert cfg.behavior == "ambush"
        assert cfg.patrol_route == [(0.0, 0.0), (10.0, 10.0)]
        assert cfg.defense_position == (50.0, 50.0)
        assert cfg.retreat_position == (100.0, 100.0)
        assert cfg.weapon == "PKM"
        assert cfg.enemy_teams == ["blue", "green"]


class TestActionDataclass:
    def test_default_action_is_idle(self) -> None:
        a = Action()
        assert a.cmd_vel == (0.0, 0.0)
        assert a.fire_weapon is None
        assert a.state_change is None
