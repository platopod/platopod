"""Tests for plato_pod.command_pipeline — velocity filtering pipeline."""

from __future__ import annotations

import math

import pytest

from plato_pod.command_pipeline import (
    PipelineResult,
    boundary_filter,
    collision_filter,
    mobility_filter,
    predict_position,
    run_pipeline,
    speed_limit,
    state_filter,
)
from plato_pod.robot import (
    Robot,
    STATUS_ACTIVE,
    STATUS_DESTROYED,
    STATUS_FROZEN,
    STATUS_INCAPACITATED,
    STATUS_WOUNDED,
)

UNIT_BOUNDARY = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))


def _robot(x: float = 0.5, y: float = 0.5, theta: float = 0.0,
           robot_id: int = 1, radius: float = 0.028) -> Robot:
    return Robot(robot_id=robot_id, deployment="virtual", x=x, y=y, theta=theta, radius=radius)


# --- speed_limit ---

class TestSpeedLimit:
    def test_within_limits_no_event(self) -> None:
        r = speed_limit(0.1, 0.5, 0.2, 2.0, robot_id=1)
        assert r.linear_x == pytest.approx(0.1)
        assert r.angular_z == pytest.approx(0.5)
        assert len(r.events) == 0

    def test_linear_clamped(self) -> None:
        r = speed_limit(0.5, 0.0, 0.2, 2.0, robot_id=1)
        assert r.linear_x == pytest.approx(0.2)
        assert len(r.events) == 1
        assert r.events[0].event == "command_clamped"

    def test_angular_clamped(self) -> None:
        r = speed_limit(0.0, 5.0, 0.2, 2.0, robot_id=1)
        assert r.angular_z == pytest.approx(2.0)
        assert len(r.events) == 1

    def test_negative_clamped(self) -> None:
        r = speed_limit(-0.5, -5.0, 0.2, 2.0, robot_id=1)
        assert r.linear_x == pytest.approx(-0.2)
        assert r.angular_z == pytest.approx(-2.0)

    def test_zero_no_event(self) -> None:
        r = speed_limit(0.0, 0.0, 0.2, 2.0, robot_id=1)
        assert len(r.events) == 0


# --- predict_position ---

class TestPredictPosition:
    def test_straight_forward(self) -> None:
        x, y, th = predict_position(0, 0, 0, 0.1, 0.0, 1.0)
        assert x == pytest.approx(0.1)
        assert y == pytest.approx(0.0)
        assert th == pytest.approx(0.0)

    def test_straight_at_angle(self) -> None:
        x, y, th = predict_position(0, 0, math.pi / 2, 0.1, 0.0, 1.0)
        assert x == pytest.approx(0.0, abs=1e-10)
        assert y == pytest.approx(0.1)

    def test_pure_rotation(self) -> None:
        x, y, th = predict_position(0.5, 0.5, 0.0, 0.0, 1.0, 0.5)
        assert x == pytest.approx(0.5)
        assert y == pytest.approx(0.5)
        assert th == pytest.approx(0.5)

    def test_zero_velocity(self) -> None:
        x, y, th = predict_position(1, 2, 3, 0, 0, 1.0)
        assert x == pytest.approx(1)
        assert y == pytest.approx(2)
        assert th == pytest.approx(3)

    def test_short_dt(self) -> None:
        x, y, th = predict_position(0, 0, 0, 0.1, 0.0, 0.1)
        assert x == pytest.approx(0.01)


# --- boundary_filter ---

class TestBoundaryFilter:
    def test_safe_movement_passes(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = boundary_filter(0.1, 0.0, robot, UNIT_BOUNDARY, [])
        assert r.linear_x == pytest.approx(0.1)
        assert len(r.events) == 0

    def test_movement_toward_boundary_clamped(self) -> None:
        # Robot near right edge, moving right
        robot = _robot(0.97, 0.5, 0.0)
        r = boundary_filter(0.5, 0.0, robot, UNIT_BOUNDARY, [])
        assert r.linear_x < 0.5
        assert len(r.events) == 1
        assert r.events[0].event == "boundary_contact"
        assert r.events[0].detail["element"] == "perimeter"

    def test_zero_velocity_passes(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = boundary_filter(0.0, 0.0, robot, UNIT_BOUNDARY, [])
        assert r.linear_x == pytest.approx(0.0)
        assert len(r.events) == 0

    def test_no_boundary_passes(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = boundary_filter(0.5, 0.0, robot, (), [])
        assert r.linear_x == pytest.approx(0.5)

    def test_obstacle_blocking(self) -> None:
        robot = _robot(0.3, 0.5, 0.0)
        # Obstacle directly ahead
        obstacle = [
            (0.35, 0.45), (0.45, 0.45), (0.45, 0.55), (0.35, 0.55)
        ]
        r = boundary_filter(0.5, 0.0, robot, UNIT_BOUNDARY, [tuple(obstacle)])
        assert r.linear_x < 0.5
        assert len(r.events) == 1
        assert r.events[0].detail["element"] == "obstacle"

    def test_movement_away_from_boundary_passes(self) -> None:
        # Robot near right edge, moving LEFT (theta=pi)
        robot = _robot(0.97, 0.5, math.pi)
        r = boundary_filter(0.1, 0.0, robot, UNIT_BOUNDARY, [])
        assert r.linear_x == pytest.approx(0.1)
        assert len(r.events) == 0


# --- collision_filter ---

class TestCollisionFilter:
    def test_no_other_robots_passes(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = collision_filter(0.1, 0.0, robot, [])
        assert r.linear_x == pytest.approx(0.1)

    def test_collision_imminent_clamped(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        other = _robot(0.55, 0.5, 0.0, robot_id=2)
        r = collision_filter(0.5, 0.0, robot, [other])
        assert r.linear_x < 0.5
        assert len(r.events) == 1
        assert r.events[0].event == "collision_contact"
        assert r.events[0].detail["other_robot_id"] == 2

    def test_far_robot_no_collision(self) -> None:
        robot = _robot(0.2, 0.5, 0.0)
        other = _robot(0.8, 0.5, 0.0, robot_id=2)
        r = collision_filter(0.1, 0.0, robot, [other])
        assert r.linear_x == pytest.approx(0.1)
        assert len(r.events) == 0

    def test_zero_velocity_passes(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        other = _robot(0.55, 0.5, 0.0, robot_id=2)
        r = collision_filter(0.0, 0.0, robot, [other])
        assert len(r.events) == 0

    def test_self_excluded(self) -> None:
        robot = _robot(0.5, 0.5, 0.0, robot_id=1)
        # Same robot_id in other_robots should be ignored
        r = collision_filter(0.1, 0.0, robot, [robot])
        assert r.linear_x == pytest.approx(0.1)


# --- run_pipeline ---

class TestRunPipeline:
    def test_unconstrained_passes(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = run_pipeline(0.1, 0.5, robot, [], UNIT_BOUNDARY, [])
        assert r.linear_x == pytest.approx(0.1)
        assert r.angular_z == pytest.approx(0.5)
        assert len(r.events) == 0

    def test_speed_clamped(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = run_pipeline(0.5, 0.0, robot, [], UNIT_BOUNDARY, [], max_linear=0.2)
        assert r.linear_x == pytest.approx(0.2)
        assert any(e.event == "command_clamped" for e in r.events)

    def test_boundary_and_speed(self) -> None:
        robot = _robot(0.97, 0.5, 0.0)
        r = run_pipeline(0.5, 0.0, robot, [], UNIT_BOUNDARY, [], max_linear=0.3)
        # Speed clamped first, then boundary
        assert r.linear_x < 0.3
        events = [e.event for e in r.events]
        assert "command_clamped" in events
        assert "boundary_contact" in events

    def test_zero_velocity_no_events(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = run_pipeline(0.0, 0.0, robot, [], UNIT_BOUNDARY, [])
        assert len(r.events) == 0

    def test_terrain_modifier_halves_speed(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = run_pipeline(
            0.2, 1.0, robot, [], UNIT_BOUNDARY, [],
            terrain_speed_modifier=0.5,
        )
        assert r.linear_x == pytest.approx(0.1)
        assert r.angular_z == pytest.approx(0.5)

    def test_terrain_modifier_one_no_change(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = run_pipeline(0.1, 0.5, robot, [], UNIT_BOUNDARY, [])
        r_mod = run_pipeline(
            0.1, 0.5, robot, [], UNIT_BOUNDARY, [],
            terrain_speed_modifier=1.0,
        )
        assert r.linear_x == pytest.approx(r_mod.linear_x)
        assert r.angular_z == pytest.approx(r_mod.angular_z)

    def test_terrain_modifier_zero_stops_robot(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        r = run_pipeline(
            0.2, 1.0, robot, [], UNIT_BOUNDARY, [],
            terrain_speed_modifier=0.0,
        )
        assert r.linear_x == pytest.approx(0.0)
        assert r.angular_z == pytest.approx(0.0)

    def test_terrain_modifier_clamped_to_valid_range(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        # Negative modifier should be clamped to 0
        r = run_pipeline(
            0.2, 1.0, robot, [], UNIT_BOUNDARY, [],
            terrain_speed_modifier=-0.5,
        )
        assert r.linear_x == pytest.approx(0.0)
        assert r.angular_z == pytest.approx(0.0)

    def test_terrain_modifier_applied_after_speed_limit(self) -> None:
        robot = _robot(0.5, 0.5, 0.0)
        # Speed limit clamps to 0.2, then terrain halves to 0.1
        r = run_pipeline(
            0.5, 0.0, robot, [], UNIT_BOUNDARY, [],
            max_linear=0.2, terrain_speed_modifier=0.5,
        )
        assert r.linear_x == pytest.approx(0.1)


# --- state_filter ---

class TestStateFilter:
    def test_active_passes(self) -> None:
        robot = _robot()
        r = state_filter(0.1, 0.5, robot)
        assert r.linear_x == pytest.approx(0.1)
        assert r.angular_z == pytest.approx(0.5)
        assert len(r.events) == 0

    def test_destroyed_rejects(self) -> None:
        robot = Robot(robot_id=1, deployment="virtual", status=STATUS_DESTROYED, health=0.0)
        r = state_filter(0.5, 1.0, robot)
        assert r.linear_x == 0.0
        assert r.angular_z == 0.0
        assert len(r.events) == 1
        assert r.events[0].event == "command_rejected"
        assert r.events[0].detail["reason"] == STATUS_DESTROYED

    def test_frozen_rejects(self) -> None:
        robot = Robot(robot_id=1, deployment="virtual", status=STATUS_FROZEN)
        r = state_filter(0.1, 0.0, robot)
        assert r.linear_x == 0.0
        assert r.events[0].detail["reason"] == STATUS_FROZEN

    def test_incapacitated_rejects(self) -> None:
        robot = Robot(robot_id=1, deployment="virtual", status=STATUS_INCAPACITATED)
        r = state_filter(0.1, 0.0, robot)
        assert r.linear_x == 0.0


# --- mobility_filter ---

class TestMobilityFilter:
    def test_full_health_no_scaling(self) -> None:
        robot = _robot()  # default health=1.0
        r = mobility_filter(0.2, 1.0, robot)
        assert r.linear_x == pytest.approx(0.2)
        assert r.angular_z == pytest.approx(1.0)
        assert len(r.events) == 0

    def test_wounded_scales_down(self) -> None:
        robot = Robot(robot_id=1, deployment="virtual", health=0.5)
        r = mobility_filter(0.2, 1.0, robot)
        # mobility_factor(0.5) = 0.5
        assert r.linear_x == pytest.approx(0.1)
        assert r.angular_z == pytest.approx(0.5)
        assert len(r.events) == 1
        assert r.events[0].event == "mobility_reduced"

    def test_destroyed_zero(self) -> None:
        robot = Robot(robot_id=1, deployment="virtual", health=0.0,
                      status=STATUS_DESTROYED)
        r = mobility_filter(0.2, 0.0, robot)
        assert r.linear_x == 0.0

    def test_zero_command_no_event(self) -> None:
        # Don't emit noisy "mobility_reduced" events when there's no command
        robot = Robot(robot_id=1, deployment="virtual", health=0.5)
        r = mobility_filter(0.0, 0.0, robot)
        assert len(r.events) == 0


class TestFuelFilter:
    def test_no_logistics_passthrough(self) -> None:
        from plato_pod.command_pipeline import fuel_filter
        robot = _robot()
        r = fuel_filter(0.2, 1.0, robot)
        assert r.linear_x == pytest.approx(0.2)
        assert len(r.events) == 0

    def test_empty_fuel_blocks(self) -> None:
        from plato_pod.command_pipeline import fuel_filter
        from plato_pod.logistics import Logistics
        robot = Robot(robot_id=1, deployment="virtual",
                       logistics=Logistics(fuel=0.0))
        r = fuel_filter(0.2, 1.0, robot)
        assert r.linear_x == 0.0
        assert r.events[0].event == "out_of_fuel"

    def test_fuel_remaining_passthrough(self) -> None:
        from plato_pod.command_pipeline import fuel_filter
        from plato_pod.logistics import Logistics
        robot = Robot(robot_id=1, deployment="virtual",
                       logistics=Logistics(fuel=0.5))
        r = fuel_filter(0.2, 1.0, robot)
        assert r.linear_x == pytest.approx(0.2)


# --- run_pipeline integration with state/mobility ---

class TestRunPipelineWithState:
    def test_destroyed_short_circuits(self) -> None:
        robot = Robot(robot_id=1, deployment="virtual", x=0.5, y=0.5,
                      health=0.0, status=STATUS_DESTROYED, radius=0.028)
        r = run_pipeline(0.5, 1.0, robot, [], UNIT_BOUNDARY, [])
        assert r.linear_x == 0.0
        assert r.angular_z == 0.0
        # Should have command_rejected event
        events = [e.event for e in r.events]
        assert "command_rejected" in events
        # Boundary/collision filters should not have run
        assert "boundary_contact" not in events
        assert "collision_contact" not in events

    def test_wounded_scales_then_pipelined(self) -> None:
        robot = Robot(robot_id=1, deployment="virtual", x=0.5, y=0.5,
                      health=0.5, status=STATUS_WOUNDED, radius=0.028)
        r = run_pipeline(0.2, 0.0, robot, [], UNIT_BOUNDARY, [], max_linear=0.2)
        # mobility halves 0.2 to 0.1; speed limit (0.2) doesn't clamp
        assert r.linear_x == pytest.approx(0.1)
        events = [e.event for e in r.events]
        assert "mobility_reduced" in events
