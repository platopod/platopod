"""Tests for plato_pod.robot_registry — robot registry and spawn validation."""

from __future__ import annotations

import math

import pytest

from plato_pod.pose import PoseSource
from plato_pod.robot_registry import (
    DEFAULT_RADIUS,
    Registry,
    Robot,
    validate_spawn_position,
)

UNIT_BOUNDARY = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
_CAM = PoseSource.CAMERA_ARTAG


# --- Robot ---

class TestRobot:
    def test_fields(self) -> None:
        e = Robot(robot_id=1, deployment="physical", x=0.1, y=0.2, theta=1.5,
                  radius=0.028, status="active", localization_id="3")
        assert e.robot_id == 1
        assert e.deployment == "physical"
        assert e.localization_id == "3"

    def test_mutable(self) -> None:
        e = Robot(robot_id=1, deployment="virtual")
        e.x = 0.5
        assert e.x == 0.5

    def test_default_values(self) -> None:
        e = Robot(robot_id=1, deployment="virtual")
        assert e.radius == 0.028
        assert e.status == "active"
        assert e.localization_id == ""
        assert e.team is None


# --- validate_spawn_position ---

class TestValidateSpawnPosition:
    def test_inside_boundary_valid(self) -> None:
        valid, msg = validate_spawn_position(0.5, 0.5, 0.028, UNIT_BOUNDARY, [])
        assert valid is True
        assert msg == ""

    def test_outside_boundary_invalid(self) -> None:
        valid, msg = validate_spawn_position(5.0, 5.0, 0.028, UNIT_BOUNDARY, [])
        assert valid is False
        assert "out_of_bounds" in msg

    def test_edge_extends_outside(self) -> None:
        # Centre at (0.01, 0.5), radius 0.028 — circle extends past x=0
        valid, msg = validate_spawn_position(0.01, 0.5, 0.028, UNIT_BOUNDARY, [])
        assert valid is False
        assert "out_of_bounds" in msg

    def test_no_boundary_invalid(self) -> None:
        valid, msg = validate_spawn_position(0.5, 0.5, 0.028, (), [])
        assert valid is False
        assert "not yet established" in msg

    def test_collision_with_existing_robot(self) -> None:
        existing = [Robot(robot_id=1, deployment="virtual", x=0.5, y=0.5,
                          theta=0.0, radius=0.028, status="active")]
        # Try to spawn at (0.52, 0.5) — distance 0.02, less than 0.028+0.028=0.056
        valid, msg = validate_spawn_position(0.52, 0.5, 0.028, UNIT_BOUNDARY, existing)
        assert valid is False
        assert "collision" in msg
        assert "robot 1" in msg

    def test_no_collision_with_clearance(self) -> None:
        existing = [Robot(robot_id=1, deployment="virtual", x=0.5, y=0.5,
                          theta=0.0, radius=0.028, status="active")]
        # Spawn at (0.7, 0.5) — distance 0.2, well above 0.056
        valid, msg = validate_spawn_position(0.7, 0.5, 0.028, UNIT_BOUNDARY, existing)
        assert valid is True

    def test_inactive_robots_ignored(self) -> None:
        existing = [Robot(robot_id=1, deployment="virtual", x=0.5, y=0.5,
                          theta=0.0, radius=0.028, status="inactive")]
        valid, msg = validate_spawn_position(0.5, 0.5, 0.028, UNIT_BOUNDARY, existing)
        assert valid is True

    def test_exact_touching_valid(self) -> None:
        existing = [Robot(robot_id=1, deployment="virtual", x=0.5, y=0.5,
                          theta=0.0, radius=0.028, status="active")]
        # Distance exactly = 0.056 (sum of radii) — touching but not overlapping
        dist = 0.056
        valid, msg = validate_spawn_position(
            0.5 + dist, 0.5, 0.028, UNIT_BOUNDARY, existing
        )
        assert valid is True


# --- Registry ---

class TestRegistry:
    def test_register_physical_success(self) -> None:
        reg = Registry()
        ok, robot_id, msg = reg.register_physical("3", _CAM, 0.028, 0.5, 0.5, 0.0)
        assert ok is True
        assert robot_id >= 1
        assert msg == "OK"

    def test_register_physical_duplicate_active_rejected(self) -> None:
        reg = Registry()
        reg.register_physical("3", _CAM, 0.028, 0.5, 0.5, 0.0)
        ok, _, msg = reg.register_physical("3", _CAM, 0.028, 0.6, 0.6, 0.0)
        assert ok is False
        assert "DUPLICATE" in msg

    def test_register_physical_restores_inactive(self) -> None:
        reg = Registry()
        ok, original_id, _ = reg.register_physical("3", _CAM, 0.028, 0.5, 0.5, 0.0)
        assert ok is True
        reg.mark_inactive(original_id)

        ok, restored_id, msg = reg.register_physical("3", _CAM, 0.028, 0.6, 0.6, 1.0)
        assert ok is True
        assert restored_id == original_id
        entry = reg.get(restored_id)
        assert entry is not None
        assert entry.status == "active"
        assert entry.x == pytest.approx(0.6)

    def test_spawn_virtual_success(self) -> None:
        reg = Registry()
        ok, robot_id, msg = reg.spawn_virtual(0.5, 0.5, 0.0, 0.028, UNIT_BOUNDARY)
        assert ok is True
        assert robot_id >= 1
        entry = reg.get(robot_id)
        assert entry is not None
        assert entry.deployment == "virtual"
        assert entry.localization_id == ""

    def test_spawn_virtual_out_of_bounds(self) -> None:
        reg = Registry()
        ok, _, msg = reg.spawn_virtual(5.0, 5.0, 0.0, 0.028, UNIT_BOUNDARY)
        assert ok is False
        assert "out_of_bounds" in msg

    def test_spawn_virtual_collision(self) -> None:
        reg = Registry()
        reg.spawn_virtual(0.5, 0.5, 0.0, 0.028, UNIT_BOUNDARY)
        ok, _, msg = reg.spawn_virtual(0.52, 0.5, 0.0, 0.028, UNIT_BOUNDARY)
        assert ok is False
        assert "collision" in msg

    def test_spawn_virtual_default_radius(self) -> None:
        reg = Registry()
        ok, robot_id, _ = reg.spawn_virtual(0.5, 0.5, 0.0, 0.0, UNIT_BOUNDARY)
        assert ok is True
        entry = reg.get(robot_id)
        assert entry is not None
        assert entry.radius == pytest.approx(DEFAULT_RADIUS)

    def test_spawn_virtual_no_boundary(self) -> None:
        reg = Registry()
        ok, _, msg = reg.spawn_virtual(0.5, 0.5, 0.0, 0.028, ())
        assert ok is False
        assert "not yet established" in msg

    def test_remove_virtual_success(self) -> None:
        reg = Registry()
        _, robot_id, _ = reg.spawn_virtual(0.5, 0.5, 0.0, 0.028, UNIT_BOUNDARY)
        ok, msg = reg.remove_robot(robot_id)
        assert ok is True
        assert reg.get(robot_id) is None

    def test_remove_physical_rejected(self) -> None:
        reg = Registry()
        _, robot_id, _ = reg.register_physical("3", _CAM, 0.028, 0.5, 0.5, 0.0)
        ok, msg = reg.remove_robot(robot_id)
        assert ok is False
        assert "physical" in msg.lower()

    def test_remove_nonexistent_rejected(self) -> None:
        reg = Registry()
        ok, msg = reg.remove_robot(999)
        assert ok is False

    def test_reset_physical_success(self) -> None:
        reg = Registry()
        _, robot_id, _ = reg.register_physical("3", _CAM, 0.028, 0.5, 0.5, 0.0)
        ok, msg = reg.reset_robot(robot_id)
        assert ok is True
        entry = reg.get(robot_id)
        assert entry is not None
        assert entry.status == "inactive"

    def test_reset_virtual_rejected(self) -> None:
        reg = Registry()
        _, robot_id, _ = reg.spawn_virtual(0.5, 0.5, 0.0, 0.028, UNIT_BOUNDARY)
        ok, msg = reg.reset_robot(robot_id)
        assert ok is False

    def test_list_robots(self) -> None:
        reg = Registry()
        reg.register_physical("3", _CAM, 0.028, 0.5, 0.5, 0.0)
        reg.spawn_virtual(0.2, 0.2, 0.0, 0.028, UNIT_BOUNDARY)
        robots = reg.list_robots()
        assert len(robots) == 2

    def test_list_robots_empty(self) -> None:
        reg = Registry()
        assert reg.list_robots() == []

    def test_update_pose(self) -> None:
        reg = Registry()
        _, robot_id, _ = reg.register_physical("3", _CAM, 0.028, 0.5, 0.5, 0.0)
        ok = reg.update_pose(robot_id, 0.6, 0.7, 1.5)
        assert ok is True
        entry = reg.get(robot_id)
        assert entry is not None
        assert entry.x == pytest.approx(0.6)
        assert entry.y == pytest.approx(0.7)
        assert entry.theta == pytest.approx(1.5)

    def test_update_pose_nonexistent(self) -> None:
        reg = Registry()
        assert reg.update_pose(999, 0, 0, 0) is False

    def test_mark_inactive(self) -> None:
        reg = Registry()
        _, robot_id, _ = reg.register_physical("3", _CAM, 0.028, 0.5, 0.5, 0.0)
        ok = reg.mark_inactive(robot_id)
        assert ok is True
        assert reg.get(robot_id).status == "inactive"

    def test_sequential_ids(self) -> None:
        reg = Registry()
        _, id1, _ = reg.register_physical("1", _CAM, 0.028, 0.2, 0.2, 0.0)
        _, id2, _ = reg.spawn_virtual(0.5, 0.5, 0.0, 0.028, UNIT_BOUNDARY)
        _, id3, _ = reg.register_physical("2", _CAM, 0.028, 0.8, 0.8, 0.0)
        assert id2 == id1 + 1
        assert id3 == id2 + 1

    def test_find_by_localization_id(self) -> None:
        reg = Registry()
        _, robot_id, _ = reg.register_physical("7", _CAM, 0.028, 0.5, 0.5, 0.0)
        entry = reg.find_by_localization_id("7")
        assert entry is not None
        assert entry.robot_id == robot_id

    def test_find_by_localization_id_not_found(self) -> None:
        reg = Registry()
        assert reg.find_by_localization_id("99") is None
