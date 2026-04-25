"""Tests for plato_pod.providers.gazebo_provider — Gazebo localization."""

from __future__ import annotations

import math

import pytest

from plato_pod.pose import PoseSource
from plato_pod.providers.gazebo_provider import GazeboProvider, quaternion_to_yaw


class TestQuaternionToYaw:
    def test_identity_is_zero(self) -> None:
        assert quaternion_to_yaw(0, 0, 0, 1) == pytest.approx(0.0)

    def test_90_degrees(self) -> None:
        # 90° rotation around Z: qw=cos(45°), qz=sin(45°)
        qw = math.cos(math.pi / 4)
        qz = math.sin(math.pi / 4)
        assert quaternion_to_yaw(0, 0, qz, qw) == pytest.approx(math.pi / 2, abs=0.01)

    def test_180_degrees(self) -> None:
        yaw = quaternion_to_yaw(0, 0, 1, 0)  # 180° around Z
        assert abs(yaw) == pytest.approx(math.pi, abs=0.01)

    def test_negative_90(self) -> None:
        qw = math.cos(-math.pi / 4)
        qz = math.sin(-math.pi / 4)
        assert quaternion_to_yaw(0, 0, qz, qw) == pytest.approx(-math.pi / 2, abs=0.01)


class TestGazeboProvider:
    def test_source_is_gazebo_sim(self) -> None:
        p = GazeboProvider()
        assert p.source == PoseSource.GAZEBO_SIM

    def test_get_poses_empty_initially(self) -> None:
        p = GazeboProvider()
        assert p.get_poses() == []

    def test_update_and_get_pose(self) -> None:
        p = GazeboProvider()
        p.update_pose(1, 2.0, 3.0, 0.0, 0, 0, 0, 1)
        poses = p.get_poses()
        assert len(poses) == 1
        assert poses[0].robot_id == 1
        assert poses[0].x == pytest.approx(2.0)
        assert poses[0].y == pytest.approx(3.0)
        assert poses[0].source == PoseSource.GAZEBO_SIM

    def test_quaternion_converted_to_theta(self) -> None:
        p = GazeboProvider()
        qw = math.cos(math.pi / 4)
        qz = math.sin(math.pi / 4)
        p.update_pose(1, 0, 0, 0, 0, 0, qz, qw)
        poses = p.get_poses()
        assert poses[0].theta == pytest.approx(math.pi / 2, abs=0.01)

    def test_multiple_robots(self) -> None:
        p = GazeboProvider()
        p.update_pose(1, 1.0, 0, 0, 0, 0, 0, 1)
        p.update_pose(2, 2.0, 0, 0, 0, 0, 0, 1)
        assert len(p.get_poses()) == 2

    def test_remove_robot(self) -> None:
        p = GazeboProvider()
        p.update_pose(1, 0, 0, 0, 0, 0, 0, 1)
        p.remove_robot(1)
        assert len(p.get_poses()) == 0

    def test_confidence_is_1(self) -> None:
        p = GazeboProvider()
        p.update_pose(1, 0, 0, 0, 0, 0, 0, 1)
        assert p.get_poses()[0].confidence == 1.0
