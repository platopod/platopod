"""Tests for plato_pod.sensor_plugins.gazebo_bridge — Gazebo sensor passthrough."""

from __future__ import annotations

import math

import pytest

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import ArenaState, SensorConfig
from plato_pod.sensor_plugins.gazebo_bridge import GazeboImuBridge, GazeboLidarBridge


def _robot() -> Robot:
    return Robot(robot_id=1, deployment="virtual", x=0.5, y=0.3, radius=0.028)


def _arena() -> ArenaState:
    return ArenaState(boundary=((0, 0), (1, 0), (1, 1), (0, 1)), obstacles=[])


class TestGazeboLidarBridge:
    def test_no_data_returns_stub(self) -> None:
        bridge = GazeboLidarBridge()
        config = SensorConfig(params={"range_max": 2.0, "angular_resolution_deg": 1.0})
        data = bridge.compute(_robot(), _arena(), [], config)
        assert all(r == 2.0 for r in data["ranges"])
        assert len(data["ranges"]) == 360

    def test_has_data_false_initially(self) -> None:
        bridge = GazeboLidarBridge()
        assert bridge.has_data() is False

    def test_update_from_scan(self) -> None:
        bridge = GazeboLidarBridge()
        ranges = [1.0, 0.5, 2.0, 1.5]
        bridge.update_from_scan(
            ranges=ranges,
            angle_min=0.0,
            angle_max=math.pi,
            angle_increment=math.pi / 4,
            range_min=0.01,
            range_max=2.0,
        )
        assert bridge.has_data() is True

        data = bridge.compute(_robot(), _arena(), [], SensorConfig())
        assert data["ranges"] == [1.0, 0.5, 2.0, 1.5]
        assert data["angle_min"] == 0.0
        assert data["angle_max"] == pytest.approx(math.pi)
        assert data["range_max"] == 2.0

    def test_update_overwrites_previous(self) -> None:
        bridge = GazeboLidarBridge()
        bridge.update_from_scan([1.0], 0, 1, 1, 0.01, 2.0)
        bridge.update_from_scan([0.5], 0, 1, 1, 0.01, 2.0)
        data = bridge.compute(_robot(), _arena(), [], SensorConfig())
        assert data["ranges"] == [0.5]

    def test_compute_returns_copy(self) -> None:
        bridge = GazeboLidarBridge()
        bridge.update_from_scan([1.0, 2.0], 0, 1, 0.5, 0.01, 3.0)
        d1 = bridge.compute(_robot(), _arena(), [], SensorConfig())
        d2 = bridge.compute(_robot(), _arena(), [], SensorConfig())
        d1["ranges"][0] = 999.0
        assert d2["ranges"][0] == 1.0  # not mutated

    def test_default_config(self) -> None:
        bridge = GazeboLidarBridge()
        cfg = bridge.default_config()
        assert cfg.rate_hz == 10.0
        assert cfg.params["range_max"] == 2.0


class TestGazeboImuBridge:
    def test_no_data_returns_gravity(self) -> None:
        bridge = GazeboImuBridge()
        data = bridge.compute(_robot(), _arena(), [], SensorConfig())
        assert data["linear_acceleration"]["z"] == pytest.approx(9.81)
        assert data["angular_velocity"]["x"] == 0.0
        assert data["orientation"]["w"] == 1.0

    def test_has_data_false_initially(self) -> None:
        bridge = GazeboImuBridge()
        assert bridge.has_data() is False

    def test_update_from_imu(self) -> None:
        bridge = GazeboImuBridge()
        bridge.update_from_imu(
            linear_acceleration=(0.1, -0.2, 9.78),
            angular_velocity=(0.01, 0.0, 0.5),
            orientation=(0.0, 0.0, 0.383, 0.924),
        )
        assert bridge.has_data() is True

        data = bridge.compute(_robot(), _arena(), [], SensorConfig())
        assert data["linear_acceleration"]["x"] == pytest.approx(0.1)
        assert data["linear_acceleration"]["z"] == pytest.approx(9.78)
        assert data["angular_velocity"]["z"] == pytest.approx(0.5)
        assert data["orientation"]["z"] == pytest.approx(0.383)
        assert data["orientation"]["w"] == pytest.approx(0.924)

    def test_default_config(self) -> None:
        bridge = GazeboImuBridge()
        cfg = bridge.default_config()
        assert cfg.rate_hz == 50.0
