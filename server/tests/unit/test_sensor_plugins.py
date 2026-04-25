"""Tests for plato_pod.sensor_plugins — sensor plugin implementations."""

from __future__ import annotations

import math

import pytest

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    SensorConfig,
    apply_dropout,
    apply_noise,
)
from plato_pod.sensor_plugins.fof import FofSensor
from plato_pod.sensor_plugins.gps import GpsSensor
from plato_pod.sensor_plugins.lidar import LidarSensor
from plato_pod.sensor_plugins.sonar import SonarSensor


def _robot(x: float = 0.5, y: float = 0.3, theta: float = 0.0) -> Robot:
    return Robot(robot_id=1, deployment="virtual", x=x, y=y, theta=theta, radius=0.028)


def _arena() -> ArenaState:
    return ArenaState(
        boundary=((0, 0), (1, 0), (1, 1), (0, 1)),
        obstacles=[],
    )


# --- apply_noise / apply_dropout ---

class TestNoiseUtilities:
    def test_zero_noise_unchanged(self) -> None:
        assert apply_noise(1.0, 0.0) == 1.0

    def test_nonzero_noise_changes(self) -> None:
        values = {round(apply_noise(1.0, 0.1), 6) for _ in range(20)}
        assert len(values) > 1

    def test_zero_dropout_unchanged(self) -> None:
        vals = [1.0, 2.0, 3.0]
        assert apply_dropout(vals, 0.0) == vals

    def test_full_dropout(self) -> None:
        vals = [1.0, 2.0, 3.0]
        result = apply_dropout(vals, 1.0)
        assert all(v == float('inf') for v in result)


# --- GPS sensor ---

class TestGpsSensor:
    def test_no_noise_returns_exact_pose(self) -> None:
        gps = GpsSensor()
        config = SensorConfig(noise_stddev=0.0, params={"heading_noise_stddev": 0.0})
        data = gps.compute(_robot(0.5, 0.3, 1.0), _arena(), [], config)
        assert data["x"] == pytest.approx(0.5)
        assert data["y"] == pytest.approx(0.3)
        assert data["theta"] == pytest.approx(1.0)
        assert data["origin_tag_id"] == 101

    def test_with_noise_changes_values(self) -> None:
        gps = GpsSensor()
        config = SensorConfig(noise_stddev=0.01, params={"heading_noise_stddev": 0.1})
        xs = {round(gps.compute(_robot(), _arena(), [], config)["x"], 6) for _ in range(20)}
        assert len(xs) > 1

    def test_default_config(self) -> None:
        gps = GpsSensor()
        cfg = gps.default_config()
        assert cfg.noise_stddev == 0.001
        assert cfg.rate_hz == 10.0

    def test_theta_normalized(self) -> None:
        gps = GpsSensor()
        config = SensorConfig(noise_stddev=0.0, params={"heading_noise_stddev": 0.0})
        data = gps.compute(_robot(theta=3.14), _arena(), [], config)
        assert -math.pi <= data["theta"] <= math.pi


# --- Lidar stub ---

class TestLidarSensor:
    def test_returns_max_range(self) -> None:
        lidar = LidarSensor()
        config = lidar.default_config()
        data = lidar.compute(_robot(), _arena(), [], config)
        assert all(r == 2.0 for r in data["ranges"])
        assert len(data["ranges"]) == 360
        assert data["range_max"] == 2.0

    def test_custom_resolution(self) -> None:
        lidar = LidarSensor()
        config = SensorConfig(params={"range_max": 1.0, "angular_resolution_deg": 5.0})
        data = lidar.compute(_robot(), _arena(), [], config)
        assert len(data["ranges"]) == 72  # 360/5


# --- Sonar stub ---

class TestSonarSensor:
    def test_returns_max_range(self) -> None:
        sonar = SonarSensor()
        config = sonar.default_config()
        data = sonar.compute(_robot(), _arena(), [], config)
        assert all(r == 1.5 for r in data["ranges"])
        assert len(data["ranges"]) == 36  # 360/10
        assert data["segment_angle_deg"] == 10.0


# --- FoF stub ---

class TestFofSensor:
    def test_returns_empty_detections(self) -> None:
        fof = FofSensor()
        config = fof.default_config()
        data = fof.compute(_robot(), _arena(), [], config)
        assert data["detections"] == []
        assert data["range_max"] == 1.0
