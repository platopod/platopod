"""Tests for the Phase-3 sensor plugins.

rangefinder, thermal, ied_detector, df_receiver, uav_camera.
"""

from __future__ import annotations

import math

import pytest

from plato_pod.line_of_sight import CoverPolygon
from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState, EnvironmentContext, SensorConfig,
)
from plato_pod.sensor_plugins.df_receiver import DfReceiverSensor
from plato_pod.sensor_plugins.ied_detector import IedDetectorSensor
from plato_pod.sensor_plugins.rangefinder import RangefinderSensor
from plato_pod.sensor_plugins.thermal import ThermalSensor
from plato_pod.sensor_plugins.uav_camera import UavCameraSensor
from plato_pod.weather import CLEAR, WeatherState


def _bot(rid: int = 1, x: float = 0.0, y: float = 0.0,
         theta: float = 0.0, team: str | None = "blue",
         status: str = "active", health: float = 1.0,
         thermal: float = 1.0,
         vehicle_role: str = "default") -> Robot:
    return Robot(
        robot_id=rid, deployment="virtual",
        x=x, y=y, theta=theta,
        team=team, status=status, health=health,
        thermal_signature=thermal, vehicle_role=vehicle_role,
    )


_EMPTY_ARENA = ArenaState(boundary=(), obstacles=[])


# ─────────────────────────── Rangefinder ──────────────────────────────────

class TestRangefinder:
    plugin = RangefinderSensor()

    def _cfg(self, **overrides) -> SensorConfig:
        cfg = self.plugin.default_config()
        cfg.params.update(overrides)
        return cfg

    def test_no_targets(self) -> None:
        out = self.plugin.compute(
            _bot(), _EMPTY_ARENA, [], self._cfg(),
        )
        assert out["range_m"] is None
        assert out["target_id"] is None

    def test_returns_nearest_visible(self) -> None:
        observer = _bot(theta=0.0)
        near = _bot(rid=2, x=50, y=0, team="red")
        far = _bot(rid=3, x=200, y=0, team="red")
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [near, far],
            self._cfg(enemy_teams=["red"], fov_deg=20.0, accuracy_m=0.0),
        )
        assert out["target_id"] == 2
        assert out["range_m"] == pytest.approx(50.0, abs=0.1)

    def test_fov_excludes_off_axis(self) -> None:
        observer = _bot(theta=0.0)  # facing +x
        side = _bot(rid=2, x=0, y=100, team="red")  # 90° off
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [side],
            self._cfg(enemy_teams=["red"], fov_deg=20.0),
        )
        assert out["range_m"] is None

    def test_max_range_excludes(self) -> None:
        observer = _bot(theta=0.0)
        far = _bot(rid=2, x=2000, y=0, team="red")
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [far],
            self._cfg(enemy_teams=["red"], max_range_m=1500.0, fov_deg=180.0),
        )
        assert out["range_m"] is None

    def test_los_blocked_by_cover(self) -> None:
        observer = _bot(theta=0.0)
        target = _bot(rid=2, x=100, y=0, team="red")
        cover = [CoverPolygon(
            vertices=[(40, -10), (60, -10), (60, 10), (40, 10)],
            cover_value=1.0, label="wall",
        )]
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [target],
            self._cfg(enemy_teams=["red"], fov_deg=180.0,
                      cover_polygons=cover, weather=CLEAR),
        )
        assert out["range_m"] is None
        assert out["reason"] == "los_blocked"

    def test_destroyed_target_skipped(self) -> None:
        observer = _bot(theta=0.0)
        dead = _bot(rid=2, x=50, y=0, team="red", status="destroyed", health=0.0)
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [dead],
            self._cfg(enemy_teams=["red"], fov_deg=180.0),
        )
        assert out["range_m"] is None


# ─────────────────────────── Thermal ──────────────────────────────────────

class TestThermal:
    plugin = ThermalSensor()

    def _cfg(self, **overrides) -> SensorConfig:
        cfg = self.plugin.default_config()
        cfg.params.update(overrides)
        return cfg

    def test_warm_unit_detected(self) -> None:
        observer = _bot(theta=0.0)
        warm = _bot(rid=2, x=100, y=0, thermal=0.8)
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [warm], self._cfg(fov_deg=180.0),
        )
        assert len(out["detections"]) == 1
        d = out["detections"][0]
        assert d["robot_id"] == 2
        assert d["range_m"] == pytest.approx(100.0)
        assert d["intensity"] > 0.0

    def test_cold_unit_filtered(self) -> None:
        observer = _bot(theta=0.0)
        cold = _bot(rid=2, x=100, y=0, thermal=0.0)
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [cold], self._cfg(fov_deg=180.0),
        )
        assert out["detections"] == []

    def test_cover_blocks_thermal(self) -> None:
        observer = _bot(theta=0.0)
        target = _bot(rid=2, x=100, y=0, thermal=1.0)
        cover = [CoverPolygon(
            vertices=[(40, -10), (60, -10), (60, 10), (40, 10)],
            cover_value=1.0, label="wall",
        )]
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [target],
            self._cfg(fov_deg=180.0, cover_polygons=cover, weather=CLEAR),
        )
        assert out["detections"] == []

    def test_outside_fov(self) -> None:
        observer = _bot(theta=0.0)  # +x
        target = _bot(rid=2, x=-50, y=0, thermal=1.0)  # behind
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [target], self._cfg(fov_deg=60.0),
        )
        assert out["detections"] == []

    def test_sorted_by_range(self) -> None:
        observer = _bot(theta=0.0)
        far = _bot(rid=1, x=200, y=0, thermal=1.0)
        near = _bot(rid=2, x=50, y=0, thermal=1.0)
        out = self.plugin.compute(
            observer, _EMPTY_ARENA, [far, near], self._cfg(fov_deg=180.0),
        )
        assert [d["robot_id"] for d in out["detections"]] == [2, 1]


# ─────────────────────────── IED Detector ─────────────────────────────────

class TestIedDetector:
    plugin = IedDetectorSensor()

    def test_no_environment_no_detections(self) -> None:
        out = self.plugin.compute(
            _bot(), _EMPTY_ARENA, [],
            self.plugin.default_config(), environment=None,
        )
        assert out["detections"] == []

    def test_detects_within_radius(self) -> None:
        env = EnvironmentContext(point_sources={
            "ied": [{"position": (5.0, 0.0), "label": "alpha"}],
        })
        out = self.plugin.compute(
            _bot(x=0, y=0), _EMPTY_ARENA, [],
            self.plugin.default_config(),
            environment=env,
        )
        assert len(out["detections"]) == 1
        assert out["detections"][0]["label"] == "alpha"

    def test_excludes_outside_radius(self) -> None:
        env = EnvironmentContext(point_sources={
            "ied": [{"position": (50.0, 0.0)}],
        })
        # default radius is 5m; source is 50m away
        out = self.plugin.compute(
            _bot(x=0, y=0), _EMPTY_ARENA, [],
            self.plugin.default_config(), environment=env,
        )
        assert out["detections"] == []

    def test_per_source_radius_override(self) -> None:
        env = EnvironmentContext(point_sources={
            "ied": [{"position": (20.0, 0.0), "detectability_radius_m": 30.0}],
        })
        out = self.plugin.compute(
            _bot(x=0, y=0), _EMPTY_ARENA, [],
            self.plugin.default_config(), environment=env,
        )
        assert len(out["detections"]) == 1

    def test_confidence_decreases_with_distance(self) -> None:
        env = EnvironmentContext(point_sources={
            "ied": [
                {"position": (1.0, 0.0)},
                {"position": (4.0, 0.0)},
            ],
        })
        out = self.plugin.compute(
            _bot(x=0, y=0), _EMPTY_ARENA, [],
            self.plugin.default_config(), environment=env,
        )
        # Sorted by distance — first is highest confidence
        assert out["detections"][0]["confidence"] > out["detections"][1]["confidence"]


# ─────────────────────────── DF Receiver ──────────────────────────────────

class TestDfReceiver:
    plugin = DfReceiverSensor()

    def test_no_environment(self) -> None:
        out = self.plugin.compute(
            _bot(), _EMPTY_ARENA, [], self.plugin.default_config(),
        )
        assert out["bearings"] == []

    def test_bearing_to_emitter(self) -> None:
        env = EnvironmentContext(point_sources={
            "ew_emitters": [{
                "position": (100.0, 0.0),
                "signal_strength": 1.0,
                "frequency_mhz": 144.0,
            }],
        })
        cfg = self.plugin.default_config()
        cfg.params["bearing_noise_deg"] = 0.0  # deterministic
        out = self.plugin.compute(
            _bot(x=0, y=0), _EMPTY_ARENA, [], cfg, environment=env,
        )
        assert len(out["bearings"]) == 1
        b = out["bearings"][0]
        assert b["bearing_deg"] == pytest.approx(0.0, abs=0.5)
        assert b["frequency_mhz"] == 144.0

    def test_weak_signal_filtered(self) -> None:
        env = EnvironmentContext(point_sources={
            "ew_emitters": [{
                "position": (10000.0, 0.0),
                "signal_strength": 0.01,
            }],
        })
        cfg = self.plugin.default_config()
        out = self.plugin.compute(
            _bot(), _EMPTY_ARENA, [], cfg, environment=env,
        )
        assert out["bearings"] == []

    def test_sorted_by_strength(self) -> None:
        env = EnvironmentContext(point_sources={
            "ew_emitters": [
                {"position": (500.0, 0.0), "signal_strength": 1.0,
                 "label": "weak"},
                {"position": (100.0, 0.0), "signal_strength": 1.0,
                 "label": "strong"},
            ],
        })
        cfg = self.plugin.default_config()
        cfg.params["bearing_noise_deg"] = 0.0
        out = self.plugin.compute(
            _bot(), _EMPTY_ARENA, [], cfg, environment=env,
        )
        assert out["bearings"][0]["label"] == "strong"


# ─────────────────────────── UAV Camera ───────────────────────────────────

class TestUavCamera:
    plugin = UavCameraSensor()

    def test_unit_inside_footprint_detected(self) -> None:
        uav = _bot(rid=99, x=0, y=0, theta=0.0,
                    team="blue", vehicle_role="uav")
        target = _bot(rid=1, x=50, y=20, team="red")
        out = self.plugin.compute(
            uav, _EMPTY_ARENA, [target], self.plugin.default_config(),
        )
        assert len(out["detections"]) == 1
        assert out["detections"][0]["robot_id"] == 1

    def test_unit_outside_footprint_excluded(self) -> None:
        uav = _bot(rid=99, x=0, y=0, theta=0.0)
        far = _bot(rid=1, x=500, y=500, team="red")
        out = self.plugin.compute(
            uav, _EMPTY_ARENA, [far], self.plugin.default_config(),
        )
        assert out["detections"] == []

    def test_self_excluded(self) -> None:
        uav = _bot(rid=99, x=0, y=0)
        out = self.plugin.compute(
            uav, _EMPTY_ARENA, [uav], self.plugin.default_config(),
        )
        assert out["detections"] == []

    def test_destroyed_excluded(self) -> None:
        uav = _bot(rid=99, x=0, y=0)
        dead = _bot(rid=1, x=10, y=0, status="destroyed", health=0.0)
        out = self.plugin.compute(
            uav, _EMPTY_ARENA, [dead], self.plugin.default_config(),
        )
        assert out["detections"] == []

    def test_ignore_team_filter(self) -> None:
        uav = _bot(rid=99, x=0, y=0, team="blue")
        friend = _bot(rid=1, x=10, y=0, team="blue")
        cfg = self.plugin.default_config()
        cfg.params["ignore_teams"] = ["blue"]
        out = self.plugin.compute(uav, _EMPTY_ARENA, [friend], cfg)
        assert out["detections"] == []

    def test_heading_rotates_footprint(self) -> None:
        # UAV facing +y (theta=pi/2); a unit due north should be inside
        # while a unit due east at the same range should be outside if
        # the height < width.
        uav = _bot(rid=99, x=0, y=0, theta=math.pi / 2)
        north = _bot(rid=1, x=0, y=80, team="red")
        east = _bot(rid=2, x=80, y=0, team="red")
        cfg = self.plugin.default_config()
        cfg.params["footprint_width_m"] = 200.0
        cfg.params["footprint_height_m"] = 50.0
        out = self.plugin.compute(uav, _EMPTY_ARENA, [north, east], cfg)
        ids = [d["robot_id"] for d in out["detections"]]
        # After rotation, +y is "forward" (long axis), +x is "side" (short axis)
        assert 1 in ids   # north — inside long axis
        assert 2 not in ids  # east — beyond short axis (80 > 25)

    def test_weather_zero_visibility_filters_all(self) -> None:
        uav = _bot(rid=99, x=0, y=0)
        target = _bot(rid=1, x=50, y=0, team="red")
        cfg = self.plugin.default_config()
        cfg.params["weather"] = WeatherState(visibility_m=10.0)
        out = self.plugin.compute(uav, _EMPTY_ARENA, [target], cfg)
        assert out["detections"] == []
