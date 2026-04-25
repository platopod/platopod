"""Tests for plato_pod.sensor_engine — sensor computation engine."""

from __future__ import annotations

import pytest

from plato_pod.robot import Robot
from plato_pod.sensor_engine import SensorEngine, SensorReading
from plato_pod.sensor_plugins.base import ArenaState, EnvironmentContext, SensorConfig
from plato_pod.spatial_field import UniformField


def _robot(robot_id: int = 1) -> Robot:
    return Robot(robot_id=robot_id, deployment="virtual", x=0.5, y=0.3, theta=1.0, radius=0.028)


def _arena() -> ArenaState:
    return ArenaState(boundary=((0, 0), (1, 0), (1, 1), (0, 1)), obstacles=[])


class TestSensorEngine:
    def test_no_config_returns_empty(self) -> None:
        engine = SensorEngine()
        readings = engine.compute_sensors(_robot(), _arena(), [])
        assert readings == []

    def test_configure_gps_and_compute(self) -> None:
        engine = SensorEngine()
        config = SensorConfig(noise_stddev=0.0, params={"heading_noise_stddev": 0.0})
        ok, msg = engine.configure_sensor(1, "gps", config)
        assert ok is True

        readings = engine.compute_sensors(_robot(), _arena(), [])
        assert len(readings) == 1
        assert readings[0].sensor_name == "gps"
        assert readings[0].data["x"] == pytest.approx(0.5)
        assert readings[0].data["y"] == pytest.approx(0.3)

    def test_configure_unknown_sensor_fails(self) -> None:
        engine = SensorEngine()
        ok, msg = engine.configure_sensor(1, "radar", SensorConfig())
        assert ok is False
        assert "Unknown" in msg

    def test_apply_preset(self) -> None:
        engine = SensorEngine()
        ok, msg = engine.apply_preset(1, "minimal")
        assert ok is True
        sensors = engine.get_robot_sensors(1)
        assert "gps" in sensors

    def test_apply_preset_full_suite(self) -> None:
        engine = SensorEngine()
        engine.apply_preset(1, "full_suite")
        sensors = engine.get_robot_sensors(1)
        assert "gps" in sensors
        assert "lidar_2d" in sensors
        assert "sonar" in sensors
        assert "fof" in sensors

    def test_apply_unknown_preset_fails(self) -> None:
        engine = SensorEngine()
        ok, msg = engine.apply_preset(1, "nonexistent")
        assert ok is False

    def test_remove_robot(self) -> None:
        engine = SensorEngine()
        engine.apply_preset(1, "minimal")
        engine.remove_robot(1)
        assert engine.get_robot_sensors(1) == []

    def test_disabled_sensor_not_computed(self) -> None:
        engine = SensorEngine()
        config = SensorConfig(enabled=False, noise_stddev=0.0)
        engine.configure_sensor(1, "gps", config)
        readings = engine.compute_sensors(_robot(), _arena(), [])
        assert len(readings) == 0

    def test_multiple_sensors(self) -> None:
        engine = SensorEngine()
        engine.configure_sensor(1, "gps", SensorConfig(noise_stddev=0.0,
                                params={"heading_noise_stddev": 0.0}))
        engine.configure_sensor(1, "lidar_2d", SensorConfig(
                                params={"range_max": 2.0, "angular_resolution_deg": 1.0}))
        readings = engine.compute_sensors(_robot(), _arena(), [])
        assert len(readings) == 2
        names = {r.sensor_name for r in readings}
        assert names == {"gps", "lidar_2d"}

    def test_list_available_sensors(self) -> None:
        engine = SensorEngine()
        available = engine.list_available_sensors()
        assert "gps" in available
        assert "lidar_2d" in available
        assert "sonar" in available
        assert "fof" in available
        assert "gas" in available

    def test_reading_has_timestamp(self) -> None:
        engine = SensorEngine()
        engine.configure_sensor(1, "gps", SensorConfig(noise_stddev=0.0,
                                params={"heading_noise_stddev": 0.0}))
        readings = engine.compute_sensors(_robot(), _arena(), [])
        assert readings[0].timestamp > 0


class TestSensorEngineEnvironment:
    def test_set_environment(self) -> None:
        engine = SensorEngine()
        env = EnvironmentContext(wind_speed=5.0, temperature=30.0)
        engine.set_environment(env)
        assert engine.get_environment() is env

    def test_no_environment_returns_none(self) -> None:
        engine = SensorEngine()
        assert engine.get_environment() is None

    def test_update_field_creates_environment(self) -> None:
        engine = SensorEngine()
        field = UniformField(value=42.0)
        engine.update_field("test", field)
        env = engine.get_environment()
        assert env is not None
        assert "test" in env.fields
        assert env.fields["test"].evaluate(0, 0, 0) == 42.0

    def test_update_field_replaces(self) -> None:
        engine = SensorEngine()
        engine.update_field("gas", UniformField(value=10.0))
        engine.update_field("gas", UniformField(value=20.0))
        assert engine.get_environment().fields["gas"].evaluate(0, 0, 0) == 20.0

    def test_remove_field(self) -> None:
        engine = SensorEngine()
        engine.update_field("gas", UniformField(value=10.0))
        assert engine.remove_field("gas") is True
        assert "gas" not in engine.get_environment().fields

    def test_remove_nonexistent_field(self) -> None:
        engine = SensorEngine()
        assert engine.remove_field("nope") is False

    def test_reset_state_clears_robot(self) -> None:
        engine = SensorEngine()
        engine.configure_sensor(1, "gas", SensorConfig(
            params={"response_time": 1.0, "dt": 0.1}))
        engine.set_environment(EnvironmentContext(
            fields={"gas": UniformField(value=100.0)}))
        # Build up state
        engine.compute_sensors(_robot(1), _arena(), [])
        engine.compute_sensors(_robot(1), _arena(), [])
        engine.reset_state(1)
        # State should be cleared — next compute starts fresh
        r1 = engine.compute_sensors(_robot(1), _arena(), [])
        assert len(r1) == 1

    def test_reset_state_all(self) -> None:
        engine = SensorEngine()
        engine.configure_sensor(1, "gas", SensorConfig())
        engine.configure_sensor(2, "gas", SensorConfig())
        engine.set_environment(EnvironmentContext(
            fields={"gas": UniformField(value=50.0)}))
        engine.compute_sensors(_robot(1), _arena(), [])
        engine.compute_sensors(_robot(2), _arena(), [])
        engine.reset_state()  # reset all
        # Both should start fresh on next compute

    def test_remove_robot_clears_state(self) -> None:
        engine = SensorEngine()
        engine.configure_sensor(1, "gas", SensorConfig())
        engine.set_environment(EnvironmentContext(
            fields={"gas": UniformField(value=50.0)}))
        engine.compute_sensors(_robot(1), _arena(), [])
        engine.remove_robot(1)
        # Config and state both cleared
        assert engine.get_robot_sensors(1) == []
