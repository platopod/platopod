"""Tests for plato_pod.sensor_plugins.gaden_bridge — GADEN gas passthrough."""

from __future__ import annotations

import pytest

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    EnvironmentContext,
    SensorConfig,
)
from plato_pod.sensor_plugins.gaden_bridge import GadenGasBridge
from plato_pod.spatial_field import UniformField


def _robot(robot_id: int = 1, x: float = 0.5, y: float = 0.3) -> Robot:
    return Robot(robot_id=robot_id, deployment="virtual", x=x, y=y, radius=0.028)


def _arena() -> ArenaState:
    return ArenaState(boundary=((0, 0), (1, 0), (1, 1), (0, 1)), obstacles=[])


def _config(**overrides) -> SensorConfig:
    params = {
        "response_time": 2.0,
        "dt": 0.1,
        "r_clean": 1.0,
        "r_gas": 0.1,
        "saturation_concentration": 1000.0,
        "field_name": "gas",
    }
    params.update(overrides)
    return SensorConfig(noise_stddev=0.0, params=params)


class TestGadenBridgeNoData:
    def test_no_gaden_no_environment_returns_zero(self) -> None:
        bridge = GadenGasBridge()
        data = bridge.compute(_robot(), _arena(), [], _config())
        assert data["concentration"] == 0.0
        assert data["source"] == "python"

    def test_no_gaden_with_environment_uses_python(self) -> None:
        bridge = GadenGasBridge()
        env = EnvironmentContext(
            fields={"gas": UniformField(value=200.0)},
            time=0.0,
        )
        data = bridge.compute(_robot(), _arena(), [], _config(), environment=env)
        assert data["concentration"] == pytest.approx(200.0)
        assert data["source"] == "python"

    def test_has_data_false(self) -> None:
        bridge = GadenGasBridge()
        assert bridge.has_data(1) is False


class TestGadenBridgeWithData:
    def test_gaden_data_used(self) -> None:
        bridge = GadenGasBridge()
        bridge.update_concentration(1, 500.0)
        assert bridge.has_data(1) is True

        data = bridge.compute(_robot(1), _arena(), [], _config())
        assert data["concentration"] == pytest.approx(500.0)
        assert data["source"] == "gaden"

    def test_gaden_overrides_environment(self) -> None:
        bridge = GadenGasBridge()
        bridge.update_concentration(1, 300.0)
        env = EnvironmentContext(
            fields={"gas": UniformField(value=999.0)},
            time=0.0,
        )
        data = bridge.compute(_robot(1), _arena(), [], _config(), environment=env)
        # GADEN data takes precedence
        assert data["concentration"] == pytest.approx(300.0)
        assert data["source"] == "gaden"

    def test_per_robot_data(self) -> None:
        bridge = GadenGasBridge()
        bridge.update_concentration(1, 100.0)
        bridge.update_concentration(2, 500.0)

        d1 = bridge.compute(_robot(1), _arena(), [], _config())
        d2 = bridge.compute(_robot(2), _arena(), [], _config())
        assert d1["concentration"] == pytest.approx(100.0)
        assert d2["concentration"] == pytest.approx(500.0)

    def test_unknown_robot_falls_back(self) -> None:
        bridge = GadenGasBridge()
        bridge.update_concentration(1, 100.0)
        # Robot 99 has no GADEN data
        data = bridge.compute(_robot(99), _arena(), [], _config())
        assert data["source"] == "python"


class TestGadenBridgeMoxDynamics:
    def test_state_evolves(self) -> None:
        bridge = GadenGasBridge()
        bridge.update_concentration(1, 500.0)
        config = _config(response_time=1.0, dt=0.1)
        state: dict = {}

        resistances = []
        for _ in range(50):
            data = bridge.compute(
                _robot(1), _arena(), [], config, state=state,
            )
            resistances.append(data["raw_resistance"])

        # Should converge from r_clean toward equilibrium
        assert resistances[0] > resistances[-1]
        assert abs(resistances[-1] - resistances[-2]) < 0.001

    def test_equilibrium_resistance_computed(self) -> None:
        bridge = GadenGasBridge()
        bridge.update_concentration(1, 0.0)
        data = bridge.compute(_robot(1), _arena(), [], _config())
        assert data["equilibrium_resistance"] == pytest.approx(1.0)  # r_clean

    def test_default_config(self) -> None:
        bridge = GadenGasBridge()
        cfg = bridge.default_config()
        assert cfg.rate_hz == 10.0
        assert cfg.params["response_time"] == 2.0
