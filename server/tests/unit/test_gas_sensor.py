"""Tests for plato_pod.sensor_plugins.gas — gas sensor with MOX dynamics."""

from __future__ import annotations

import math

import pytest

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    EnvironmentContext,
    SensorConfig,
)
from plato_pod.sensor_plugins.gas import GasSensor
from plato_pod.spatial_field import GaussianPlumeField, UniformField


def _robot(x: float = 0.5, y: float = 0.3) -> Robot:
    return Robot(robot_id=1, deployment="virtual", x=x, y=y, theta=0.0, radius=0.028)


def _arena() -> ArenaState:
    return ArenaState(
        boundary=((0, 0), (1, 0), (1, 1), (0, 1)),
        obstacles=[],
    )


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


class TestGasSensorNoEnvironment:
    def test_no_environment_returns_zero(self) -> None:
        gas = GasSensor()
        data = gas.compute(_robot(), _arena(), [], _config())
        assert data["concentration"] == 0.0
        assert data["raw_resistance"] == 1.0  # r_clean

    def test_no_gas_field_returns_zero(self) -> None:
        gas = GasSensor()
        env = EnvironmentContext(fields={"temperature": UniformField(value=25.0)})
        data = gas.compute(_robot(), _arena(), [], _config(), environment=env)
        assert data["concentration"] == 0.0

    def test_default_config(self) -> None:
        gas = GasSensor()
        cfg = gas.default_config()
        assert cfg.rate_hz == 10.0
        assert cfg.params["response_time"] == 2.0


class TestGasSensorWithField:
    def _env_with_uniform_gas(self, concentration: float) -> EnvironmentContext:
        return EnvironmentContext(
            fields={"gas": UniformField(value=concentration)},
            time=0.0,
        )

    def test_uniform_gas_concentration(self) -> None:
        gas = GasSensor()
        env = self._env_with_uniform_gas(500.0)
        data = gas.compute(_robot(), _arena(), [], _config(), environment=env)
        assert data["concentration"] == pytest.approx(500.0)

    def test_high_concentration_low_resistance(self) -> None:
        gas = GasSensor()
        env_low = self._env_with_uniform_gas(10.0)
        env_high = self._env_with_uniform_gas(5000.0)
        r_low = gas.compute(_robot(), _arena(), [], _config(), environment=env_low)
        r_high = gas.compute(_robot(), _arena(), [], _config(), environment=env_high)
        # Higher concentration → lower resistance
        assert r_high["equilibrium_resistance"] < r_low["equilibrium_resistance"]

    def test_zero_concentration_clean_resistance(self) -> None:
        gas = GasSensor()
        env = self._env_with_uniform_gas(0.0)
        data = gas.compute(_robot(), _arena(), [], _config(), environment=env)
        assert data["equilibrium_resistance"] == pytest.approx(1.0)

    def test_very_high_concentration_approaches_r_gas(self) -> None:
        gas = GasSensor()
        env = self._env_with_uniform_gas(1e6)
        data = gas.compute(_robot(), _arena(), [], _config(), environment=env)
        assert data["equilibrium_resistance"] == pytest.approx(0.1, abs=0.01)


class TestGasSensorStateEvolution:
    def test_state_evolves_toward_equilibrium(self) -> None:
        gas = GasSensor()
        env = EnvironmentContext(
            fields={"gas": UniformField(value=500.0)},
            time=0.0,
        )
        state: dict = {}
        config = _config(response_time=1.0, dt=0.1)

        # Compute several timesteps
        resistances = []
        for _ in range(50):
            data = gas.compute(
                _robot(), _arena(), [], config,
                environment=env, state=state,
            )
            resistances.append(data["raw_resistance"])

        # Should start near r_clean (1.0) and decrease toward equilibrium
        assert resistances[0] > resistances[-1]
        # Should converge (last values close together)
        assert abs(resistances[-1] - resistances[-2]) < 0.001

    def test_state_tracks_resistance(self) -> None:
        gas = GasSensor()
        env = EnvironmentContext(
            fields={"gas": UniformField(value=100.0)},
            time=0.0,
        )
        state: dict = {}
        config = _config(response_time=1.0, dt=0.1)

        gas.compute(_robot(), _arena(), [], config, environment=env, state=state)
        assert "resistance" in state
        assert "concentration" in state

    def test_no_state_returns_equilibrium(self) -> None:
        gas = GasSensor()
        env = EnvironmentContext(
            fields={"gas": UniformField(value=500.0)},
            time=0.0,
        )
        config = _config()
        # No state dict → immediate equilibrium
        data = gas.compute(_robot(), _arena(), [], config, environment=env)
        assert data["raw_resistance"] == data["equilibrium_resistance"]

    def test_resistance_recovers_when_gas_clears(self) -> None:
        gas = GasSensor()
        config = _config(response_time=0.5, dt=0.1)
        state: dict = {}

        # Phase 1: expose to gas
        env_gas = EnvironmentContext(
            fields={"gas": UniformField(value=1000.0)},
            time=0.0,
        )
        for _ in range(100):
            gas.compute(_robot(), _arena(), [], config, environment=env_gas, state=state)
        r_exposed = state["resistance"]

        # Phase 2: move to clean air
        env_clean = EnvironmentContext(
            fields={"gas": UniformField(value=0.0)},
            time=0.0,
        )
        for _ in range(100):
            gas.compute(_robot(), _arena(), [], config, environment=env_clean, state=state)
        r_recovered = state["resistance"]

        # Resistance should recover toward r_clean (1.0)
        assert r_recovered > r_exposed
        assert r_recovered == pytest.approx(1.0, abs=0.01)

    def test_zero_tau_instant_response(self) -> None:
        gas = GasSensor()
        config = _config(response_time=0.0, dt=0.1)
        state: dict = {}
        env = EnvironmentContext(
            fields={"gas": UniformField(value=500.0)},
            time=0.0,
        )
        data = gas.compute(_robot(), _arena(), [], config, environment=env, state=state)
        # With tau=0, should immediately reach equilibrium
        assert data["raw_resistance"] == pytest.approx(data["equilibrium_resistance"])


class TestGasSensorWithPlume:
    def test_downwind_higher_than_upwind(self) -> None:
        gas = GasSensor()
        plume = GaussianPlumeField(
            source_x=0.0, source_y=0.0,
            release_rate=100.0, wind_speed=2.0,
            wind_direction=0.0, diffusion_coeff=0.05,
        )
        env = EnvironmentContext(fields={"gas": plume}, time=0.0)
        config = _config()

        # Robot downwind of source
        d_down = gas.compute(
            _robot(x=0.5, y=0.0), _arena(), [], config, environment=env
        )
        # Robot upwind of source
        d_up = gas.compute(
            _robot(x=-0.5, y=0.0), _arena(), [], config, environment=env
        )
        assert d_down["concentration"] > d_up["concentration"]

    def test_custom_field_name(self) -> None:
        gas = GasSensor()
        env = EnvironmentContext(
            fields={"toxic_gas": UniformField(value=200.0)},
            time=0.0,
        )
        config = _config(field_name="toxic_gas")
        data = gas.compute(_robot(), _arena(), [], config, environment=env)
        assert data["concentration"] == pytest.approx(200.0)


class TestGasSensorEngineIntegration:
    def test_engine_with_gas_sensor(self) -> None:
        from plato_pod.sensor_engine import SensorEngine

        engine = SensorEngine()
        assert "gas" in engine.list_available_sensors()

        robot = _robot()
        engine.configure_sensor(robot.robot_id, "gas", _config())

        env = EnvironmentContext(
            fields={"gas": UniformField(value=300.0)},
            time=0.0,
        )
        engine.set_environment(env)

        readings = engine.compute_sensors(robot, _arena(), [])
        assert len(readings) == 1
        assert readings[0].sensor_name == "gas"
        assert readings[0].data["concentration"] == pytest.approx(300.0)

    def test_engine_state_persists_across_calls(self) -> None:
        from plato_pod.sensor_engine import SensorEngine

        engine = SensorEngine()
        robot = _robot()
        config = _config(response_time=1.0, dt=0.1)
        engine.configure_sensor(robot.robot_id, "gas", config)
        engine.set_environment(EnvironmentContext(
            fields={"gas": UniformField(value=500.0)},
            time=0.0,
        ))

        r1 = engine.compute_sensors(robot, _arena(), [])
        r2 = engine.compute_sensors(robot, _arena(), [])

        # State should evolve between calls
        assert r1[0].data["raw_resistance"] != r2[0].data["raw_resistance"]

    def test_engine_reset_state(self) -> None:
        from plato_pod.sensor_engine import SensorEngine

        engine = SensorEngine()
        robot = _robot()
        config = _config(response_time=1.0, dt=0.1)
        engine.configure_sensor(robot.robot_id, "gas", config)
        engine.set_environment(EnvironmentContext(
            fields={"gas": UniformField(value=500.0)},
            time=0.0,
        ))

        # Compute to build up state
        for _ in range(10):
            engine.compute_sensors(robot, _arena(), [])

        # Reset and recompute — should start fresh
        engine.reset_state(robot.robot_id)
        r_after_reset = engine.compute_sensors(robot, _arena(), [])
        r_fresh = GasSensor().compute(
            robot, _arena(), [], config,
            environment=engine.get_environment(), state={},
        )
        assert r_after_reset[0].data["raw_resistance"] == pytest.approx(
            r_fresh["raw_resistance"]
        )
