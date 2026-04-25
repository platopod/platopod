"""Gas concentration sensor plugin — virtual MOX sensor digital twin.

Evaluates a gas spatial field at the robot's position and models the
first-order response dynamics of a metal-oxide (MOX) gas sensor:

    tau * dR/dt = R_eq(C) - R(t)

where R is the sensor resistance, C is the gas concentration, R_eq is the
equilibrium resistance for concentration C, and tau is the response time.

This plugin requires an EnvironmentContext with a "gas" spatial field and
uses per-robot state to track the sensor's resistance over time.
"""

from __future__ import annotations

import math

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    EnvironmentContext,
    SensorConfig,
    apply_noise,
)


class GasSensor:
    """Virtual gas concentration sensor with MOX dynamics."""

    name = "gas"

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        environment: EnvironmentContext | None = None,
        state: dict | None = None,
        **kwargs,
    ) -> dict:
        """Compute gas sensor reading at the robot's position.

        If no gas field is present in the environment, returns zero
        concentration. If state is provided, applies first-order MOX
        sensor dynamics to model realistic sensor response lag.

        Config params:
            response_time (float): MOX time constant tau in seconds (default 2.0)
            dt (float): Integration timestep in seconds (default 0.1)
            r_clean (float): Resistance in clean air (default 1.0)
            r_gas (float): Resistance at saturation concentration (default 0.1)
            saturation_concentration (float): Concentration at which
                resistance reaches r_gas (default 1000.0)
            field_name (str): Name of the gas field in environment (default "gas")
        """
        field_name = config.params.get("field_name", "gas")
        r_clean = config.params.get("r_clean", 1.0)
        r_gas = config.params.get("r_gas", 0.1)
        saturation = config.params.get("saturation_concentration", 1000.0)

        # Get concentration from spatial field
        concentration = 0.0
        if environment is not None and field_name in environment.fields:
            concentration = environment.fields[field_name].evaluate(
                robot.x, robot.y, environment.time,
            )

        # Compute equilibrium resistance for this concentration
        r_eq = self._resistance_from_concentration(
            concentration, r_clean, r_gas, saturation,
        )

        # Apply MOX sensor dynamics if state is available
        if state is not None:
            tau = config.params.get("response_time", 2.0)
            dt = config.params.get("dt", 0.1)
            r_prev = state.get("resistance", r_clean)

            if tau > 0:
                r_new = r_prev + (dt / tau) * (r_eq - r_prev)
            else:
                r_new = r_eq

            state["resistance"] = r_new
            state["concentration"] = concentration
        else:
            r_new = r_eq

        return {
            "concentration": apply_noise(concentration, config.noise_stddev),
            "raw_resistance": r_new,
            "equilibrium_resistance": r_eq,
        }

    @staticmethod
    def _resistance_from_concentration(
        concentration: float,
        r_clean: float,
        r_gas: float,
        saturation: float,
    ) -> float:
        """Map gas concentration to equilibrium sensor resistance.

        Uses an exponential decay model: R_eq = R_gas + (R_clean - R_gas) * exp(-C/S)
        where S is the saturation concentration.
        """
        if saturation <= 0:
            return r_clean
        ratio = concentration / saturation
        return r_gas + (r_clean - r_gas) * math.exp(-ratio)

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=10.0,
            noise_stddev=0.5,
            params={
                "response_time": 2.0,
                "dt": 0.1,
                "r_clean": 1.0,
                "r_gas": 0.1,
                "saturation_concentration": 1000.0,
                "field_name": "gas",
            },
        )
