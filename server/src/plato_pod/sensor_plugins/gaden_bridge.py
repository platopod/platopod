"""GADEN gas dispersion bridge — passes Gazebo-simulated gas data through.

When GADEN (Gas Distribution in Environments) is running alongside Gazebo,
this plugin receives gas concentration values from GADEN's simulated sensor
and formats them for the Plato Pod sensor engine.

Optionally applies the MOX sensor ODE dynamics on top of the raw GADEN
concentration for realistic sensor response lag (reuses the existing
GasSensor's resistance model).

Falls back to the Python GasSensor when no GADEN data is available.

Reference: https://github.com/MAPIRlab/gaden

No ROS2 dependency — this is a pure data container + formatter.
"""

from __future__ import annotations

import math
import threading

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    SensorConfig,
    apply_noise,
)


class GadenGasBridge:
    """Bridges GADEN gas concentration into the sensor plugin format.

    The sensor engine node subscribes to GADEN's concentration topic
    and calls update_concentration() on each message. compute() then
    returns the latest concentration with optional MOX dynamics.

    If no GADEN data is available, falls back to the Python GasSensor
    behavior (which uses GaussianPlumeField from the environment context).
    """

    name = "gas"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._concentrations: dict[int, float] = {}  # robot_id -> latest ppm
        self._mox_state: dict[int, float] = {}        # robot_id -> resistance

    def update_concentration(self, robot_id: int, concentration: float) -> None:
        """Update with gas concentration from GADEN for a specific robot.

        Called by the sensor engine node's ROS2 subscription callback.

        Args:
            robot_id: Which robot this reading is for.
            concentration: Gas concentration in ppm at the robot's position.
        """
        with self._lock:
            self._concentrations[robot_id] = concentration

    def has_data(self, robot_id: int) -> bool:
        """Check if GADEN data has been received for a robot."""
        with self._lock:
            return robot_id in self._concentrations

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        environment=None,
        state: dict | None = None,
        **kwargs,
    ) -> dict:
        """Return gas sensor reading from GADEN data or fallback.

        If GADEN concentration is available for this robot, uses it.
        Otherwise, falls back to the Python GasSensor (via environment
        context and GaussianPlumeField).

        Optionally applies MOX sensor ODE dynamics if state dict is
        provided (same model as GasSensor).

        Config params (same as GasSensor):
            response_time (float): MOX time constant (default 2.0)
            dt (float): Integration timestep (default 0.1)
            r_clean (float): Clean-air resistance (default 1.0)
            r_gas (float): Saturation resistance (default 0.1)
            saturation_concentration (float): Saturation ppm (default 1000.0)
        """
        r_clean = config.params.get("r_clean", 1.0)
        r_gas = config.params.get("r_gas", 0.1)
        saturation = config.params.get("saturation_concentration", 1000.0)

        # Try GADEN data first
        with self._lock:
            gaden_conc = self._concentrations.get(robot.robot_id)

        if gaden_conc is not None:
            concentration = gaden_conc
        else:
            # Fallback to Python GaussianPlumeField via environment context
            concentration = 0.0
            field_name = config.params.get("field_name", "gas")
            if environment is not None and field_name in environment.fields:
                concentration = environment.fields[field_name].evaluate(
                    robot.x, robot.y, environment.time,
                )

        # Compute equilibrium resistance
        r_eq = _resistance_from_concentration(
            concentration, r_clean, r_gas, saturation,
        )

        # Apply MOX dynamics if state is available
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
            "source": "gaden" if gaden_conc is not None else "python",
        }

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


def _resistance_from_concentration(
    concentration: float,
    r_clean: float,
    r_gas: float,
    saturation: float,
) -> float:
    """Map gas concentration to equilibrium sensor resistance."""
    if saturation <= 0:
        return r_clean
    ratio = concentration / saturation
    return r_gas + (r_clean - r_gas) * math.exp(-ratio)
