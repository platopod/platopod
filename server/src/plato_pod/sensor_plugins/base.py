"""Base sensor plugin protocol and configuration.

All sensor plugins implement the SensorPlugin protocol. No ROS2 dependency.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from plato_pod.spatial_field import SpatialField


@dataclass
class SensorConfig:
    """Per-robot configuration for a single sensor."""
    enabled: bool = True
    rate_hz: float = 10.0
    noise_stddev: float = 0.0
    dropout_rate: float = 0.0
    # Sensor-specific parameters stored as a dict
    params: dict = field(default_factory=dict)


from plato_pod.robot import Robot


@dataclass
class ArenaState:
    """Arena state snapshot for sensor computation."""
    boundary: tuple[tuple[float, float], ...]
    obstacles: list[tuple[tuple[float, float], ...]]


@dataclass
class EnvironmentContext:
    """Environmental state for sensor computation.

    Carries virtual data layers (spatial fields) and ambient conditions.
    Passed to sensor plugins that need environmental awareness.
    """
    fields: dict[str, SpatialField] = field(default_factory=dict)
    wind_speed: float = 0.0          # m/s
    wind_direction: float = 0.0      # radians (direction wind blows TO)
    temperature: float = 20.0        # Celsius
    time: float = 0.0                # exercise time in seconds


class SensorPlugin(Protocol):
    """Protocol for sensor plugins.

    Each plugin computes sensor data for a single robot given the arena state.
    Plugins that don't need environment or state can ignore those parameters.
    """

    name: str

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        environment: EnvironmentContext | None = None,
        state: dict | None = None,
    ) -> dict:
        """Compute sensor reading for a robot.

        Args:
            robot: The robot's current state.
            arena: Arena boundary and obstacles.
            other_robots: All other active robots.
            config: Sensor configuration including noise parameters.
            environment: Environmental context with spatial fields and
                ambient conditions. None if no environment is configured.
            state: Mutable per-robot state dict for this sensor. Plugins
                can read/write this dict to maintain state across calls
                (e.g., ODE integrator state). None if stateless.

        Returns:
            Sensor data dict (format depends on sensor type).
        """
        ...

    def default_config(self) -> SensorConfig:
        """Return the default configuration for this sensor."""
        ...


def apply_noise(value: float, stddev: float) -> float:
    """Add Gaussian noise to a value."""
    if stddev <= 0:
        return value
    return value + random.gauss(0, stddev)


def apply_dropout(values: list[float], dropout_rate: float,
                  invalid_value: float = float('inf')) -> list[float]:
    """Randomly replace values with invalid readings."""
    if dropout_rate <= 0:
        return values
    return [
        invalid_value if random.random() < dropout_rate else v
        for v in values
    ]
