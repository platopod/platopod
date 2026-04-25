"""Sensor engine — manages sensor computation for all robots.

Thread-safe. Tracks per-robot sensor configurations, computes sensor data
using registered plugins, and returns results. No ROS2 dependency.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from plato_pod.sensor_plugins import ALL_PLUGINS
from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import (
    ArenaState,
    EnvironmentContext,
    SensorConfig,
)
from plato_pod.sensor_presets import get_preset

if TYPE_CHECKING:
    from plato_pod.spatial_field import SpatialField

logger = logging.getLogger(__name__)


@dataclass
class RobotSensorConfig:
    """All sensor configurations for a single robot."""
    robot_id: int
    sensors: dict[str, SensorConfig] = field(default_factory=dict)
    # sensor_name -> config


@dataclass(frozen=True, slots=True)
class SensorReading:
    """A single sensor reading result."""
    robot_id: int
    sensor_name: str
    timestamp: float
    data: dict


class SensorEngine:
    """Manages sensor computation for all robots."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._robot_configs: dict[int, RobotSensorConfig] = {}
        self._plugins = {name: cls() for name, cls in ALL_PLUGINS.items()}
        self._robot_state: dict[tuple[int, str], dict] = {}
        self._environment: EnvironmentContext | None = None

    def configure_sensor(
        self, robot_id: int, sensor_name: str, config: SensorConfig
    ) -> tuple[bool, str]:
        """Configure a sensor for a robot.

        Returns:
            (success, message).
        """
        if sensor_name not in self._plugins:
            return False, f"Unknown sensor: {sensor_name}"

        with self._lock:
            if robot_id not in self._robot_configs:
                self._robot_configs[robot_id] = RobotSensorConfig(robot_id=robot_id)
            self._robot_configs[robot_id].sensors[sensor_name] = config

        return True, "ok"

    def apply_preset(
        self, robot_id: int, preset_name: str
    ) -> tuple[bool, str]:
        """Apply a named sensor preset to a robot.

        Returns:
            (success, message).
        """
        preset = get_preset(preset_name)
        if preset is None:
            return False, f"Unknown preset: {preset_name}"

        with self._lock:
            self._robot_configs[robot_id] = RobotSensorConfig(
                robot_id=robot_id,
                sensors=dict(preset),
            )

        return True, "ok"

    def set_environment(self, environment: EnvironmentContext) -> None:
        """Set the environment context for sensor computation."""
        with self._lock:
            self._environment = environment

    def update_field(self, name: str, field: SpatialField) -> None:
        """Add or replace a spatial field in the environment.

        Creates an EnvironmentContext if none exists.
        """
        with self._lock:
            if self._environment is None:
                self._environment = EnvironmentContext()
            self._environment.fields[name] = field

    def remove_field(self, name: str) -> bool:
        """Remove a spatial field by name. Returns True if it existed."""
        with self._lock:
            if self._environment is None:
                return False
            return self._environment.fields.pop(name, None) is not None

    def get_environment(self) -> EnvironmentContext | None:
        """Return the current environment context (or None)."""
        with self._lock:
            return self._environment

    def reset_state(self, robot_id: int | None = None) -> None:
        """Reset per-robot sensor state.

        Args:
            robot_id: If given, reset only this robot's state.
                If None, reset all robots.
        """
        with self._lock:
            if robot_id is None:
                self._robot_state.clear()
            else:
                keys = [k for k in self._robot_state if k[0] == robot_id]
                for k in keys:
                    del self._robot_state[k]

    def remove_robot(self, robot_id: int) -> None:
        """Remove all sensor config and state for a robot."""
        with self._lock:
            self._robot_configs.pop(robot_id, None)
            keys = [k for k in self._robot_state if k[0] == robot_id]
            for k in keys:
                del self._robot_state[k]

    def get_robot_sensors(self, robot_id: int) -> list[str]:
        """Get list of configured sensor names for a robot."""
        with self._lock:
            cfg = self._robot_configs.get(robot_id)
            if cfg is None:
                return []
            return [name for name, sc in cfg.sensors.items() if sc.enabled]

    def compute_sensors(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
    ) -> list[SensorReading]:
        """Compute all sensor readings for a robot.

        Args:
            robot: The robot's current state.
            arena: Arena boundary and obstacles.
            other_robots: All other active robots.

        Returns:
            List of SensorReading objects.
        """
        with self._lock:
            cfg = self._robot_configs.get(robot.robot_id)
            if cfg is None:
                return []
            # Copy configs under lock
            sensors_to_compute = dict(cfg.sensors)
            environment = self._environment

        now = time.time()
        readings = []

        for sensor_name, sensor_config in sensors_to_compute.items():
            if not sensor_config.enabled:
                continue

            plugin = self._plugins.get(sensor_name)
            if plugin is None:
                continue

            # Get or create per-robot state for this sensor
            state_key = (robot.robot_id, sensor_name)
            with self._lock:
                if state_key not in self._robot_state:
                    self._robot_state[state_key] = {}
                state = self._robot_state[state_key]

            data = plugin.compute(
                robot, arena, other_robots, sensor_config,
                environment=environment, state=state,
            )
            readings.append(SensorReading(
                robot_id=robot.robot_id,
                sensor_name=sensor_name,
                timestamp=now,
                data=data,
            ))

        return readings

    def list_available_sensors(self) -> list[str]:
        """Return all registered sensor plugin names."""
        return list(self._plugins.keys())
