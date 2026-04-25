"""Pluggable robot kinematics models.

Each model implements the KinematicsModel protocol, providing a predict()
method that computes the next pose given a velocity command and time step.
The platform supports different robot types (differential drive, omnidirectional,
Ackermann) by selecting the appropriate model.
"""

from __future__ import annotations

import math
from typing import Protocol


class KinematicsModel(Protocol):
    """Protocol for robot motion models."""

    name: str

    def predict(
        self,
        x: float, y: float, theta: float,
        linear_x: float, angular_z: float,
        dt: float,
    ) -> tuple[float, float, float]:
        """Predict the next pose given velocity command and time step.

        Args:
            x: Current X position (metres).
            y: Current Y position (metres).
            theta: Current heading (radians).
            linear_x: Forward velocity (m/s).
            angular_z: Rotational velocity (rad/s).
            dt: Time step (seconds).

        Returns:
            Predicted (x', y', theta') with theta in [-pi, pi].
        """
        ...


def _normalize_angle(theta: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


class DifferentialDrive:
    """Standard two-wheel differential drive kinematics.

    The robot moves forward/backward along its heading axis and rotates
    in place. Cannot move sideways without rotating first.
    """

    name = "differential_drive"

    def predict(
        self,
        x: float, y: float, theta: float,
        linear_x: float, angular_z: float,
        dt: float,
    ) -> tuple[float, float, float]:
        new_x = x + linear_x * math.cos(theta) * dt
        new_y = y + linear_x * math.sin(theta) * dt
        new_theta = _normalize_angle(theta + angular_z * dt)
        return new_x, new_y, new_theta


class Omnidirectional:
    """Omnidirectional (holonomic) robot kinematics.

    The robot can move in any direction without rotating. linear_x is
    forward speed, angular_z is rotation. For full omnidirectional control,
    extend the command interface to include lateral velocity.
    """

    name = "omnidirectional"

    def predict(
        self,
        x: float, y: float, theta: float,
        linear_x: float, angular_z: float,
        dt: float,
    ) -> tuple[float, float, float]:
        # Same as differential drive for forward + rotation
        # True omnidirectional would need a lateral velocity component
        new_x = x + linear_x * math.cos(theta) * dt
        new_y = y + linear_x * math.sin(theta) * dt
        new_theta = _normalize_angle(theta + angular_z * dt)
        return new_x, new_y, new_theta


# Registry of available models
KINEMATICS_MODELS: dict[str, type] = {
    "differential_drive": DifferentialDrive,
    "omnidirectional": Omnidirectional,
}


def get_kinematics_model(name: str) -> KinematicsModel:
    """Get a kinematics model instance by name.

    Args:
        name: Model name (e.g. "differential_drive").

    Returns:
        KinematicsModel instance.

    Raises:
        KeyError: If the model name is not registered.
    """
    cls = KINEMATICS_MODELS.get(name)
    if cls is None:
        raise KeyError(f"Unknown kinematics model: {name}. "
                       f"Available: {list(KINEMATICS_MODELS.keys())}")
    return cls()
