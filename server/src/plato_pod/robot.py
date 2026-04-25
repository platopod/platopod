"""Unified robot data model for the Plato Pod platform.

Single Robot class used across all modules: registry, command pipeline,
sensor engine, virtual simulation, and API gateway. Localization-agnostic —
the Robot doesn't know how its pose was determined.
"""

from __future__ import annotations

from dataclasses import dataclass

from plato_pod.pose import PoseSource

DEFAULT_RADIUS = 0.028  # metres


@dataclass
class Robot:
    """A robot in the Plato Pod system — physical or virtual.

    The localization_id and localization_source fields identify the robot
    to its localization provider. For AprilTag robots, localization_id is
    the tag number (e.g. "3"). For GPS robots, it might be "rover-1".
    The rest of the platform ignores these fields — it only uses robot_id.
    """
    robot_id: int
    deployment: str             # "physical" or "virtual"
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    radius: float = DEFAULT_RADIUS
    status: str = "active"      # "active", "inactive", "error"
    localization_id: str = ""   # provider-specific ID (tag "3", "rover-1")
    localization_source: PoseSource = PoseSource.UNKNOWN
    kinematics_model: str = "differential_drive"
    team: str | None = None
    linear_x: float = 0.0      # current commanded linear velocity
    angular_z: float = 0.0     # current commanded angular velocity
    sensor_preset: str = "minimal"

    def to_dict(self) -> dict:
        """Convert to JSON-serializable dict."""
        return {
            "robot_id": self.robot_id,
            "type": self.deployment,
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "radius": self.radius,
            "status": self.status,
            "localization_id": self.localization_id,
            "localization_source": self.localization_source.value,
            "kinematics_model": self.kinematics_model,
            "team": self.team,
            "linear_x": self.linear_x,
            "angular_z": self.angular_z,
            "sensor_preset": self.sensor_preset,
        }
