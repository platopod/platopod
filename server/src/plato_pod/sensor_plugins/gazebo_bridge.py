"""Gazebo sensor bridge plugins — pass through Gazebo sensor data.

When Gazebo is running, these plugins receive real sensor data from
Gazebo's physics engine (via ros_gz_bridge) instead of computing
stub values. They implement the same SensorPlugin interface so the
sensor engine can seamlessly multiplex between Python and Gazebo sources.

Each bridge plugin has an update method called by the sensor engine node
when new ROS2 sensor data arrives from Gazebo, and a compute() method
that returns the latest data in the standard plugin dict format.

No ROS2 dependency — these are pure data containers + formatters.
"""

from __future__ import annotations

import math
import threading

from plato_pod.robot import Robot
from plato_pod.sensor_plugins.base import ArenaState, SensorConfig


class GazeboLidarBridge:
    """Passes through Gazebo lidar data in sensor plugin format.

    When Gazebo is running, the sensor engine node calls update_from_scan()
    with each incoming LaserScan message. compute() then returns the latest
    scan data instead of the Python stub's max-range values.

    Falls back to max-range stub if no Gazebo data has arrived yet.
    """

    name = "lidar_2d"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict | None = None

    def update_from_scan(
        self,
        ranges: list[float],
        angle_min: float,
        angle_max: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> None:
        """Update with data from a ROS2 LaserScan message.

        Called by the sensor engine node's ROS2 subscription callback.
        All parameters map directly to sensor_msgs/LaserScan fields.
        """
        with self._lock:
            self._latest = {
                "ranges": list(ranges),
                "angle_min": angle_min,
                "angle_max": angle_max,
                "angle_increment": angle_increment,
                "range_min": range_min,
                "range_max": range_max,
            }

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        **kwargs,
    ) -> dict:
        """Return latest Gazebo lidar data, or stub if unavailable."""
        with self._lock:
            if self._latest is not None:
                # Deep copy to prevent mutation of ranges list
                copy = dict(self._latest)
                copy["ranges"] = list(copy["ranges"])
                return copy

        # Fallback: max-range stub (same as LidarSensor)
        range_max = config.params.get("range_max", 2.0)
        resolution_deg = config.params.get("angular_resolution_deg", 1.0)
        n_rays = int(360.0 / resolution_deg)
        return {
            "ranges": [range_max] * n_rays,
            "angle_min": 0.0,
            "angle_max": 2.0 * math.pi,
            "angle_increment": math.radians(resolution_deg),
            "range_min": 0.01,
            "range_max": range_max,
        }

    def has_data(self) -> bool:
        """True if at least one Gazebo scan has been received."""
        with self._lock:
            return self._latest is not None

    def default_config(self) -> SensorConfig:
        return SensorConfig(
            rate_hz=10.0,
            noise_stddev=0.0,
            params={"range_max": 2.0, "angular_resolution_deg": 1.0},
        )


class GazeboImuBridge:
    """Passes through Gazebo IMU data in sensor plugin format.

    Returns linear acceleration and angular velocity from Gazebo's
    IMU sensor plugin via ros_gz_bridge.
    """

    name = "imu"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict | None = None

    def update_from_imu(
        self,
        linear_acceleration: tuple[float, float, float],
        angular_velocity: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> None:
        """Update with data from a ROS2 Imu message.

        Args:
            linear_acceleration: (x, y, z) in m/s^2
            angular_velocity: (x, y, z) in rad/s
            orientation: (x, y, z, w) quaternion
        """
        with self._lock:
            self._latest = {
                "linear_acceleration": {
                    "x": linear_acceleration[0],
                    "y": linear_acceleration[1],
                    "z": linear_acceleration[2],
                },
                "angular_velocity": {
                    "x": angular_velocity[0],
                    "y": angular_velocity[1],
                    "z": angular_velocity[2],
                },
                "orientation": {
                    "x": orientation[0],
                    "y": orientation[1],
                    "z": orientation[2],
                    "w": orientation[3],
                },
            }

    def compute(
        self,
        robot: Robot,
        arena: ArenaState,
        other_robots: list[Robot],
        config: SensorConfig,
        **kwargs,
    ) -> dict:
        """Return latest Gazebo IMU data, or defaults if unavailable."""
        with self._lock:
            if self._latest is not None:
                return dict(self._latest)

        # Fallback: stationary reading (gravity on Z)
        return {
            "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

    def has_data(self) -> bool:
        with self._lock:
            return self._latest is not None

    def default_config(self) -> SensorConfig:
        return SensorConfig(rate_hz=50.0, noise_stddev=0.0)
