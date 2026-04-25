"""Sensor plugins for the Plato Pod sensor engine."""

from plato_pod.sensor_plugins.base import SensorConfig, SensorPlugin
from plato_pod.sensor_plugins.gps import GpsSensor
from plato_pod.sensor_plugins.lidar import LidarSensor
from plato_pod.sensor_plugins.sonar import SonarSensor
from plato_pod.sensor_plugins.fof import FofSensor
from plato_pod.sensor_plugins.gas import GasSensor

ALL_PLUGINS = {
    "gps": GpsSensor,
    "lidar_2d": LidarSensor,
    "sonar": SonarSensor,
    "fof": FofSensor,
    "gas": GasSensor,
}

__all__ = [
    "SensorPlugin", "SensorConfig", "ALL_PLUGINS",
    "GpsSensor", "LidarSensor", "SonarSensor", "FofSensor", "GasSensor",
]
