"""Sensor plugins for the Plato Pod sensor engine."""

from plato_pod.sensor_plugins.base import SensorConfig, SensorPlugin
from plato_pod.sensor_plugins.df_receiver import DfReceiverSensor
from plato_pod.sensor_plugins.fof import FofSensor
from plato_pod.sensor_plugins.gas import GasSensor
from plato_pod.sensor_plugins.gps import GpsSensor
from plato_pod.sensor_plugins.ied_detector import IedDetectorSensor
from plato_pod.sensor_plugins.lidar import LidarSensor
from plato_pod.sensor_plugins.rangefinder import RangefinderSensor
from plato_pod.sensor_plugins.sonar import SonarSensor
from plato_pod.sensor_plugins.thermal import ThermalSensor
from plato_pod.sensor_plugins.uav_camera import UavCameraSensor

ALL_PLUGINS = {
    "gps": GpsSensor,
    "lidar_2d": LidarSensor,
    "sonar": SonarSensor,
    "fof": FofSensor,
    "gas": GasSensor,
    "rangefinder": RangefinderSensor,
    "thermal": ThermalSensor,
    "ied_detector": IedDetectorSensor,
    "df_receiver": DfReceiverSensor,
    "uav_camera": UavCameraSensor,
}

__all__ = [
    "SensorPlugin", "SensorConfig", "ALL_PLUGINS",
    "GpsSensor", "LidarSensor", "SonarSensor", "FofSensor", "GasSensor",
    "RangefinderSensor", "ThermalSensor", "IedDetectorSensor",
    "DfReceiverSensor", "UavCameraSensor",
]
