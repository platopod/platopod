"""Sensor preset definitions for the Plato Pod platform.

Named presets define which sensors a robot has and their configuration.
No ROS2 dependency.
"""

from __future__ import annotations

from plato_pod.sensor_plugins.base import SensorConfig

# Each preset maps sensor_name -> SensorConfig
PRESETS: dict[str, dict[str, SensorConfig]] = {
    "minimal": {
        "gps": SensorConfig(rate_hz=10.0, noise_stddev=0.001,
                            params={"heading_noise_stddev": 0.01}),
    },
    "basic_scout": {
        "gps": SensorConfig(rate_hz=10.0, noise_stddev=0.001,
                            params={"heading_noise_stddev": 0.01}),
        "sonar": SensorConfig(rate_hz=10.0, noise_stddev=0.02,
                              params={"range_max": 1.5, "segment_angle_deg": 10.0}),
    },
    "full_suite": {
        "gps": SensorConfig(rate_hz=10.0, noise_stddev=0.001,
                            params={"heading_noise_stddev": 0.01}),
        "lidar_2d": SensorConfig(rate_hz=10.0, noise_stddev=0.005,
                                 params={"range_max": 2.0, "angular_resolution_deg": 1.0}),
        "sonar": SensorConfig(rate_hz=10.0, noise_stddev=0.02,
                              params={"range_max": 1.5, "segment_angle_deg": 10.0}),
        "fof": SensorConfig(rate_hz=5.0, noise_stddev=0.0,
                            params={"range_max": 1.0, "fov_deg": 360.0}),
    },
    "ideal": {
        "gps": SensorConfig(rate_hz=10.0, noise_stddev=0.0,
                            params={"heading_noise_stddev": 0.0}),
        "lidar_2d": SensorConfig(rate_hz=10.0, noise_stddev=0.0,
                                 params={"range_max": 2.0, "angular_resolution_deg": 1.0}),
    },
    "competitive": {
        "gps": SensorConfig(rate_hz=10.0, noise_stddev=0.005,
                            params={"heading_noise_stddev": 0.05}),
        "sonar": SensorConfig(rate_hz=10.0, noise_stddev=0.05,
                              params={"range_max": 1.0, "segment_angle_deg": 10.0}),
        "fof": SensorConfig(rate_hz=5.0, noise_stddev=0.0,
                            params={"range_max": 0.5, "fov_deg": 180.0}),
    },
    "gas_scout": {
        "gps": SensorConfig(rate_hz=10.0, noise_stddev=0.001,
                            params={"heading_noise_stddev": 0.01}),
        "gas": SensorConfig(rate_hz=10.0, noise_stddev=0.5,
                            params={
                                "response_time": 2.0,
                                "dt": 0.1,
                                "r_clean": 1.0,
                                "r_gas": 0.1,
                                "saturation_concentration": 1000.0,
                                "field_name": "gas",
                            }),
    },
}


def get_preset(name: str) -> dict[str, SensorConfig] | None:
    """Get a sensor preset by name.

    Returns:
        Dict of sensor_name -> SensorConfig, or None if preset not found.
    """
    return PRESETS.get(name)


def list_presets() -> list[str]:
    """Return available preset names."""
    return list(PRESETS.keys())
