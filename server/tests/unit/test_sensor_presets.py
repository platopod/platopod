"""Tests for plato_pod.sensor_presets — preset definitions."""

from __future__ import annotations

from plato_pod.sensor_presets import get_preset, list_presets


class TestSensorPresets:
    def test_minimal_has_gps(self) -> None:
        preset = get_preset("minimal")
        assert preset is not None
        assert "gps" in preset
        assert len(preset) == 1

    def test_basic_scout_has_gps_and_sonar(self) -> None:
        preset = get_preset("basic_scout")
        assert preset is not None
        assert "gps" in preset
        assert "sonar" in preset

    def test_full_suite_has_all_sensors(self) -> None:
        preset = get_preset("full_suite")
        assert preset is not None
        assert "gps" in preset
        assert "lidar_2d" in preset
        assert "sonar" in preset
        assert "fof" in preset

    def test_ideal_has_zero_noise(self) -> None:
        preset = get_preset("ideal")
        assert preset is not None
        assert preset["gps"].noise_stddev == 0.0
        assert preset["lidar_2d"].noise_stddev == 0.0

    def test_unknown_preset_returns_none(self) -> None:
        assert get_preset("nonexistent") is None

    def test_gas_scout_has_gps_and_gas(self) -> None:
        preset = get_preset("gas_scout")
        assert preset is not None
        assert "gps" in preset
        assert "gas" in preset
        assert preset["gas"].params["response_time"] == 2.0

    def test_list_presets(self) -> None:
        names = list_presets()
        assert "minimal" in names
        assert "basic_scout" in names
        assert "full_suite" in names
        assert "ideal" in names
        assert "competitive" in names
        assert "gas_scout" in names
