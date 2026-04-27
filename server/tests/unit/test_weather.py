"""Tests for plato_pod.weather — atmospheric and visibility state."""

from __future__ import annotations

import pytest

from plato_pod.weather import CLEAR, WeatherState, from_dict, visibility_factor


class TestWeatherState:
    def test_default_clear(self) -> None:
        w = WeatherState()
        assert w.fog_density == 0.0
        assert w.wind_speed == 0.0

    def test_clear_constant(self) -> None:
        assert CLEAR.fog_density == 0.0
        assert CLEAR.visibility_m >= 1000.0


class TestVisibilityFactor:
    def test_short_distance_full_visibility(self) -> None:
        assert visibility_factor(CLEAR, 1.0) == pytest.approx(1.0, abs=0.001)

    def test_zero_distance_full(self) -> None:
        assert visibility_factor(CLEAR, 0.0) == 1.0

    def test_negative_distance_full(self) -> None:
        assert visibility_factor(CLEAR, -1.0) == 1.0

    def test_beyond_max_zero(self) -> None:
        w = WeatherState(visibility_m=100.0)
        assert visibility_factor(w, 100.0) == 0.0
        assert visibility_factor(w, 200.0) == 0.0

    def test_fog_attenuation(self) -> None:
        no_fog = WeatherState(visibility_m=1000.0, fog_density=0.0)
        heavy_fog = WeatherState(visibility_m=1000.0, fog_density=0.8)
        assert visibility_factor(heavy_fog, 100.0) < visibility_factor(no_fog, 100.0)

    def test_full_fog_zero_visibility(self) -> None:
        w = WeatherState(visibility_m=1000.0, fog_density=1.0)
        assert visibility_factor(w, 100.0) == 0.0

    def test_monotonic_decreasing_with_distance(self) -> None:
        prev = 2.0
        for d in [0, 100, 200, 300, 500, 800]:
            v = visibility_factor(CLEAR, d)
            assert v <= prev
            prev = v


class TestFromDict:
    def test_minimal(self) -> None:
        w = from_dict({})
        assert w.visibility_m == 1000.0
        assert w.fog_density == 0.0

    def test_full(self) -> None:
        w = from_dict({
            "visibility_m": 500,
            "wind_speed": 5.0,
            "wind_direction": 1.57,
            "fog_density": 0.3,
            "time_of_day": 18.0,
        })
        assert w.visibility_m == 500.0
        assert w.wind_speed == 5.0
        assert w.fog_density == pytest.approx(0.3)
