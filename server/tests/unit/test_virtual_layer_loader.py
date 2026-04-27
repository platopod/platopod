"""Tests for plato_pod.virtual_layer_loader — YAML virtual layer loading."""

from __future__ import annotations

import pytest

from plato_pod.virtual_layer_loader import load_virtual_layers
from plato_pod.spatial_field import GaussianPlumeField, ElevationField, UniformField


class TestLoadVirtualLayersEmpty:
    def test_no_virtual_layers_returns_none(self) -> None:
        config = {"exercise": {"arena": {"boundary": []}}}
        assert load_virtual_layers(config) is None

    def test_empty_virtual_layers_returns_none(self) -> None:
        config = {"exercise": {"virtual_layers": {}}}
        result = load_virtual_layers(config)
        # Empty dict has no gas_sources or terrain, but environment defaults
        # should still produce a valid EnvironmentContext
        assert result is not None
        assert len(result.fields) == 0

    def test_no_exercise_wrapper(self) -> None:
        config = {"virtual_layers": {"gas_sources": [{"name": "gas", "x": 0.5, "y": 0.3}]}}
        result = load_virtual_layers(config)
        assert result is not None
        assert "gas" in result.fields


class TestLoadGasSources:
    def test_single_gas_source(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "gas_sources": [{
                        "name": "plume_alpha",
                        "x": 0.5,
                        "y": 0.3,
                        "release_rate": 100.0,
                        "wind_speed": 2.0,
                        "wind_direction": 0.0,
                        "diffusion_coeff": 0.05,
                    }],
                },
            },
        }
        env = load_virtual_layers(config)
        assert env is not None
        assert "plume_alpha" in env.fields
        field = env.fields["plume_alpha"]
        assert isinstance(field, GaussianPlumeField)
        assert field.source_x == pytest.approx(0.5)
        assert field.release_rate == pytest.approx(100.0)

    def test_multiple_gas_sources(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "gas_sources": [
                        {"name": "gas_a", "x": 0.1, "y": 0.1},
                        {"name": "gas_b", "x": 0.8, "y": 0.5},
                    ],
                },
            },
        }
        env = load_virtual_layers(config)
        assert env is not None
        assert "gas_a" in env.fields
        assert "gas_b" in env.fields

    def test_gas_source_defaults(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "gas_sources": [{"name": "gas"}],
                },
            },
        }
        env = load_virtual_layers(config)
        field = env.fields["gas"]
        assert isinstance(field, GaussianPlumeField)
        assert field.source_x == 0.0
        assert field.wind_speed == 2.0
        assert field.diffusion_coeff == 0.05


class TestLoadTerrain:
    def test_elevation_grid(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "terrain": {
                        "grid_data": [[0.0, 1.0], [2.0, 3.0]],
                        "origin": [0.0, 0.0],
                        "resolution": 1.0,
                    },
                },
            },
        }
        env = load_virtual_layers(config)
        assert env is not None
        assert "elevation" in env.fields
        field = env.fields["elevation"]
        assert isinstance(field, ElevationField)
        assert field.evaluate(0.5, 0.5, 0) == pytest.approx(1.5)

    def test_custom_terrain_name(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "terrain": {
                        "name": "hills",
                        "grid_data": [[1.0, 2.0]],
                        "origin": [0, 0],
                        "resolution": 0.5,
                    },
                },
            },
        }
        env = load_virtual_layers(config)
        assert "hills" in env.fields

    def test_empty_grid_skipped(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "terrain": {
                        "grid_data": [],
                        "origin": [0, 0],
                        "resolution": 1.0,
                    },
                },
            },
        }
        env = load_virtual_layers(config)
        assert "elevation" not in env.fields


class TestLoadEnvironment:
    def test_environment_params(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "environment": {
                        "wind_speed": 5.0,
                        "wind_direction": 1.57,
                        "temperature": 30.0,
                    },
                },
            },
        }
        env = load_virtual_layers(config)
        assert env is not None
        assert env.wind_speed == pytest.approx(5.0)
        assert env.wind_direction == pytest.approx(1.57)
        assert env.temperature == pytest.approx(30.0)

    def test_default_environment(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "gas_sources": [{"name": "gas"}],
                },
            },
        }
        env = load_virtual_layers(config)
        assert env.wind_speed == 0.0
        assert env.temperature == 20.0


class TestLoadBackgroundFields:
    def test_uniform_background(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "background_fields": [
                        {"name": "radiation", "value": 0.1},
                    ],
                },
            },
        }
        env = load_virtual_layers(config)
        assert "radiation" in env.fields
        field = env.fields["radiation"]
        assert isinstance(field, UniformField)
        assert field.evaluate(0, 0, 0) == pytest.approx(0.1)


class TestLoadCombined:
    def test_full_config(self) -> None:
        config = {
            "exercise": {
                "virtual_layers": {
                    "gas_sources": [
                        {"name": "gas", "x": 0.5, "y": 0.3, "release_rate": 200.0},
                    ],
                    "terrain": {
                        "grid_data": [[0, 0, 0], [0, 5, 0], [0, 0, 0]],
                        "origin": [0, 0],
                        "resolution": 0.5,
                    },
                    "background_fields": [
                        {"name": "temperature_field", "value": 25.0},
                    ],
                    "environment": {
                        "wind_speed": 3.0,
                        "wind_direction": 0.78,
                        "temperature": 28.0,
                    },
                },
            },
        }
        env = load_virtual_layers(config)
        assert env is not None
        assert len(env.fields) == 3  # gas, elevation, temperature_field
        assert "gas" in env.fields
        assert "elevation" in env.fields
        assert "temperature_field" in env.fields
        assert env.wind_speed == pytest.approx(3.0)
        assert env.temperature == pytest.approx(28.0)


class TestLoadPointSources:
    def test_ied_zones(self) -> None:
        env = load_virtual_layers({"virtual_layers": {
            "ied_zones": [
                {"position": [10.0, 20.0], "detectability_radius_m": 8,
                 "label": "alpha"},
                {"position": [30.0, 40.0]},
            ],
        }})
        assert env is not None
        assert len(env.point_sources["ied"]) == 2
        assert env.point_sources["ied"][0]["label"] == "alpha"
        assert env.point_sources["ied"][0]["position"] == (10.0, 20.0)

    def test_ew_emitters(self) -> None:
        env = load_virtual_layers({"virtual_layers": {
            "ew_emitters": [
                {"position": [50.0, 60.0], "frequency_mhz": 144,
                 "signal_strength": 0.7, "label": "EW1"},
            ],
        }})
        assert env is not None
        assert env.point_sources["ew_emitters"][0]["frequency_mhz"] == 144.0

    def test_civilian_population(self) -> None:
        env = load_virtual_layers({"virtual_layers": {
            "civilian_population": [
                {"position": [0.5, 0.5], "movement": "stationary",
                 "count": 5, "label": "market"},
            ],
        }})
        assert env is not None
        civs = env.point_sources["civilian"]
        assert len(civs) == 1
        assert civs[0]["count"] == 5
        assert civs[0]["movement"] == "stationary"

    def test_no_point_sources_section(self) -> None:
        env = load_virtual_layers({"virtual_layers": {
            "gas_sources": [{"name": "g", "x": 0, "y": 0}],
        }})
        assert env is not None
        assert env.point_sources == {}
