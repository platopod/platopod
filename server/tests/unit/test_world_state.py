"""Tests for plato_pod.world_state — exercise YAML → WorldState aggregation."""

from __future__ import annotations

from plato_pod.weather import CLEAR
from plato_pod.world_state import WorldState, world_state_from_config


class TestEmpty:
    def test_no_config(self) -> None:
        ws = world_state_from_config({})
        assert ws.cover == []
        assert ws.civilians == []
        assert ws.weapons == {}
        assert ws.roe.fire_permission == "weapons_free"

    def test_no_virtual_layers(self) -> None:
        ws = world_state_from_config({"exercise": {"id": "x"}})
        assert isinstance(ws, WorldState)


class TestCover:
    def test_cover_polygons_loaded(self) -> None:
        ws = world_state_from_config({"exercise": {"virtual_layers": {
            "cover_polygons": [
                {"vertices": [[0, 0], [10, 0], [10, 10], [0, 10]],
                 "cover_value": 0.7, "label": "wall"},
            ],
        }}})
        assert len(ws.cover) == 1
        assert ws.cover[0].label == "wall"
        assert ws.cover[0].cover_value == 0.7

    def test_bad_polygon_skipped(self) -> None:
        ws = world_state_from_config({"exercise": {"virtual_layers": {
            "cover_polygons": [
                {"label": "no_vertices"},
                {"vertices": [[0, 0], [10, 0], [10, 10]],
                 "cover_value": 1.0, "label": "good"},
            ],
        }}})
        assert len(ws.cover) == 1
        assert ws.cover[0].label == "good"


class TestCivilians:
    def test_loaded(self) -> None:
        ws = world_state_from_config({"exercise": {"virtual_layers": {
            "civilian_population": [
                {"position": [0.5, 0.5], "label": "shopkeeper", "count": 1},
                {"position": [0.6, 0.6], "movement": "random", "count": 5},
            ],
        }}})
        assert len(ws.civilians) == 2
        assert ws.civilians[0]["label"] == "shopkeeper"
        assert ws.civilians[1]["movement"] == "random"

    def test_skips_no_position(self) -> None:
        ws = world_state_from_config({"exercise": {"virtual_layers": {
            "civilian_population": [{"label": "ghost"}],
        }}})
        assert ws.civilians == []


class TestIed:
    def test_ied_zones(self) -> None:
        ws = world_state_from_config({"exercise": {"virtual_layers": {
            "ied_zones": [
                {"position": [10, 20], "detectability_radius_m": 8,
                 "label": "alpha"},
            ],
        }}})
        assert len(ws.ied_zones) == 1
        assert ws.ied_zones[0]["detectability_radius_m"] == 8.0


class TestEwAndJamming:
    def test_ew_emitters(self) -> None:
        ws = world_state_from_config({"exercise": {"virtual_layers": {
            "ew_emitters": [
                {"position": [50, 60], "frequency_mhz": 144,
                 "signal_strength": 0.7},
            ],
        }}})
        assert ws.ew_emitters[0]["frequency_mhz"] == 144.0

    def test_jamming_zones(self) -> None:
        ws = world_state_from_config({"exercise": {"virtual_layers": {
            "jamming_zones": [
                {"position": [100, 100], "radius_m": 50, "strength": 0.8,
                 "label": "EW1"},
            ],
        }}})
        assert len(ws.jamming_zones) == 1
        assert ws.jamming_zones[0].label == "EW1"

    def test_dead_zones(self) -> None:
        ws = world_state_from_config({"exercise": {"virtual_layers": {
            "comms_dead_zones": [
                {"vertices": [[0, 0], [10, 0], [10, 10], [0, 10]],
                 "label": "ridge"},
            ],
        }}})
        assert len(ws.dead_zones) == 1
        assert ws.dead_zones[0].label == "ridge"


class TestWeather:
    def test_default_clear(self) -> None:
        ws = world_state_from_config({})
        assert ws.weather.fog_density == CLEAR.fog_density

    def test_custom_weather(self) -> None:
        ws = world_state_from_config({"exercise": {"weather": {
            "visibility_m": 500, "fog_density": 0.4,
        }}})
        assert ws.weather.visibility_m == 500.0
        assert ws.weather.fog_density == 0.4


class TestRoe:
    def test_default(self) -> None:
        ws = world_state_from_config({})
        assert ws.roe.fire_permission == "weapons_free"

    def test_custom(self) -> None:
        ws = world_state_from_config({"exercise": {"roe": {
            "fire_permission": "weapons_tight",
            "civilian_proximity_m": 25,
            "require_target_id": True,
        }}})
        assert ws.roe.fire_permission == "weapons_tight"
        assert ws.roe.civilian_proximity_m == 25.0
        assert ws.roe.require_target_id


class TestWeapons:
    def test_catalog(self) -> None:
        ws = world_state_from_config({"exercise": {"weapons": {
            "M4": {"max_range_m": 400, "base_pok_at_100m": 0.6},
            "AK74": {"max_range_m": 350, "base_pok_at_100m": 0.5},
        }}})
        assert "M4" in ws.weapons
        assert "AK74" in ws.weapons
        assert ws.weapons["M4"].max_range_m == 400.0


class TestFullScenario:
    def test_loads_cordon_and_search_yaml(self) -> None:
        # Just verify the canonical scenarios parse without exception
        import yaml
        from pathlib import Path
        repo_root = Path(__file__).resolve().parents[3].parent
        yaml_path = repo_root / "config" / "exercises" / "cordon-and-search.yaml"
        if not yaml_path.exists():
            return  # skip when run from a partial checkout
        config = yaml.safe_load(yaml_path.read_text())
        ws = world_state_from_config(config)
        assert len(ws.cover) >= 1
        assert len(ws.civilians) >= 1
        assert len(ws.ied_zones) >= 1
        assert ws.roe.fire_permission == "weapons_tight"
