"""Tests for plato_pod.line_of_sight — terrain + cover-aware visibility."""

from __future__ import annotations

import pytest

from plato_pod.line_of_sight import (
    CoverPolygon,
    LosResult,
    has_line_of_sight,
)
from plato_pod.spatial_field import ElevationField
from plato_pod.weather import CLEAR, WeatherState


def _flat_terrain(height: float = 0.0, size: int = 10) -> ElevationField:
    """ElevationField with constant height across a 100x100 area."""
    grid = [[height] * size for _ in range(size)]
    return ElevationField(grid_data=grid, origin_x=0, origin_y=0, resolution=10.0)


def _ridge_terrain() -> ElevationField:
    """ElevationField with a tall ridge in the middle."""
    grid = [[0.0] * 10 for _ in range(10)]
    # Ridge at row 5
    for col in range(10):
        grid[5][col] = 50.0
    return ElevationField(grid_data=grid, origin_x=0, origin_y=0, resolution=10.0)


class TestHasLineOfSightBasic:
    def test_clear_short_distance(self) -> None:
        result = has_line_of_sight((0, 0, 1), (10, 0, 1), weather=CLEAR)
        assert isinstance(result, LosResult)
        assert result.visible
        assert result.attenuation > 0.99

    def test_zero_distance(self) -> None:
        result = has_line_of_sight((5, 5, 1), (5, 5, 1), weather=CLEAR)
        assert result.visible


class TestCoverPolygons:
    def test_blocked_by_full_cover(self) -> None:
        cover = CoverPolygon(
            vertices=[(40, -10), (60, -10), (60, 10), (40, 10)],
            cover_value=1.0,
            label="building",
        )
        result = has_line_of_sight((0, 0, 1), (100, 0, 1), cover_polygons=[cover])
        assert not result.visible
        assert "building" in result.rationale

    def test_partial_cover_attenuates(self) -> None:
        cover = CoverPolygon(
            vertices=[(40, -10), (60, -10), (60, 10), (40, 10)],
            cover_value=0.5,
            label="trees",
        )
        result = has_line_of_sight((0, 0, 1), (100, 0, 1), cover_polygons=[cover])
        assert result.visible
        assert result.attenuation < 1.0
        assert result.attenuation == pytest.approx(0.5, abs=0.01)

    def test_cover_outside_path_no_effect(self) -> None:
        # Cover is at y=50, far from line which goes y=0
        cover = CoverPolygon(
            vertices=[(40, 50), (60, 50), (60, 60), (40, 60)],
            cover_value=1.0,
        )
        # Use CLEAR weather (visibility_m=10000) so distance attenuation is negligible
        result = has_line_of_sight(
            (0, 0, 1), (100, 0, 1),
            cover_polygons=[cover], weather=CLEAR,
        )
        assert result.visible
        assert result.attenuation > 0.98


class TestWeather:
    def test_fog_blocks_long_distance(self) -> None:
        weather = WeatherState(visibility_m=50.0, fog_density=0.5)
        result = has_line_of_sight((0, 0, 1), (100, 0, 1), weather=weather)
        assert not result.visible
        assert result.rationale == "out_of_visual_range"

    def test_fog_short_distance_still_visible(self) -> None:
        weather = WeatherState(visibility_m=200.0, fog_density=0.3)
        result = has_line_of_sight((0, 0, 1), (50, 0, 1), weather=weather)
        assert result.visible
        # but attenuated
        assert result.attenuation < 1.0


class TestTerrainOcclusion:
    def test_ridge_blocks_view(self) -> None:
        terrain = _ridge_terrain()
        # Observer at (10,10), target at (10,90), ridge at y=50 with height 50
        # Both at z=1; ridge at z=50 blocks them
        result = has_line_of_sight((10, 10, 1), (10, 90, 1), terrain=terrain)
        assert not result.visible
        assert result.rationale == "blocked_by_terrain"

    def test_above_ridge_visible(self) -> None:
        terrain = _ridge_terrain()
        # Both observers above the ridge (z=100)
        result = has_line_of_sight((10, 10, 100), (10, 90, 100), terrain=terrain)
        assert result.visible

    def test_flat_terrain_no_block(self) -> None:
        terrain = _flat_terrain(0.0)
        result = has_line_of_sight((10, 10, 1), (10, 90, 1), terrain=terrain)
        assert result.visible


class TestComposed:
    def test_cover_and_weather(self) -> None:
        cover = CoverPolygon(
            vertices=[(40, -10), (60, -10), (60, 10), (40, 10)],
            cover_value=0.4,
        )
        weather = WeatherState(visibility_m=200.0, fog_density=0.0)
        result = has_line_of_sight(
            (0, 0, 1), (100, 0, 1),
            cover_polygons=[cover],
            weather=weather,
        )
        # cover * weather distance attenuation
        assert result.visible
        # 0.6 * (1 - 100/200) = 0.6 * 0.5 = 0.3
        assert result.attenuation == pytest.approx(0.3, abs=0.05)
