"""Tests for plato_pod.terrain_pipeline — DEM to Gazebo heightmap conversion."""

from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np
import pytest

from plato_pod.terrain_pipeline import (
    BoundingBox,
    TerrainConfig,
    TerrainResult,
    generate_terrain,
    resample_elevation,
    terrain_config_from_yaml,
    validate_heightmap_size,
)


class TestValidateHeightmapSize:
    def test_already_valid(self) -> None:
        assert validate_heightmap_size(257) == 257  # 2^8 + 1
        assert validate_heightmap_size(129) == 129  # 2^7 + 1
        assert validate_heightmap_size(513) == 513  # 2^9 + 1

    def test_rounds_up(self) -> None:
        assert validate_heightmap_size(200) == 257
        assert validate_heightmap_size(100) == 129
        assert validate_heightmap_size(4) == 5  # 2^2 + 1

    def test_minimum(self) -> None:
        assert validate_heightmap_size(1) == 3  # 2^1 + 1
        assert validate_heightmap_size(2) == 3


class TestBoundingBox:
    def test_dimensions_deg(self) -> None:
        bb = BoundingBox(north=-35.29, south=-35.31, east=149.11, west=149.09)
        assert bb.width_deg == pytest.approx(0.02)
        assert bb.height_deg == pytest.approx(0.02)

    def test_dimensions_metres(self) -> None:
        bb = BoundingBox(north=-35.29, south=-35.31, east=149.11, west=149.09)
        # ~0.02 deg lat ≈ 2220m, ~0.02 deg lon at -35° ≈ 1830m
        assert 2000 < bb.height_m() < 2500
        assert 1500 < bb.width_m() < 2200


class TestResampleElevation:
    def test_identity(self) -> None:
        data = np.array([[1.0, 2.0], [3.0, 4.0]])
        result = resample_elevation(data, 2)
        np.testing.assert_array_almost_equal(result, data)

    def test_upsample(self) -> None:
        data = np.array([[0.0, 10.0], [20.0, 30.0]])
        result = resample_elevation(data, 5)
        assert result.shape == (5, 5)
        # Corners should match
        assert result[0, 0] == pytest.approx(0.0)
        assert result[0, -1] == pytest.approx(10.0)
        assert result[-1, 0] == pytest.approx(20.0)
        assert result[-1, -1] == pytest.approx(30.0)
        # Center should be average
        assert result[2, 2] == pytest.approx(15.0)

    def test_downsample(self) -> None:
        data = np.random.rand(100, 100)
        result = resample_elevation(data, 17)
        assert result.shape == (17, 17)


class TestElevationToPng:
    def test_writes_file(self) -> None:
        try:
            from PIL import Image
        except ImportError:
            pytest.skip("Pillow not available")

        from plato_pod.terrain_pipeline import elevation_to_png

        data = np.array([[0.0, 10.0], [20.0, 30.0]])
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "test.png"
            e_min, e_max = elevation_to_png(data, path)
            assert path.exists()
            assert e_min == pytest.approx(0.0)
            assert e_max == pytest.approx(30.0)

            # Verify it's a valid image
            img = Image.open(path)
            assert img.size == (2, 2)

    def test_flat_terrain(self) -> None:
        try:
            from PIL import Image
        except ImportError:
            pytest.skip("Pillow not available")

        from plato_pod.terrain_pipeline import elevation_to_png

        data = np.ones((5, 5)) * 100.0
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "flat.png"
            elevation_to_png(data, path)
            assert path.exists()


class TestGenerateTerrain:
    def test_flat_returns_none(self) -> None:
        config = TerrainConfig(source="flat")
        result = generate_terrain(config, Path("/tmp"))
        assert result is None

    def test_numpy_source(self) -> None:
        try:
            from PIL import Image
        except ImportError:
            pytest.skip("Pillow not available")

        # Create a synthetic elevation array
        elevation = np.random.rand(33, 33) * 50.0
        with tempfile.TemporaryDirectory() as tmpdir:
            npy_path = Path(tmpdir) / "dem.npy"
            np.save(npy_path, elevation)

            config = TerrainConfig(
                source="numpy",
                geotiff_path=str(npy_path),
                heightmap_size=17,
                max_elevation_m=50.0,
            )
            result = generate_terrain(config, Path(tmpdir))
            assert result is not None
            assert result.heightmap_path.exists()
            assert result.size_z == 50.0

    def test_numpy_with_bounding_box(self) -> None:
        try:
            from PIL import Image
        except ImportError:
            pytest.skip("Pillow not available")

        elevation = np.random.rand(33, 33) * 30.0
        bbox = BoundingBox(north=-35.29, south=-35.31, east=149.11, west=149.09)

        with tempfile.TemporaryDirectory() as tmpdir:
            npy_path = Path(tmpdir) / "dem.npy"
            np.save(npy_path, elevation)

            config = TerrainConfig(
                source="numpy",
                geotiff_path=str(npy_path),
                heightmap_size=17,
                max_elevation_m=30.0,
                bounding_box=bbox,
            )
            result = generate_terrain(config, Path(tmpdir))
            assert result is not None
            # Size should be based on bounding box
            assert result.size_x > 1000  # ~2.2km
            assert result.size_y > 1000  # ~1.8km


class TestTerrainConfigFromYaml:
    def test_no_terrain_returns_none(self) -> None:
        config = {"exercise": {"arena": {"boundary": []}}}
        assert terrain_config_from_yaml(config) is None

    def test_flat_source(self) -> None:
        config = {"exercise": {"terrain": {"source": "flat"}}}
        tc = terrain_config_from_yaml(config)
        assert tc is not None
        assert tc.source == "flat"

    def test_geotiff_source(self) -> None:
        config = {
            "exercise": {
                "terrain": {
                    "source": "geotiff",
                    "geotiff_path": "/data/canberra.tif",
                    "bounding_box": {
                        "north": -35.29,
                        "south": -35.31,
                        "east": 149.11,
                        "west": 149.09,
                    },
                    "max_elevation": 40.0,
                    "heightmap_size": 513,
                },
            },
        }
        tc = terrain_config_from_yaml(config)
        assert tc is not None
        assert tc.source == "geotiff"
        assert tc.geotiff_path == "/data/canberra.tif"
        assert tc.bounding_box is not None
        assert tc.bounding_box.north == pytest.approx(-35.29)
        assert tc.heightmap_size == 513
        assert tc.max_elevation_m == pytest.approx(40.0)

    def test_defaults(self) -> None:
        config = {"exercise": {"terrain": {"source": "geotiff", "geotiff_path": "x.tif"}}}
        tc = terrain_config_from_yaml(config)
        assert tc.heightmap_size == 257
        assert tc.max_elevation_m == 50.0
        assert tc.bounding_box is None
