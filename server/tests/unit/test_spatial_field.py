"""Tests for plato_pod.spatial_field — spatial field implementations."""

from __future__ import annotations

import math

import pytest

from plato_pod.spatial_field import (
    CompositeField,
    ElevationField,
    GaussianPlumeField,
    UniformField,
    compute_iso_contour,
)


# --- UniformField ---

class TestUniformField:
    def test_returns_constant(self) -> None:
        f = UniformField(value=42.0)
        assert f.evaluate(0, 0, 0) == 42.0
        assert f.evaluate(100, -50, 999) == 42.0

    def test_metadata(self) -> None:
        f = UniformField(value=5.0, name="background")
        meta = f.get_metadata()
        assert meta["name"] == "background"
        assert meta["type"] == "uniform"
        assert meta["value"] == 5.0


# --- GaussianPlumeField ---

class TestGaussianPlumeField:
    def _plume(self, **kwargs) -> GaussianPlumeField:
        defaults = dict(
            source_x=0.0, source_y=0.0,
            release_rate=100.0,
            wind_speed=2.0,
            wind_direction=0.0,  # wind blows in +X direction
            diffusion_coeff=0.05,
        )
        defaults.update(kwargs)
        return GaussianPlumeField(**defaults)

    def test_upwind_is_zero(self) -> None:
        p = self._plume(wind_direction=0.0)
        # Point upwind (negative X from source)
        assert p.evaluate(-1.0, 0.0, 0.0) == 0.0

    def test_downwind_center_positive(self) -> None:
        p = self._plume(wind_direction=0.0)
        c = p.evaluate(0.5, 0.0, 0.0)
        assert c > 0.0

    def test_concentration_decreases_downwind(self) -> None:
        p = self._plume(wind_direction=0.0)
        c_near = p.evaluate(0.2, 0.0, 0.0)
        c_far = p.evaluate(2.0, 0.0, 0.0)
        assert c_near > c_far

    def test_concentration_decreases_crosswind(self) -> None:
        p = self._plume(wind_direction=0.0)
        c_center = p.evaluate(1.0, 0.0, 0.0)
        c_offset = p.evaluate(1.0, 0.5, 0.0)
        assert c_center > c_offset

    def test_symmetric_crosswind(self) -> None:
        p = self._plume(wind_direction=0.0)
        c_pos = p.evaluate(1.0, 0.3, 0.0)
        c_neg = p.evaluate(1.0, -0.3, 0.0)
        assert c_pos == pytest.approx(c_neg)

    def test_zero_wind_returns_zero(self) -> None:
        p = self._plume(wind_speed=0.0)
        assert p.evaluate(0.5, 0.0, 0.0) == 0.0

    def test_zero_release_returns_zero(self) -> None:
        p = self._plume(release_rate=0.0)
        assert p.evaluate(0.5, 0.0, 0.0) == 0.0

    def test_wind_direction_rotates_plume(self) -> None:
        # Wind blows in +Y direction (pi/2 radians)
        p = self._plume(wind_direction=math.pi / 2)
        # Downwind is now +Y direction
        c_downwind = p.evaluate(0.0, 1.0, 0.0)
        assert c_downwind > 0.0
        # Upwind (+X direction relative to old) should be zero or near-zero
        c_crosswind = p.evaluate(1.0, 0.0, 0.0)
        # The crosswind point is at 90 degrees, not upwind
        assert c_downwind > c_crosswind

    def test_at_source_returns_zero(self) -> None:
        p = self._plume()
        # x_wind = 0 at source, should return 0 (not divide by zero)
        assert p.evaluate(0.0, 0.0, 0.0) == 0.0

    def test_higher_release_rate_higher_concentration(self) -> None:
        p_low = self._plume(release_rate=10.0)
        p_high = self._plume(release_rate=100.0)
        point = (0.5, 0.0, 0.0)
        assert p_high.evaluate(*point) > p_low.evaluate(*point)

    def test_metadata(self) -> None:
        p = self._plume()
        meta = p.get_metadata()
        assert meta["type"] == "gaussian_plume"
        assert meta["source_x"] == 0.0
        assert meta["release_rate"] == 100.0

    def test_never_negative(self) -> None:
        p = self._plume()
        for x in [-1, 0, 0.1, 0.5, 1, 5]:
            for y in [-2, -1, 0, 1, 2]:
                assert p.evaluate(x, y, 0) >= 0.0


# --- ElevationField ---

class TestElevationField:
    def _flat_grid(self, value: float = 0.0) -> ElevationField:
        grid = [[value] * 10 for _ in range(10)]
        return ElevationField(
            grid_data=grid, origin_x=0.0, origin_y=0.0, resolution=0.1,
        )

    def _slope_grid(self) -> ElevationField:
        # 10x10 grid, elevation increases linearly with X
        grid = [[col * 1.0 for col in range(10)] for _ in range(10)]
        return ElevationField(
            grid_data=grid, origin_x=0.0, origin_y=0.0, resolution=1.0,
        )

    def test_flat_grid_returns_constant(self) -> None:
        f = self._flat_grid(5.0)
        assert f.evaluate(0.5, 0.5, 0) == pytest.approx(5.0)

    def test_slope_increases_with_x(self) -> None:
        f = self._slope_grid()
        e1 = f.evaluate(2.0, 5.0, 0)
        e2 = f.evaluate(7.0, 5.0, 0)
        assert e2 > e1

    def test_bilinear_interpolation(self) -> None:
        # 2x2 grid: corners at 0, 1, 2, 3
        grid = [[0.0, 1.0], [2.0, 3.0]]
        f = ElevationField(grid_data=grid, origin_x=0, origin_y=0, resolution=1.0)
        # Center should be average of all four
        assert f.evaluate(0.5, 0.5, 0) == pytest.approx(1.5)

    def test_outside_grid_returns_edge(self) -> None:
        f = self._flat_grid(10.0)
        # Far outside should clamp to edge or return 0
        result = f.evaluate(-100, -100, 0)
        # Should return the nearest edge value (10.0) or 0.0
        assert result in (0.0, 10.0)

    def test_empty_grid_returns_zero(self) -> None:
        f = ElevationField(grid_data=[], origin_x=0, origin_y=0, resolution=1.0)
        assert f.evaluate(0, 0, 0) == 0.0

    def test_time_ignored(self) -> None:
        f = self._flat_grid(3.0)
        assert f.evaluate(0.5, 0.5, 0) == f.evaluate(0.5, 0.5, 1000)

    def test_metadata(self) -> None:
        f = self._slope_grid()
        meta = f.get_metadata()
        assert meta["type"] == "elevation_grid"
        assert meta["rows"] == 10
        assert meta["cols"] == 10
        assert meta["resolution"] == 1.0

    def test_exact_grid_point(self) -> None:
        grid = [[0.0, 10.0], [20.0, 30.0]]
        f = ElevationField(grid_data=grid, origin_x=0, origin_y=0, resolution=1.0)
        assert f.evaluate(0.0, 0.0, 0) == pytest.approx(0.0)
        assert f.evaluate(1.0, 0.0, 0) == pytest.approx(10.0)


# --- CompositeField ---

class TestCompositeField:
    def test_empty_returns_zero(self) -> None:
        f = CompositeField()
        assert f.evaluate(0, 0, 0) == 0.0

    def test_sums_fields(self) -> None:
        f = CompositeField(fields=[
            UniformField(value=10.0),
            UniformField(value=5.0),
        ])
        assert f.evaluate(0, 0, 0) == pytest.approx(15.0)

    def test_mixed_fields(self) -> None:
        plume = GaussianPlumeField(
            source_x=0, source_y=0,
            release_rate=100, wind_speed=2, wind_direction=0,
            diffusion_coeff=0.05,
        )
        background = UniformField(value=1.0)
        f = CompositeField(fields=[plume, background])
        # Downwind should be > 1.0 (background + plume)
        assert f.evaluate(0.5, 0.0, 0) > 1.0
        # Upwind should be exactly 1.0 (background only)
        assert f.evaluate(-1.0, 0.0, 0) == pytest.approx(1.0)

    def test_metadata(self) -> None:
        f = CompositeField(fields=[UniformField(1.0), UniformField(2.0)])
        meta = f.get_metadata()
        assert meta["type"] == "composite"
        assert meta["num_fields"] == 2
        assert len(meta["fields"]) == 2


# --- compute_iso_contour ---

class TestComputeIsoContour:
    def test_uniform_above_threshold(self) -> None:
        f = UniformField(value=100.0)
        contour = compute_iso_contour(f, 50.0, (0, 0, 1, 1), resolution=0.1)
        assert len(contour) >= 3  # convex hull of grid points

    def test_uniform_below_threshold_empty(self) -> None:
        f = UniformField(value=10.0)
        contour = compute_iso_contour(f, 50.0, (0, 0, 1, 1), resolution=0.1)
        assert len(contour) == 0

    def test_plume_contour_surrounds_source(self) -> None:
        p = GaussianPlumeField(
            source_x=0.5, source_y=0.5,
            release_rate=1000.0, wind_speed=2.0,
            wind_direction=0.0, diffusion_coeff=0.1,
        )
        contour = compute_iso_contour(p, 1.0, (0, 0, 2, 1), resolution=0.05)
        assert len(contour) >= 3
        # Contour should be downwind of source (x > 0.5)
        xs = [pt[0] for pt in contour]
        assert min(xs) >= 0.45  # near or past source

    def test_higher_threshold_smaller_area(self) -> None:
        from plato_pod.geometry import polygon_area
        p = GaussianPlumeField(
            source_x=0.5, source_y=0.5,
            release_rate=1000.0, wind_speed=2.0,
            wind_direction=0.0, diffusion_coeff=0.1,
        )
        low = compute_iso_contour(p, 1.0, (0, 0, 2, 1), resolution=0.05)
        high = compute_iso_contour(p, 10.0, (0, 0, 2, 1), resolution=0.05)
        assert len(low) >= 3
        assert len(high) >= 3
        assert abs(polygon_area(low)) > abs(polygon_area(high))
