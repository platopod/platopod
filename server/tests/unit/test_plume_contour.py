"""Tests for plato_pod.plume_contour — SpatialField → CoT-ready polygons."""

from __future__ import annotations

import math

import pytest

from plato_pod.plume_contour import (
    ContourLevel,
    extract_contours,
    fit_ellipse,
)
from plato_pod.spatial_field import GaussianPlumeField, UniformField


# ---------- helpers --------------------------------------------------------

class _PointPlume:
    """Toy field: triangle bump at the origin, max value at (0,0)."""

    def __init__(self, peak: float = 1000.0, falloff_per_m: float = 10.0) -> None:
        self.peak = peak
        self.falloff_per_m = falloff_per_m

    def evaluate(self, x: float, y: float, t: float) -> float:
        return max(0.0, self.peak - self.falloff_per_m * math.hypot(x, y))


# ---------- empty / degenerate ---------------------------------------------

class TestEmpty:
    def test_no_thresholds(self) -> None:
        field = UniformField(value=500.0)
        out = extract_contours(field, (-10, -10, 10, 10), [])
        assert out == []

    def test_no_cells_above_threshold(self) -> None:
        field = UniformField(value=10.0)
        out = extract_contours(field, (0, 0, 10, 10), [1000.0])
        assert len(out) == 1
        assert out[0].threshold == 1000.0
        assert out[0].polygons == []

    def test_zero_span_bbox(self) -> None:
        field = UniformField(value=500.0)
        out = extract_contours(field, (5, 5, 5, 5), [100.0])
        assert len(out) == 1
        assert out[0].polygons == []


# ---------- single-component plumes ----------------------------------------

class TestSingleComponent:
    def test_uniform_field_above_threshold_covers_whole_bbox(self) -> None:
        field = UniformField(value=500.0)
        out = extract_contours(
            field, (-10, -10, 10, 10), [100.0], grid_size=10,
        )
        assert len(out[0].polygons) == 1
        # The polygon should approximately span the bbox
        verts = out[0].polygons[0]
        xs = [v[0] for v in verts]
        ys = [v[1] for v in verts]
        assert min(xs) <= -9 and max(xs) >= 9
        assert min(ys) <= -9 and max(ys) >= 9

    def test_point_plume_one_polygon_per_threshold(self) -> None:
        field = _PointPlume(peak=1000.0, falloff_per_m=10.0)
        out = extract_contours(
            field, (-50, -50, 50, 50), [100.0, 500.0, 900.0], grid_size=80,
        )
        assert len(out) == 3
        for level in out:
            assert len(level.polygons) == 1, (
                f"expected 1 polygon at threshold {level.threshold}, "
                f"got {len(level.polygons)}"
            )

    def test_higher_threshold_smaller_polygon(self) -> None:
        from shapely.geometry import Polygon
        field = _PointPlume(peak=1000.0, falloff_per_m=10.0)
        out = extract_contours(
            field, (-50, -50, 50, 50), [100.0, 500.0, 900.0], grid_size=80,
        )
        areas = [Polygon(level.polygons[0]).area for level in out]
        # Higher threshold (smaller hazard zone) → smaller polygon area
        assert areas[0] > areas[1] > areas[2]


# ---------- multi-component plumes ----------------------------------------

class TestMultiComponent:
    def test_two_disjoint_sources(self) -> None:
        class _TwoPeaks:
            def evaluate(self, x: float, y: float, t: float) -> float:
                a = max(0.0, 1000.0 - 10.0 * math.hypot(x - 30, y))
                b = max(0.0, 1000.0 - 10.0 * math.hypot(x + 30, y))
                return max(a, b)

        field = _TwoPeaks()
        out = extract_contours(
            field, (-60, -50, 60, 50), [800.0], grid_size=80,
        )
        assert len(out) == 1
        # At a high enough threshold the two plumes don't overlap → 2 polygons
        assert len(out[0].polygons) == 2


# ---------- threshold ordering preserved ----------------------------------

class TestOrdering:
    def test_thresholds_returned_in_input_order(self) -> None:
        field = _PointPlume()
        thresholds = [900.0, 100.0, 500.0]   # deliberately unsorted
        out = extract_contours(field, (-50, -50, 50, 50), thresholds,
                                grid_size=40)
        returned = [level.threshold for level in out]
        assert returned == thresholds


# ---------- gaussian plume integration ------------------------------------

class TestGaussianPlume:
    def test_gaussian_plume_yields_one_lobe_downwind(self) -> None:
        # Source at (0,0), wind blowing +x at 2 m/s
        field = GaussianPlumeField(
            source_x=0.0, source_y=0.0,
            release_rate=500.0, wind_speed=2.0, wind_direction=0.0,
            diffusion_coeff=0.05,
        )
        out = extract_contours(
            field, (-5, -10, 30, 10), [10.0], grid_size=80,
        )
        assert len(out) == 1
        assert len(out[0].polygons) == 1
        # Centroid of the contour should be downwind of the source
        verts = out[0].polygons[0]
        centroid_x = sum(v[0] for v in verts) / len(verts)
        assert centroid_x > 0  # downwind


# ---------- simplification --------------------------------------------------

class TestSimplification:
    def test_higher_tolerance_reduces_vertex_count(self) -> None:
        field = _PointPlume()
        # Two non-zero tolerances; bigger should produce fewer vertices.
        # (Passing 0.0 lets extract_contours pick its smart default.)
        verts_low = extract_contours(
            field, (-50, -50, 50, 50), [500.0],
            grid_size=80, simplify_tolerance=0.5,
        )[0].polygons[0]
        verts_high = extract_contours(
            field, (-50, -50, 50, 50), [500.0],
            grid_size=80, simplify_tolerance=5.0,
        )[0].polygons[0]
        assert len(verts_high) <= len(verts_low)
        # And must still be roughly the same shape — area within 30%
        from shapely.geometry import Polygon
        a_low = Polygon(verts_low).area
        a_high = Polygon(verts_high).area
        assert abs(a_high - a_low) / a_low < 0.3


class TestFitEllipse:
    def test_circle_fits_circle(self) -> None:
        # Vertices on a unit circle around origin
        n = 64
        verts = [
            (math.cos(2 * math.pi * i / n), math.sin(2 * math.pi * i / n))
            for i in range(n)
        ]
        result = fit_ellipse(verts)
        assert result is not None
        cx, cy, major, minor, _ = result
        assert abs(cx) < 0.05 and abs(cy) < 0.05
        assert abs(major - 2.0) < 0.05   # diameter
        assert abs(minor - 2.0) < 0.05

    def test_axis_aligned_ellipse(self) -> None:
        # Wide horizontal ellipse: a=10, b=2, axis-aligned
        n = 64
        verts = [
            (10 * math.cos(2 * math.pi * i / n),
              2 * math.sin(2 * math.pi * i / n))
            for i in range(n)
        ]
        result = fit_ellipse(verts)
        assert result is not None
        _, _, major, minor, angle = result
        assert abs(major - 20.0) < 0.5
        assert abs(minor - 4.0) < 0.5
        # Major axis along x → angle ~ 0 (or pi, equivalent for an axis)
        assert abs(angle) < 0.05 or abs(abs(angle) - math.pi) < 0.05

    def test_rotated_ellipse(self) -> None:
        # Same ellipse, rotated 45°
        n = 64
        a, b = 10.0, 2.0
        ang = math.pi / 4
        cos_a, sin_a = math.cos(ang), math.sin(ang)
        verts = []
        for i in range(n):
            t = 2 * math.pi * i / n
            x = a * math.cos(t)
            y = b * math.sin(t)
            verts.append((x * cos_a - y * sin_a, x * sin_a + y * cos_a))
        result = fit_ellipse(verts)
        assert result is not None
        _, _, major, minor, angle = result
        assert abs(major - 20.0) < 0.5
        assert abs(minor - 4.0) < 0.5
        # Recovered angle should match within 1° (or ±π for axis ambiguity)
        a_norm = abs(angle - ang)
        assert a_norm < 0.05 or abs(a_norm - math.pi) < 0.05

    def test_too_few_vertices_returns_none(self) -> None:
        assert fit_ellipse([]) is None
        assert fit_ellipse([(0, 0), (1, 1)]) is None
