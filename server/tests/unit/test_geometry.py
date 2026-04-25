"""Tests for plato_pod.geometry — computational geometry utilities."""

from __future__ import annotations

import math

import pytest

from plato_pod.geometry import (
    circle_to_polygon,
    convex_hull,
    point_in_polygon,
    polygon_area,
    polygon_contains_polygon,
    rectangle_to_polygon,
    to_shapely_polygon,
)


# --- convex_hull ---

class TestConvexHull:
    def test_square_points(self) -> None:
        pts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        hull = convex_hull(pts)
        assert len(hull) == 4

    def test_with_interior_point(self) -> None:
        pts = [(0, 0), (2, 0), (2, 2), (0, 2), (1, 1)]
        hull = convex_hull(pts)
        assert len(hull) == 4  # interior point excluded

    def test_triangle(self) -> None:
        pts = [(0, 0), (1, 0), (0.5, 1)]
        hull = convex_hull(pts)
        assert len(hull) == 3

    def test_fewer_than_3_points(self) -> None:
        assert convex_hull([(1, 2)]) == [(1, 2)]
        assert convex_hull([(0, 0), (1, 1)]) == [(0, 0), (1, 1)]

    def test_collinear_points(self) -> None:
        pts = [(0, 0), (1, 0), (2, 0)]
        hull = convex_hull(pts)
        # Collinear points form a line, not a polygon
        assert len(hull) <= 3

    def test_hull_area_matches_known_square(self) -> None:
        pts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        hull = convex_hull(pts)
        area = abs(polygon_area(hull))
        assert area == pytest.approx(1.0)

    def test_duplicate_points(self) -> None:
        pts = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0), (1, 0)]
        hull = convex_hull(pts)
        assert len(hull) == 4


# --- point_in_polygon ---

class TestPointInPolygon:
    def test_inside_square(self) -> None:
        square = [(0, 0), (1, 0), (1, 1), (0, 1)]
        assert point_in_polygon((0.5, 0.5), square) is True

    def test_outside_square(self) -> None:
        square = [(0, 0), (1, 0), (1, 1), (0, 1)]
        assert point_in_polygon((2, 2), square) is False

    def test_on_edge(self) -> None:
        square = [(0, 0), (1, 0), (1, 1), (0, 1)]
        assert point_in_polygon((0.5, 0), square) is True

    def test_on_vertex(self) -> None:
        square = [(0, 0), (1, 0), (1, 1), (0, 1)]
        assert point_in_polygon((0, 0), square) is True

    def test_concave_polygon(self) -> None:
        # L-shaped polygon
        poly = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
        assert point_in_polygon((0.5, 0.5), poly) is True
        assert point_in_polygon((1.5, 1.5), poly) is False

    def test_degenerate_polygon(self) -> None:
        assert point_in_polygon((0, 0), [(0, 0), (1, 0)]) is False


# --- polygon_contains_polygon ---

class TestPolygonContainsPolygon:
    def test_fully_inside(self) -> None:
        outer = [(0, 0), (10, 0), (10, 10), (0, 10)]
        inner = [(2, 2), (4, 2), (4, 4), (2, 4)]
        all_inside, outside = polygon_contains_polygon(outer, inner)
        assert all_inside is True
        assert outside == []

    def test_partially_outside(self) -> None:
        outer = [(0, 0), (10, 0), (10, 10), (0, 10)]
        inner = [(2, 2), (12, 2), (12, 8), (2, 8)]
        all_inside, outside = polygon_contains_polygon(outer, inner)
        assert all_inside is False
        assert len(outside) == 2  # vertices at x=12 are outside

    def test_fully_outside(self) -> None:
        outer = [(0, 0), (1, 0), (1, 1), (0, 1)]
        inner = [(5, 5), (6, 5), (6, 6), (5, 6)]
        all_inside, outside = polygon_contains_polygon(outer, inner)
        assert all_inside is False
        assert len(outside) == 4

    def test_on_boundary_counts_as_inside(self) -> None:
        outer = [(0, 0), (1, 0), (1, 1), (0, 1)]
        inner = [(0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5)]
        all_inside, _ = polygon_contains_polygon(outer, inner)
        assert all_inside is True

    def test_empty_outer(self) -> None:
        all_inside, outside = polygon_contains_polygon([], [(1, 1)])
        assert all_inside is False


# --- rectangle_to_polygon ---

class TestRectangleToPolygon:
    def test_axis_aligned(self) -> None:
        verts = rectangle_to_polygon(1.0, 2.0, 0.4, 0.2)
        assert len(verts) == 4
        xs = [v[0] for v in verts]
        ys = [v[1] for v in verts]
        assert min(xs) == pytest.approx(0.8)
        assert max(xs) == pytest.approx(1.2)
        assert min(ys) == pytest.approx(1.9)
        assert max(ys) == pytest.approx(2.1)

    def test_area(self) -> None:
        verts = rectangle_to_polygon(0, 0, 2.0, 3.0)
        area = abs(polygon_area(verts))
        assert area == pytest.approx(6.0)

    def test_rotation_90_degrees(self) -> None:
        verts = rectangle_to_polygon(0, 0, 2.0, 1.0, rotation=math.pi / 2)
        # After 90-degree rotation, width and height swap
        xs = [v[0] for v in verts]
        ys = [v[1] for v in verts]
        assert max(xs) - min(xs) == pytest.approx(1.0, abs=1e-10)
        assert max(ys) - min(ys) == pytest.approx(2.0, abs=1e-10)

    def test_rotation_preserves_area(self) -> None:
        verts = rectangle_to_polygon(1, 1, 0.5, 0.3, rotation=0.7)
        area = abs(polygon_area(verts))
        assert area == pytest.approx(0.15)

    def test_zero_rotation_same_as_default(self) -> None:
        v1 = rectangle_to_polygon(1, 2, 3, 4, rotation=0.0)
        v2 = rectangle_to_polygon(1, 2, 3, 4)
        for a, b in zip(v1, v2):
            assert a[0] == pytest.approx(b[0])
            assert a[1] == pytest.approx(b[1])


# --- circle_to_polygon ---

class TestCircleToPolygon:
    def test_vertex_count(self) -> None:
        verts = circle_to_polygon(0, 0, 1.0, n_vertices=16)
        assert len(verts) == 16

    def test_custom_vertex_count(self) -> None:
        verts = circle_to_polygon(0, 0, 1.0, n_vertices=32)
        assert len(verts) == 32

    def test_area_approaches_pi_r_squared(self) -> None:
        verts = circle_to_polygon(0, 0, 1.0, n_vertices=64)
        area = abs(polygon_area(verts))
        assert area == pytest.approx(math.pi, abs=0.01)

    def test_center_inside(self) -> None:
        verts = circle_to_polygon(5.0, 3.0, 0.5, n_vertices=16)
        assert point_in_polygon((5.0, 3.0), verts) is True

    def test_point_outside(self) -> None:
        verts = circle_to_polygon(0, 0, 1.0, n_vertices=16)
        assert point_in_polygon((2.0, 0), verts) is False


# --- polygon_area ---

class TestPolygonArea:
    def test_unit_square_ccw(self) -> None:
        verts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        assert polygon_area(verts) == pytest.approx(1.0)

    def test_unit_square_cw(self) -> None:
        verts = [(0, 0), (0, 1), (1, 1), (1, 0)]
        assert polygon_area(verts) == pytest.approx(-1.0)

    def test_triangle(self) -> None:
        verts = [(0, 0), (4, 0), (0, 3)]
        assert abs(polygon_area(verts)) == pytest.approx(6.0)

    def test_degenerate(self) -> None:
        assert polygon_area([(0, 0), (1, 1)]) == 0.0
        assert polygon_area([]) == 0.0


# --- to_shapely_polygon ---

class TestToShapelyPolygon:
    def test_creates_valid_polygon(self) -> None:
        verts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        poly = to_shapely_polygon(verts)
        assert poly.is_valid
        assert poly.area == pytest.approx(1.0)

    def test_contains_interior_point(self) -> None:
        verts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        poly = to_shapely_polygon(verts)
        assert poly.contains(poly.centroid)
