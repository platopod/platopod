"""Tests for plato_pod.homography — homography computation and application."""

from __future__ import annotations

import numpy as np
import pytest

from plato_pod.homography import (
    apply_homography,
    apply_homography_batch,
    compute_homography,
)


def _simple_correspondences() -> tuple[np.ndarray, np.ndarray]:
    """4-point correspondence: pixel rectangle → arena rectangle.

    Maps a 1000x1000 pixel region to a 2m x 1.5m arena.
    """
    pixel_pts = np.array([
        [0.0, 0.0],       # top-left
        [1000.0, 0.0],    # top-right
        [1000.0, 1000.0], # bottom-right
        [0.0, 1000.0],    # bottom-left
    ], dtype=np.float64)

    arena_pts = np.array([
        [0.0, 0.0],    # top-left
        [2.0, 0.0],    # top-right
        [2.0, 1.5],    # bottom-right
        [0.0, 1.5],    # bottom-left
    ], dtype=np.float64)

    return pixel_pts, arena_pts


# --- compute_homography ---

class TestComputeHomography:
    def test_simple_rectangle_mapping(self) -> None:
        pixel_pts, arena_pts = _simple_correspondences()
        H = compute_homography(pixel_pts, arena_pts)
        assert H is not None
        assert H.shape == (3, 3)
        assert H[2, 2] == pytest.approx(1.0)

    def test_maps_corner_points_correctly(self) -> None:
        pixel_pts, arena_pts = _simple_correspondences()
        H = compute_homography(pixel_pts, arena_pts)
        assert H is not None

        for px, ax in zip(pixel_pts, arena_pts):
            result = apply_homography(H, (px[0], px[1]))
            assert result[0] == pytest.approx(ax[0], abs=1e-6)
            assert result[1] == pytest.approx(ax[1], abs=1e-6)

    def test_maps_center_point_correctly(self) -> None:
        pixel_pts, arena_pts = _simple_correspondences()
        H = compute_homography(pixel_pts, arena_pts)
        assert H is not None

        # Center of pixel rectangle should map to center of arena
        cx, cy = apply_homography(H, (500.0, 500.0))
        assert cx == pytest.approx(1.0, abs=1e-6)
        assert cy == pytest.approx(0.75, abs=1e-6)

    def test_returns_none_with_too_few_points(self) -> None:
        pixel_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
        arena_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
        assert compute_homography(pixel_pts, arena_pts) is None

    def test_returns_none_with_mismatched_counts(self) -> None:
        pixel_pts = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float64)
        arena_pts = np.array([[0, 0], [1, 0], [1, 1]], dtype=np.float64)
        assert compute_homography(pixel_pts, arena_pts) is None

    def test_identity_like_mapping(self) -> None:
        """Same coordinates in both spaces should give ~identity homography."""
        pts = np.array([
            [0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0],
        ], dtype=np.float64)
        H = compute_homography(pts, pts)
        assert H is not None
        # H should be close to identity (up to scale)
        H_norm = H / H[2, 2]
        np.testing.assert_array_almost_equal(H_norm, np.eye(3), decimal=5)


# --- apply_homography ---

class TestApplyHomography:
    def test_identity_homography(self) -> None:
        H = np.eye(3, dtype=np.float64)
        result = apply_homography(H, (100.0, 200.0))
        assert result[0] == pytest.approx(100.0)
        assert result[1] == pytest.approx(200.0)

    def test_scale_homography(self) -> None:
        H = np.diag([2.0, 3.0, 1.0])
        result = apply_homography(H, (10.0, 20.0))
        assert result[0] == pytest.approx(20.0)
        assert result[1] == pytest.approx(60.0)


# --- apply_homography_batch ---

class TestApplyHomographyBatch:
    def test_matches_single_point_results(self) -> None:
        pixel_pts, arena_pts = _simple_correspondences()
        H = compute_homography(pixel_pts, arena_pts)
        assert H is not None

        batch_result = apply_homography_batch(H, pixel_pts)
        assert batch_result.shape == (4, 2)

        for i in range(4):
            single = apply_homography(H, (pixel_pts[i, 0], pixel_pts[i, 1]))
            assert batch_result[i, 0] == pytest.approx(single[0], abs=1e-10)
            assert batch_result[i, 1] == pytest.approx(single[1], abs=1e-10)

    def test_identity_batch(self) -> None:
        H = np.eye(3, dtype=np.float64)
        pts = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float64)
        result = apply_homography_batch(H, pts)
        np.testing.assert_array_almost_equal(result, pts)
