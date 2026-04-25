"""Homography computation between camera pixels and arena coordinates.

Computes a 3x3 homography matrix from boundary corner tag correspondences
(pixel positions ↔ arena positions in metres). No ROS2 dependency.
"""

from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger(__name__)

# Minimum number of point correspondences for homography
_MIN_POINTS = 4


def compute_homography(
    pixel_points: np.ndarray,
    arena_points: np.ndarray,
) -> np.ndarray | None:
    """Compute the homography mapping pixel coordinates to arena coordinates.

    Uses the Direct Linear Transform (DLT) algorithm. Requires at least 4
    point correspondences (the 4 boundary corner tags).

    Args:
        pixel_points: Nx2 array of pixel coordinates (x, y).
        arena_points: Nx2 array of arena coordinates in metres (x, y).

    Returns:
        3x3 homography matrix, or None if computation fails.
    """
    if len(pixel_points) < _MIN_POINTS or len(arena_points) < _MIN_POINTS:
        logger.warning(
            "Need at least %d point pairs for homography, got %d",
            _MIN_POINTS, min(len(pixel_points), len(arena_points)),
        )
        return None

    if len(pixel_points) != len(arena_points):
        logger.error(
            "Mismatched point counts: %d pixel vs %d arena",
            len(pixel_points), len(arena_points),
        )
        return None

    # Build the DLT system: for each correspondence (x,y) -> (x',y'),
    # we get two equations per point.
    n = len(pixel_points)
    A = np.zeros((2 * n, 9), dtype=np.float64)

    for i in range(n):
        px, py = pixel_points[i]
        ax, ay = arena_points[i]
        A[2 * i] = [-px, -py, -1, 0, 0, 0, ax * px, ax * py, ax]
        A[2 * i + 1] = [0, 0, 0, -px, -py, -1, ay * px, ay * py, ay]

    # Solve using SVD: h is the last row of V^T (smallest singular value)
    try:
        _, _, Vt = np.linalg.svd(A)
        h = Vt[-1]
        H = h.reshape(3, 3)

        # Normalise so H[2,2] = 1 (standard convention)
        if abs(H[2, 2]) < 1e-10:
            logger.warning("Degenerate homography: H[2,2] near zero")
            return None
        H = H / H[2, 2]

        return H
    except np.linalg.LinAlgError:
        logger.error("SVD failed during homography computation")
        return None


def apply_homography(
    H: np.ndarray,
    pixel_point: tuple[float, float],
) -> tuple[float, float]:
    """Map a single pixel coordinate to arena coordinate using the homography.

    Args:
        H: 3x3 homography matrix (pixel → arena).
        pixel_point: (x, y) pixel coordinate.

    Returns:
        (x, y) arena coordinate in metres.
    """
    p = np.array([pixel_point[0], pixel_point[1], 1.0], dtype=np.float64)
    result = H @ p
    # Dehomogenise
    w = result[2]
    if abs(w) < 1e-10:
        logger.warning("Homography projection at infinity for point %s", pixel_point)
        return (0.0, 0.0)
    return (float(result[0] / w), float(result[1] / w))


def apply_homography_batch(
    H: np.ndarray,
    pixel_points: np.ndarray,
) -> np.ndarray:
    """Map multiple pixel coordinates to arena coordinates.

    Args:
        H: 3x3 homography matrix (pixel → arena).
        pixel_points: Nx2 array of pixel coordinates.

    Returns:
        Nx2 array of arena coordinates in metres.
    """
    n = len(pixel_points)
    # Convert to homogeneous coordinates
    ones = np.ones((n, 1), dtype=np.float64)
    pts_h = np.hstack([pixel_points, ones])  # Nx3

    # Apply homography
    result = (H @ pts_h.T).T  # Nx3

    # Dehomogenise
    w = result[:, 2:3]
    w = np.where(np.abs(w) < 1e-10, 1.0, w)
    return result[:, :2] / w
