"""Tests for plato_pod.calibration — calibration state and math utilities."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from plato_pod.calibration import (
    CalibrationState,
    add_frame,
    generate_object_points,
    save_calibration,
)
from plato_pod.config import load_camera_calibration


# --- generate_object_points ---

class TestGenerateObjectPoints:
    def test_shape_9x6(self) -> None:
        pts = generate_object_points(9, 6, 0.025)
        assert pts.shape == (54, 3)

    def test_all_z_zero(self) -> None:
        pts = generate_object_points(9, 6, 0.025)
        np.testing.assert_array_equal(pts[:, 2], 0.0)

    def test_first_point_at_origin(self) -> None:
        pts = generate_object_points(9, 6, 0.025)
        np.testing.assert_array_almost_equal(pts[0], [0.0, 0.0, 0.0])

    def test_spacing_matches_square_size(self) -> None:
        pts = generate_object_points(3, 2, 0.05)
        # Points should be at 0, 0.05, 0.1 along X
        assert pts.shape == (6, 3)
        # Second point along X
        assert pts[1, 0] == pytest.approx(0.05)

    def test_custom_size(self) -> None:
        pts = generate_object_points(4, 3, 0.1)
        assert pts.shape == (12, 3)
        max_x = pts[:, 0].max()
        max_y = pts[:, 1].max()
        assert max_x == pytest.approx(0.3)  # (4-1) * 0.1
        assert max_y == pytest.approx(0.2)  # (3-1) * 0.1


# --- CalibrationState ---

class TestCalibrationState:
    def test_initial_state(self) -> None:
        state = CalibrationState()
        assert state.frames_captured == 0
        assert state.is_complete is False
        assert state.image_size is None

    def test_add_frame_increments_count(self) -> None:
        state = CalibrationState(squares_x=3, squares_y=2, target_frames=5)
        corners = np.zeros((6, 1, 2), dtype=np.float32)
        add_frame(state, corners, (640, 480))
        assert state.frames_captured == 1
        assert state.image_size == (640, 480)

    def test_is_complete_after_target_frames(self) -> None:
        state = CalibrationState(squares_x=3, squares_y=2, target_frames=2)
        corners = np.zeros((6, 1, 2), dtype=np.float32)
        add_frame(state, corners, (640, 480))
        assert state.is_complete is False
        add_frame(state, corners, (640, 480))
        assert state.is_complete is True


# --- save_calibration round-trip ---

class TestSaveCalibration:
    def test_round_trip(self, tmp_path: Path) -> None:
        out_path = tmp_path / "test_cal.yaml"
        camera_matrix = np.array([
            [800.0, 0.0, 320.0],
            [0.0, 800.0, 240.0],
            [0.0, 0.0, 1.0],
        ])
        distortion = np.array([0.01, -0.02, 0.001, -0.001, 0.005])

        save_calibration(out_path, camera_matrix, distortion, 640, 480, 0.35)

        # Load it back
        cal = load_camera_calibration(out_path)
        assert cal.image_width == 640
        assert cal.image_height == 480
        assert cal.fx == pytest.approx(800.0)
        assert cal.fy == pytest.approx(800.0)
        assert cal.cx == pytest.approx(320.0)
        assert cal.cy == pytest.approx(240.0)
        assert cal.reprojection_error == pytest.approx(0.35)
        assert len(cal.distortion_coefficients) == 5

    def test_creates_parent_directory(self, tmp_path: Path) -> None:
        out_path = tmp_path / "nested" / "dir" / "cal.yaml"
        camera_matrix = np.eye(3)
        distortion = np.zeros(5)
        save_calibration(out_path, camera_matrix, distortion, 640, 480, 0.5)
        assert out_path.exists()
