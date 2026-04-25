"""Tests for plato_pod.config — YAML loading and dataclass construction."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from plato_pod.config import (
    TagCategory,
    TagRanges,
    TagSizes,
    default_calibration,
    load_apriltag_settings,
    load_camera_calibration,
)

DATA_DIR = Path(__file__).parent.parent / "data"
CONFIG_DIR = Path(__file__).parent.parent.parent / "config"


# --- TagRanges.classify ---

class TestTagRangesClassify:
    def test_classify_robot_tag_returns_robot(self, tag_ranges: TagRanges) -> None:
        assert tag_ranges.classify(0) == TagCategory.ROBOT
        assert tag_ranges.classify(7) == TagCategory.ROBOT
        assert tag_ranges.classify(15) == TagCategory.ROBOT

    def test_classify_boundary_tag_returns_boundary(self, tag_ranges: TagRanges) -> None:
        assert tag_ranges.classify(100) == TagCategory.BOUNDARY
        assert tag_ranges.classify(102) == TagCategory.BOUNDARY
        assert tag_ranges.classify(103) == TagCategory.BOUNDARY

    def test_classify_origin_tag_returns_origin(self, tag_ranges: TagRanges) -> None:
        assert tag_ranges.classify(101) == TagCategory.ORIGIN

    def test_classify_unknown_tag_returns_unknown(self, tag_ranges: TagRanges) -> None:
        assert tag_ranges.classify(50) == TagCategory.UNKNOWN
        assert tag_ranges.classify(200) == TagCategory.UNKNOWN
        assert tag_ranges.classify(-1) == TagCategory.UNKNOWN


# --- TagSizes.get_size ---

class TestTagSizesGetSize:
    def test_robot_tag_uses_default_size(
        self, tag_sizes: TagSizes, tag_ranges: TagRanges
    ) -> None:
        assert tag_sizes.get_size(5, tag_ranges) == 0.050

    def test_boundary_tag_uses_boundary_size(
        self, tag_sizes: TagSizes, tag_ranges: TagRanges
    ) -> None:
        assert tag_sizes.get_size(100, tag_ranges) == 0.070

    def test_origin_tag_uses_boundary_size(
        self, tag_sizes: TagSizes, tag_ranges: TagRanges
    ) -> None:
        assert tag_sizes.get_size(101, tag_ranges) == 0.070

    def test_unknown_tag_uses_default_size(
        self, tag_sizes: TagSizes, tag_ranges: TagRanges
    ) -> None:
        assert tag_sizes.get_size(999, tag_ranges) == 0.050


# --- load_apriltag_settings ---

class TestLoadAprilTagSettings:
    def test_loads_real_config_successfully(self) -> None:
        settings = load_apriltag_settings(CONFIG_DIR / "apriltag_settings.yaml")
        assert settings.family == "tag36h11"
        assert settings.tag_ranges.origin_id == 101
        assert settings.tag_sizes.default == 0.050
        assert settings.tag_sizes.boundary == 0.070
        assert settings.detector.nthreads == 4
        assert settings.detector.quad_decimate == 2.0
        assert settings.detection_fps == 30
        assert settings.pose_estimation is True

    def test_tag_ranges_parsed_correctly(self) -> None:
        settings = load_apriltag_settings(CONFIG_DIR / "apriltag_settings.yaml")
        assert settings.tag_ranges.robots_min == 0
        assert settings.tag_ranges.robots_max == 15
        assert settings.tag_ranges.boundary_min == 100
        assert settings.tag_ranges.boundary_max == 199

    def test_nonexistent_file_raises_error(self, tmp_path: Path) -> None:
        with pytest.raises(FileNotFoundError):
            load_apriltag_settings(tmp_path / "nonexistent.yaml")


# --- load_camera_calibration ---

class TestLoadCameraCalibration:
    def test_loads_test_calibration(self) -> None:
        cal = load_camera_calibration(DATA_DIR / "test_camera_calibration.yaml")
        assert cal.image_width == 1920
        assert cal.image_height == 1080
        assert cal.fx == 800.0
        assert cal.fy == 800.0
        assert cal.cx == 960.0
        assert cal.cy == 540.0
        assert cal.calibration_date == "2026-04-12"
        assert cal.reprojection_error == 0.25

    def test_camera_params_tuple(self) -> None:
        cal = load_camera_calibration(DATA_DIR / "test_camera_calibration.yaml")
        fx, fy, cx, cy = cal.camera_params
        assert fx == 800.0
        assert fy == 800.0
        assert cx == 960.0
        assert cy == 540.0

    def test_distortion_coefficients_shape(self) -> None:
        cal = load_camera_calibration(DATA_DIR / "test_camera_calibration.yaml")
        assert cal.distortion_coefficients.shape == (5,)

    def test_camera_matrix_shape(self) -> None:
        cal = load_camera_calibration(DATA_DIR / "test_camera_calibration.yaml")
        assert cal.camera_matrix.shape == (3, 3)


# --- default_calibration ---

class TestDefaultCalibration:
    def test_default_1080p_100deg(self) -> None:
        cal = default_calibration(1920, 1080, 100.0)
        assert cal.image_width == 1920
        assert cal.image_height == 1080
        assert cal.cx == 960.0
        assert cal.cy == 540.0
        assert cal.calibration_date == "default"
        assert cal.reprojection_error == -1.0

    def test_default_zero_distortion(self) -> None:
        cal = default_calibration()
        np.testing.assert_array_equal(cal.distortion_coefficients, np.zeros(5))

    def test_default_fx_fy_equal(self) -> None:
        cal = default_calibration()
        assert cal.fx == cal.fy

    def test_wider_fov_gives_shorter_focal_length(self) -> None:
        cal_narrow = default_calibration(1920, 1080, 60.0)
        cal_wide = default_calibration(1920, 1080, 120.0)
        assert cal_wide.fx < cal_narrow.fx
