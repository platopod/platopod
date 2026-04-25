"""Shared test fixtures for Plato Pod unit and integration tests.

No ROS2 imports here — unit tests must run without rclpy.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from plato_pod.config import (
    AprilTagSettings,
    CameraCalibration,
    TagRanges,
    TagSizes,
    load_apriltag_settings,
    load_camera_calibration,
)

DATA_DIR = Path(__file__).parent / "data"
SERVER_DIR = Path(__file__).parent.parent
CONFIG_DIR = SERVER_DIR / "config"


@pytest.fixture
def apriltag_settings() -> AprilTagSettings:
    """Load AprilTag settings from the real project config."""
    return load_apriltag_settings(CONFIG_DIR / "apriltag_settings.yaml")


@pytest.fixture
def tag_ranges() -> TagRanges:
    """Default tag ranges matching the project config."""
    return TagRanges(
        robots_min=0,
        robots_max=15,
        boundary_min=100,
        boundary_max=103,
        origin_id=101,
    )


@pytest.fixture
def tag_sizes() -> TagSizes:
    """Default tag sizes matching the project config."""
    return TagSizes(default=0.050, boundary=0.070)


@pytest.fixture
def sample_camera_calibration() -> CameraCalibration:
    """Load sample camera calibration from test data."""
    return load_camera_calibration(DATA_DIR / "test_camera_calibration.yaml")


@pytest.fixture
def default_camera_params() -> tuple[float, float, float, float]:
    """Camera params (fx, fy, cx, cy) for a 1920x1080 100-degree FOV camera."""
    hfov_rad = np.deg2rad(100.0)
    fx = (1920 / 2.0) / np.tan(hfov_rad / 2.0)
    fy = fx
    cx = 960.0
    cy = 540.0
    return (fx, fy, cx, cy)


@pytest.fixture
def identity_transform() -> np.ndarray:
    """4x4 identity transform matrix."""
    return np.eye(4, dtype=np.float64)


# --- Arena model fixtures ---


@pytest.fixture
def sample_boundary() -> list[tuple[float, float]]:
    """A 0.84m x 0.59m rectangle boundary (typical arena)."""
    return [(0.0, 0.0), (0.84, 0.0), (0.84, 0.59), (0.0, 0.59)]




# --- Robot registry fixtures ---


@pytest.fixture
def unit_boundary() -> tuple[tuple[float, float], ...]:
    """A 1m x 1m boundary for spawn validation tests."""
    return ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
