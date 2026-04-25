"""Configuration loading for the Plato Pod vision pipeline.

Loads apriltag_settings.yaml and camera_calibration.yaml into typed dataclasses.
No ROS2 dependency — this module is used by both the ROS2 node and unit tests.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path

import numpy as np
import yaml


class TagCategory(Enum):
    """Classification of an AprilTag by its ID range."""
    ROBOT = auto()
    BOUNDARY = auto()
    ORIGIN = auto()
    UNKNOWN = auto()


@dataclass(frozen=True, slots=True)
class TagRanges:
    """Tag ID range configuration."""
    robots_min: int
    robots_max: int
    boundary_min: int
    boundary_max: int
    origin_id: int

    def classify(self, tag_id: int) -> TagCategory:
        """Classify a tag ID into its category."""
        if tag_id == self.origin_id:
            return TagCategory.ORIGIN
        if self.boundary_min <= tag_id <= self.boundary_max:
            return TagCategory.BOUNDARY
        if self.robots_min <= tag_id <= self.robots_max:
            return TagCategory.ROBOT
        return TagCategory.UNKNOWN


@dataclass(frozen=True, slots=True)
class DetectorConfig:
    """AprilTag detector parameters."""
    nthreads: int = 4
    quad_decimate: float = 2.0
    refine_edges: bool = True
    decode_sharpening: float = 0.25


@dataclass(frozen=True, slots=True)
class TagSizes:
    """Physical tag sizes in metres."""
    default: float = 0.050
    boundary: float = 0.070

    def get_size(self, tag_id: int, tag_ranges: TagRanges) -> float:
        """Return the physical size for a given tag ID."""
        category = tag_ranges.classify(tag_id)
        if category in (TagCategory.BOUNDARY, TagCategory.ORIGIN):
            return self.boundary
        return self.default


@dataclass(frozen=True, slots=True)
class AprilTagSettings:
    """Complete AprilTag configuration."""
    family: str = "tag36h11"
    tag_sizes: TagSizes = field(default_factory=TagSizes)
    tag_ranges: TagRanges = field(default_factory=lambda: TagRanges(
        robots_min=0, robots_max=15,
        boundary_min=100, boundary_max=103,
        origin_id=101,
    ))
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    detection_fps: int = 30
    pose_estimation: bool = True


@dataclass(frozen=True, slots=True)
class CameraCalibration:
    """Camera intrinsic parameters from calibration."""
    image_width: int
    image_height: int
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    calibration_date: str = ""
    reprojection_error: float = 0.0

    class Config:
        arbitrary_types_allowed = True

    @property
    def fx(self) -> float:
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self) -> float:
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self) -> float:
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self) -> float:
        return float(self.camera_matrix[1, 2])

    @property
    def camera_params(self) -> tuple[float, float, float, float]:
        """Return (fx, fy, cx, cy) tuple for pupil-apriltags."""
        return (self.fx, self.fy, self.cx, self.cy)


def load_apriltag_settings(path: str | Path) -> AprilTagSettings:
    """Load AprilTag settings from a YAML file.

    Args:
        path: Path to apriltag_settings.yaml.

    Returns:
        Parsed AprilTagSettings dataclass.

    Raises:
        FileNotFoundError: If the file does not exist.
        KeyError: If required fields are missing.
    """
    path = Path(path)
    with open(path) as f:
        raw = yaml.safe_load(f)

    cfg = raw["apriltag_settings"]

    tag_ranges_raw = cfg["tag_ranges"]
    tag_ranges = TagRanges(
        robots_min=tag_ranges_raw["robots"][0],
        robots_max=tag_ranges_raw["robots"][1],
        boundary_min=tag_ranges_raw["boundary"][0],
        boundary_max=tag_ranges_raw["boundary"][1],
        origin_id=tag_ranges_raw["origin"],
    )

    tag_sizes_raw = cfg["tag_sizes"]
    tag_sizes = TagSizes(
        default=tag_sizes_raw["default"],
        boundary=tag_sizes_raw["boundary"],
    )

    det_raw = cfg["detector"]
    detector = DetectorConfig(
        nthreads=det_raw["nthreads"],
        quad_decimate=float(det_raw["quad_decimate"]),
        refine_edges=det_raw["refine_edges"],
        decode_sharpening=float(det_raw["decode_sharpening"]),
    )

    return AprilTagSettings(
        family=cfg["family"],
        tag_sizes=tag_sizes,
        tag_ranges=tag_ranges,
        detector=detector,
        detection_fps=cfg["detection_fps"],
        pose_estimation=cfg["pose_estimation"],
    )


def load_camera_calibration(path: str | Path) -> CameraCalibration:
    """Load camera calibration from a YAML file.

    Args:
        path: Path to camera_calibration.yaml.

    Returns:
        Parsed CameraCalibration dataclass.

    Raises:
        FileNotFoundError: If the file does not exist.
        KeyError: If required fields are missing.
    """
    path = Path(path)
    with open(path) as f:
        raw = yaml.safe_load(f)

    cfg = raw["camera_calibration"]

    camera_matrix = np.array(cfg["camera_matrix"], dtype=np.float64)
    distortion = np.array(cfg["distortion_coefficients"], dtype=np.float64)

    return CameraCalibration(
        image_width=cfg["image_width"],
        image_height=cfg["image_height"],
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion,
        calibration_date=cfg.get("calibration_date", ""),
        reprojection_error=cfg.get("reprojection_error", 0.0),
    )


def default_calibration(width: int = 1920, height: int = 1080,
                        hfov_deg: float = 100.0) -> CameraCalibration:
    """Generate a default pinhole camera model from resolution and FOV.

    Used when no calibration file exists. Provides approximate poses but may
    have errors of several centimetres at the frame edges.

    Args:
        width: Image width in pixels.
        height: Image height in pixels.
        hfov_deg: Horizontal field of view in degrees.

    Returns:
        CameraCalibration with estimated intrinsics and zero distortion.
    """
    hfov_rad = np.deg2rad(hfov_deg)
    fx = (width / 2.0) / np.tan(hfov_rad / 2.0)
    fy = fx  # assume square pixels
    cx = width / 2.0
    cy = height / 2.0

    camera_matrix = np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    distortion = np.zeros(5, dtype=np.float64)

    return CameraCalibration(
        image_width=width,
        image_height=height,
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion,
        calibration_date="default",
        reprojection_error=-1.0,
    )
