"""AprilTag detection and tag classification.

Wraps the pupil-apriltags Detector with Plato Pod configuration.
Pure functions operating on numpy arrays — no ROS2 dependency.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

from plato_pod.config import AprilTagSettings, DetectorConfig, TagCategory, TagRanges

logger = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class RawDetection:
    """A single detected AprilTag with pixel-level data.

    This is a simplified representation of what pupil-apriltags returns,
    decoupled from the library's internal types.
    """
    tag_id: int
    center: tuple[float, float]        # pixel (cx, cy)
    corners: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ]                                    # 4 corner pixels, counter-clockwise
    decision_margin: float              # detection confidence (higher = better)
    hamming: int                        # bit error count (0 = perfect)
    # Pose data (filled by estimate_tag_pose, None if pose_estimation disabled)
    pose_R: np.ndarray | None = None    # 3x3 rotation matrix (camera frame)
    pose_t: np.ndarray | None = None    # 3x1 translation vector (camera frame, metres)

    @property
    def confidence(self) -> float:
        """Normalised confidence score 0.0-1.0 from decision margin.

        The decision margin from pupil-apriltags is typically 20-200+.
        We normalise: margin/200 clamped to [0, 1].
        """
        return min(max(self.decision_margin / 200.0, 0.0), 1.0)


def create_detector(config: DetectorConfig, family: str = "tag36h11"):
    """Create a pupil-apriltags Detector with the given config.

    Args:
        config: Detector parameters.
        family: AprilTag family string.

    Returns:
        A pupil_apriltags.Detector instance.
    """
    from pupil_apriltags import Detector

    return Detector(
        families=family,
        nthreads=config.nthreads,
        quad_decimate=config.quad_decimate,
        quad_sigma=0.0,
        decode_sharpening=config.decode_sharpening,
        refine_edges=config.refine_edges,
    )


def detect_tags(
    frame_gray: np.ndarray,
    detector,
    camera_params: tuple[float, float, float, float] | None = None,
    tag_sizes: dict[int, float] | None = None,
) -> list[RawDetection]:
    """Detect AprilTags in a grayscale frame.

    pupil-apriltags' detect() only accepts a single tag_size for pose estimation.
    Since we need per-tag sizes (robot=50mm, boundary=70mm), we call detect()
    once per unique size with pose estimation, then merge results by tag ID.

    Args:
        frame_gray: Grayscale uint8 image (H, W).
        detector: A pupil_apriltags.Detector instance.
        camera_params: (fx, fy, cx, cy) for pose estimation. If None, no pose.
        tag_sizes: Map of tag_id → physical size in metres for pose estimation.
            If a tag_id is not in the map, it is skipped for pose estimation.

    Returns:
        List of RawDetection objects.
    """
    estimate_pose = camera_params is not None and tag_sizes is not None

    if not estimate_pose:
        # Simple path: detect without pose estimation
        raw_detections = detector.detect(frame_gray, estimate_tag_pose=False)
        return [_convert_detection(det) for det in raw_detections]

    # Detect once WITHOUT pose estimation (fast — only image processing)
    raw_detections = detector.detect(frame_gray, estimate_tag_pose=False)

    # Compute pose per-tag using solvePnP (much faster than re-running detect)
    results = []
    for det in raw_detections:
        corners = tuple(tuple(float(v) for v in c) for c in det.corners)
        center = (float(det.center[0]), float(det.center[1]))

        pose_R = None
        pose_t = None

        if det.tag_id in tag_sizes:
            tag_size = tag_sizes[det.tag_id]
            pose_data = _estimate_pose_solvepnp(
                det.corners, camera_params, tag_size
            )
            if pose_data is not None:
                pose_R, pose_t = pose_data

        results.append(RawDetection(
            tag_id=det.tag_id,
            center=center,
            corners=corners,
            decision_margin=float(det.decision_margin),
            hamming=int(det.hamming),
            pose_R=pose_R,
            pose_t=pose_t,
        ))

    return results


def _estimate_pose_solvepnp(
    corners,
    camera_params: tuple[float, float, float, float],
    tag_size: float,
) -> tuple[np.ndarray, np.ndarray] | None:
    """Estimate tag pose using OpenCV solvePnP.

    Much faster than re-running the full AprilTag detector with pose estimation.

    Args:
        corners: 4 detected corner pixel positions from the detector.
        camera_params: (fx, fy, cx, cy).
        tag_size: Physical tag size in metres.

    Returns:
        (R, t) where R is 3x3 rotation and t is 3-element translation, or None.
    """
    try:
        import cv2
    except ImportError:
        return None

    fx, fy, cx, cy = camera_params
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1],
    ], dtype=np.float64)

    # Tag corners in object frame (tag-centered, Z=0 plane)
    half = tag_size / 2.0
    object_points = np.array([
        [-half, -half, 0],
        [half, -half, 0],
        [half, half, 0],
        [-half, half, 0],
    ], dtype=np.float64)

    image_points = np.array(corners, dtype=np.float64).reshape(4, 2)

    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, None,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )

    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.flatten()
    return R, t


def _convert_detection(det) -> RawDetection:
    """Convert a pupil-apriltags Detection to a RawDetection."""
    corners = tuple(tuple(float(v) for v in c) for c in det.corners)
    center = (float(det.center[0]), float(det.center[1]))
    pose_R = None
    pose_t = None
    if det.pose_R is not None:
        pose_R = np.array(det.pose_R, dtype=np.float64)
        pose_t = np.array(det.pose_t, dtype=np.float64).flatten()
    return RawDetection(
        tag_id=det.tag_id,
        center=center,
        corners=corners,
        decision_margin=float(det.decision_margin),
        hamming=int(det.hamming),
        pose_R=pose_R,
        pose_t=pose_t,
    )


def build_tag_size_map(settings: AprilTagSettings) -> dict[int, float]:
    """Build a tag_id → size (metres) mapping for all expected tags.

    Args:
        settings: AprilTag configuration.

    Returns:
        Dictionary mapping each known tag ID to its physical size.
    """
    sizes: dict[int, float] = {}
    ranges = settings.tag_ranges

    # Robot tags
    for tag_id in range(ranges.robots_min, ranges.robots_max + 1):
        sizes[tag_id] = settings.tag_sizes.default

    # Boundary tags (including origin)
    for tag_id in range(ranges.boundary_min, ranges.boundary_max + 1):
        sizes[tag_id] = settings.tag_sizes.boundary

    return sizes


def classify_detections(
    detections: list[RawDetection],
    tag_ranges: TagRanges,
) -> tuple[list[RawDetection], list[RawDetection], list[RawDetection]]:
    """Split detections into robot, boundary, and unknown lists.

    Args:
        detections: All detected tags.
        tag_ranges: Tag ID range configuration.

    Returns:
        Tuple of (robot_detections, boundary_detections, unknown_detections).
    """
    robots = []
    boundary = []
    unknown = []

    for det in detections:
        category = tag_ranges.classify(det.tag_id)
        if category in (TagCategory.ROBOT,):
            robots.append(det)
        elif category in (TagCategory.BOUNDARY, TagCategory.ORIGIN):
            boundary.append(det)
        else:
            unknown.append(det)

    return robots, boundary, unknown


def frame_to_gray(frame_bgr: np.ndarray) -> np.ndarray:
    """Convert a BGR frame to grayscale for detection.

    Uses OpenCV's optimized C implementation when available, falling back
    to numpy for environments without OpenCV.

    Args:
        frame_bgr: BGR uint8 image (H, W, 3).

    Returns:
        Grayscale uint8 image (H, W).
    """
    if len(frame_bgr.shape) == 2:
        return frame_bgr

    try:
        import cv2
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    except ImportError:
        return np.ascontiguousarray(
            np.dot(frame_bgr[..., :3], [0.114, 0.587, 0.299]).astype(np.uint8)
        )
