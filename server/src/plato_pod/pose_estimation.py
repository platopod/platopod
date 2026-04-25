"""Pose estimation and coordinate transforms for the Plato Pod vision pipeline.

Transforms detected AprilTag poses from camera frame to arena frame using
Tag 101 as the coordinate origin. All functions are pure — no ROS2 dependency.

Coordinate convention:
- Camera frame: origin at camera, Z forward, X right, Y down
- Arena frame: origin at Tag 101, X right, Y forward (away from Tag 101's
  bottom edge), Z up. All positions in metres.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np

from plato_pod.detection import RawDetection

logger = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class ArenaPose:
    """A tag's pose in the arena coordinate frame."""
    tag_id: int
    x: float            # metres from Tag 101
    y: float            # metres from Tag 101
    theta: float        # heading in radians [-pi, pi]
    confidence: float   # 0.0-1.0
    timestamp: float    # seconds


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from rotation and translation.

    Args:
        R: 3x3 rotation matrix.
        t: 3-element translation vector.

    Returns:
        4x4 homogeneous transform matrix.
    """
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Invert a 4x4 homogeneous transform efficiently.

    For a rigid transform [R|t; 0 1], the inverse is [R^T | -R^T t; 0 1].

    Args:
        T: 4x4 homogeneous transform.

    Returns:
        4x4 inverted transform.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def compute_camera_to_arena_transform(
    origin_detection: RawDetection,
) -> np.ndarray | None:
    """Compute the transform from camera frame to arena frame using Tag 101.

    The arena frame has its origin at Tag 101, with the tag's local X axis
    as arena X, and the tag's local Y axis as arena Y.

    Args:
        origin_detection: Detection of Tag 101 with pose_R and pose_t set.

    Returns:
        4x4 transform matrix (camera → arena), or None if pose data is missing.
    """
    if origin_detection.pose_R is None or origin_detection.pose_t is None:
        logger.warning("Origin tag %d has no pose data", origin_detection.tag_id)
        return None

    # T_camera_to_tag101: transform from camera frame to Tag 101's frame
    # The detector gives us the tag's pose in camera frame:
    #   pose_R rotates from tag frame to camera frame
    #   pose_t is the tag's position in camera frame
    # So T_cam_to_tag = [R|t] maps tag-frame points to camera-frame points.
    # We want the inverse: camera-frame → tag-frame (arena frame).
    T_cam_to_tag = make_transform(origin_detection.pose_R, origin_detection.pose_t)
    T_tag_to_cam = invert_transform(T_cam_to_tag)

    # The arena frame is the tag frame projected onto the tag plane.
    # For an overhead camera looking down, the tag's Z axis points up (toward camera).
    # We want arena X = tag X, arena Y = tag Y, and we ignore Z (height).
    return T_tag_to_cam


def transform_detection_to_arena(
    detection: RawDetection,
    T_camera_to_arena: np.ndarray,
    timestamp: float = 0.0,
) -> ArenaPose:
    """Transform a single detection from camera frame to arena frame.

    Args:
        detection: A RawDetection with pose_R and pose_t set.
        T_camera_to_arena: 4x4 transform from camera frame to arena frame.
        timestamp: Timestamp to attach to the pose.

    Returns:
        ArenaPose with (x, y, theta) in the arena frame.

    Raises:
        ValueError: If the detection has no pose data.
    """
    if detection.pose_R is None or detection.pose_t is None:
        raise ValueError(f"Tag {detection.tag_id} has no pose data for transform")

    # Tag position in camera frame
    T_cam_to_tag = make_transform(detection.pose_R, detection.pose_t)

    # Tag position in arena frame
    T_arena_to_tag = T_camera_to_arena @ T_cam_to_tag

    # Extract 2D position (x, y) from the transform
    x = float(T_arena_to_tag[0, 3])
    y = float(T_arena_to_tag[1, 3])

    # Extract heading angle from rotation matrix
    # theta = atan2(R[1,0], R[0,0]) gives the angle of the tag's X-axis
    # relative to arena X-axis, projected onto the XY plane
    R_arena = T_arena_to_tag[:3, :3]
    theta = math.atan2(float(R_arena[1, 0]), float(R_arena[0, 0]))

    return ArenaPose(
        tag_id=detection.tag_id,
        x=x,
        y=y,
        theta=theta,
        confidence=detection.confidence,
        timestamp=timestamp,
    )


def transform_detections_to_arena(
    detections: list[RawDetection],
    T_camera_to_arena: np.ndarray,
    timestamp: float = 0.0,
) -> list[ArenaPose]:
    """Transform a batch of detections from camera frame to arena frame.

    Detections without pose data are skipped with a warning.

    Args:
        detections: List of RawDetections with pose data.
        T_camera_to_arena: 4x4 transform from camera frame to arena frame.
        timestamp: Timestamp to attach to all poses.

    Returns:
        List of ArenaPose objects.
    """
    poses = []
    for det in detections:
        if det.pose_R is None or det.pose_t is None:
            logger.debug("Skipping tag %d: no pose data", det.tag_id)
            continue
        poses.append(transform_detection_to_arena(det, T_camera_to_arena, timestamp))
    return poses


def extract_2d_pose(T: np.ndarray) -> tuple[float, float, float]:
    """Extract (x, y, theta) from a 4x4 homogeneous transform.

    Args:
        T: 4x4 homogeneous transform.

    Returns:
        Tuple (x, y, theta) where theta is in radians [-pi, pi].
    """
    x = float(T[0, 3])
    y = float(T[1, 3])
    theta = math.atan2(float(T[1, 0]), float(T[0, 0]))
    return x, y, theta
