"""Tests for plato_pod.pose_estimation — coordinate transforms and pose extraction."""

from __future__ import annotations

import math

import numpy as np
import pytest

from plato_pod.detection import RawDetection
from plato_pod.pose_estimation import (
    ArenaPose,
    compute_camera_to_arena_transform,
    extract_2d_pose,
    invert_transform,
    make_transform,
    transform_detection_to_arena,
    transform_detections_to_arena,
)


def _make_detection_with_pose(
    tag_id: int,
    R: np.ndarray,
    t: np.ndarray,
    margin: float = 100.0,
) -> RawDetection:
    """Helper: create a RawDetection with pose data."""
    return RawDetection(
        tag_id=tag_id,
        center=(500.0, 500.0),
        corners=(
            (480.0, 480.0), (520.0, 480.0),
            (520.0, 520.0), (480.0, 520.0),
        ),
        decision_margin=margin,
        hamming=0,
        pose_R=R,
        pose_t=t,
    )


def _rotation_z(angle_rad: float) -> np.ndarray:
    """Rotation matrix around Z axis."""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ], dtype=np.float64)


# --- make_transform ---

class TestMakeTransform:
    def test_identity(self) -> None:
        T = make_transform(np.eye(3), np.zeros(3))
        np.testing.assert_array_almost_equal(T, np.eye(4))

    def test_translation_only(self) -> None:
        T = make_transform(np.eye(3), np.array([1.0, 2.0, 3.0]))
        assert T[0, 3] == 1.0
        assert T[1, 3] == 2.0
        assert T[2, 3] == 3.0
        np.testing.assert_array_almost_equal(T[:3, :3], np.eye(3))

    def test_rotation_only(self) -> None:
        R = _rotation_z(math.pi / 2)
        T = make_transform(R, np.zeros(3))
        np.testing.assert_array_almost_equal(T[:3, :3], R)
        np.testing.assert_array_almost_equal(T[:3, 3], np.zeros(3))

    def test_bottom_row(self) -> None:
        T = make_transform(np.eye(3), np.array([1, 2, 3]))
        np.testing.assert_array_equal(T[3, :], [0, 0, 0, 1])


# --- invert_transform ---

class TestInvertTransform:
    def test_identity_inverse_is_identity(self) -> None:
        T = np.eye(4, dtype=np.float64)
        np.testing.assert_array_almost_equal(invert_transform(T), np.eye(4))

    def test_translation_inverse(self) -> None:
        T = make_transform(np.eye(3), np.array([1.0, 2.0, 3.0]))
        T_inv = invert_transform(T)
        np.testing.assert_array_almost_equal(T_inv[:3, 3], [-1.0, -2.0, -3.0])

    def test_rotation_inverse(self) -> None:
        R = _rotation_z(math.pi / 4)
        T = make_transform(R, np.zeros(3))
        T_inv = invert_transform(T)
        result = T @ T_inv
        np.testing.assert_array_almost_equal(result, np.eye(4))

    def test_combined_inverse_is_round_trip(self) -> None:
        R = _rotation_z(0.7)
        t = np.array([0.5, -0.3, 1.2])
        T = make_transform(R, t)
        T_inv = invert_transform(T)
        np.testing.assert_array_almost_equal(T @ T_inv, np.eye(4), decimal=10)
        np.testing.assert_array_almost_equal(T_inv @ T, np.eye(4), decimal=10)


# --- extract_2d_pose ---

class TestExtract2dPose:
    def test_identity_gives_origin(self) -> None:
        x, y, theta = extract_2d_pose(np.eye(4))
        assert x == 0.0
        assert y == 0.0
        assert theta == 0.0

    def test_translation_only(self) -> None:
        T = make_transform(np.eye(3), np.array([1.5, -0.3, 0.0]))
        x, y, theta = extract_2d_pose(T)
        assert x == pytest.approx(1.5)
        assert y == pytest.approx(-0.3)
        assert theta == pytest.approx(0.0)

    def test_90_degree_rotation(self) -> None:
        R = _rotation_z(math.pi / 2)
        T = make_transform(R, np.zeros(3))
        _, _, theta = extract_2d_pose(T)
        assert theta == pytest.approx(math.pi / 2)

    def test_negative_rotation(self) -> None:
        R = _rotation_z(-math.pi / 4)
        T = make_transform(R, np.zeros(3))
        _, _, theta = extract_2d_pose(T)
        assert theta == pytest.approx(-math.pi / 4)

    def test_180_degree_rotation(self) -> None:
        R = _rotation_z(math.pi)
        T = make_transform(R, np.zeros(3))
        _, _, theta = extract_2d_pose(T)
        assert abs(theta) == pytest.approx(math.pi)


# --- compute_camera_to_arena_transform ---

class TestComputeCameraToArenaTransform:
    def test_returns_none_without_pose(self) -> None:
        det = RawDetection(
            tag_id=101,
            center=(500.0, 500.0),
            corners=((480.0, 480.0), (520.0, 480.0), (520.0, 520.0), (480.0, 520.0)),
            decision_margin=100.0,
            hamming=0,
        )
        assert compute_camera_to_arena_transform(det) is None

    def test_identity_pose_produces_valid_transform(self) -> None:
        det = _make_detection_with_pose(101, np.eye(3), np.zeros(3))
        T = compute_camera_to_arena_transform(det)
        assert T is not None
        assert T.shape == (4, 4)
        np.testing.assert_array_almost_equal(T, np.eye(4))

    def test_translated_origin_inverts_correctly(self) -> None:
        """If Tag 101 is 1m in front of camera (Z=1), arena origin should be there."""
        R = np.eye(3, dtype=np.float64)
        t = np.array([0.0, 0.0, 1.0])  # tag is 1m along camera Z
        det = _make_detection_with_pose(101, R, t)
        T = compute_camera_to_arena_transform(det)
        assert T is not None
        # A point at [0,0,1] in camera frame (where the tag is) should map to origin
        point_cam = np.array([0.0, 0.0, 1.0, 1.0])
        point_arena = T @ point_cam
        assert point_arena[0] == pytest.approx(0.0, abs=1e-10)
        assert point_arena[1] == pytest.approx(0.0, abs=1e-10)


# --- transform_detection_to_arena ---

class TestTransformDetectionToArena:
    def test_identity_transform_preserves_position(self) -> None:
        R = np.eye(3, dtype=np.float64)
        t = np.array([0.5, 0.3, 1.0])
        det = _make_detection_with_pose(5, R, t)
        T_cam_to_arena = np.eye(4, dtype=np.float64)

        pose = transform_detection_to_arena(det, T_cam_to_arena, timestamp=1.0)
        assert pose.tag_id == 5
        assert pose.x == pytest.approx(0.5)
        assert pose.y == pytest.approx(0.3)
        assert pose.timestamp == 1.0

    def test_raises_without_pose(self) -> None:
        det = RawDetection(
            tag_id=5,
            center=(500.0, 500.0),
            corners=((480.0, 480.0), (520.0, 480.0), (520.0, 520.0), (480.0, 520.0)),
            decision_margin=100.0,
            hamming=0,
        )
        with pytest.raises(ValueError, match="no pose data"):
            transform_detection_to_arena(det, np.eye(4))

    def test_confidence_carried_through(self) -> None:
        det = _make_detection_with_pose(5, np.eye(3), np.zeros(3), margin=150.0)
        pose = transform_detection_to_arena(det, np.eye(4))
        assert pose.confidence == pytest.approx(0.75)


# --- transform_detections_to_arena ---

class TestTransformDetectionsToArena:
    def test_empty_input(self) -> None:
        poses = transform_detections_to_arena([], np.eye(4))
        assert poses == []

    def test_skips_detections_without_pose(self) -> None:
        det_no_pose = RawDetection(
            tag_id=5,
            center=(500.0, 500.0),
            corners=((480.0, 480.0), (520.0, 480.0), (520.0, 520.0), (480.0, 520.0)),
            decision_margin=100.0,
            hamming=0,
        )
        det_with_pose = _make_detection_with_pose(6, np.eye(3), np.array([1.0, 0.0, 0.0]))
        poses = transform_detections_to_arena([det_no_pose, det_with_pose], np.eye(4))
        assert len(poses) == 1
        assert poses[0].tag_id == 6

    def test_multiple_detections(self) -> None:
        dets = [
            _make_detection_with_pose(0, np.eye(3), np.array([0.1, 0.2, 1.0])),
            _make_detection_with_pose(1, np.eye(3), np.array([0.5, 0.6, 1.0])),
        ]
        poses = transform_detections_to_arena(dets, np.eye(4), timestamp=42.0)
        assert len(poses) == 2
        assert poses[0].timestamp == 42.0
        assert poses[1].timestamp == 42.0
