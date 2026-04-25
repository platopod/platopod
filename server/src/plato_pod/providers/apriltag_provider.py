"""AprilTag localization provider.

Detects AprilTag markers via an overhead camera and produces RobotPose
objects in the arena coordinate frame. This is one of several possible
localization backends — the rest of the platform consumes only RobotPose.
"""

from __future__ import annotations

import logging
import time

import numpy as np

from plato_pod.config import AprilTagSettings, CameraCalibration
from plato_pod.detection import (
    RawDetection,
    build_tag_size_map,
    classify_detections,
    create_detector,
    detect_tags,
    frame_to_gray,
)
from plato_pod.pose import PoseSource, RobotPose
from plato_pod.pose_estimation import (
    ArenaPose,
    compute_camera_to_arena_transform,
    transform_detections_to_arena,
)

logger = logging.getLogger(__name__)


class AprilTagProvider:
    """Localization provider using overhead camera + AprilTag detection.

    Implements the detection → pose estimation → arena transform pipeline.
    Returns RobotPose objects with source=CAMERA_ARTAG.
    """

    name = "apriltag"
    source = PoseSource.CAMERA_ARTAG

    def __init__(
        self,
        settings: AprilTagSettings,
        calibration: CameraCalibration,
    ) -> None:
        self._settings = settings
        self._calibration = calibration
        self._tag_size_map = build_tag_size_map(settings)
        self._detector = None
        self._T_camera_to_arena: np.ndarray | None = None

        try:
            self._detector = create_detector(
                settings.detector, settings.family
            )
            logger.info("AprilTag detector initialised")
        except ImportError:
            logger.error("pupil-apriltags not installed — detection disabled")

    def detect(self, frame_bgr: np.ndarray) -> AprilTagResult:
        """Detect tags in a camera frame and return poses in arena coordinates.

        Args:
            frame_bgr: BGR camera frame.

        Returns:
            AprilTagResult with robot poses, boundary poses, and raw detections.
        """
        timestamp = time.time()

        if self._detector is None:
            return AprilTagResult.empty(timestamp)

        # Detect tags
        gray = frame_to_gray(frame_bgr)
        detections = detect_tags(
            gray, self._detector,
            camera_params=self._calibration.camera_params,
            tag_sizes=self._tag_size_map,
        )

        # Classify into robot / boundary / unknown
        robot_dets, boundary_dets, _ = classify_detections(
            detections, self._settings.tag_ranges
        )

        # Find origin tag and update camera-to-arena transform
        origin_det = None
        for det in boundary_dets:
            if det.tag_id == self._settings.tag_ranges.origin_id:
                origin_det = det
                break

        if origin_det is not None and origin_det.pose_R is not None:
            self._T_camera_to_arena = compute_camera_to_arena_transform(origin_det)
        elif self._T_camera_to_arena is not None:
            logger.debug("Origin tag not visible — using last known transform")

        # Transform to arena coordinates
        arena_poses: list[ArenaPose] = []
        robot_poses: list[ArenaPose] = []
        boundary_poses: list[ArenaPose] = []

        if self._T_camera_to_arena is not None:
            arena_poses = transform_detections_to_arena(
                detections, self._T_camera_to_arena, timestamp
            )
            robot_poses = transform_detections_to_arena(
                robot_dets, self._T_camera_to_arena, timestamp
            )
            boundary_poses = transform_detections_to_arena(
                boundary_dets, self._T_camera_to_arena, timestamp
            )

        # Convert to unified RobotPose
        robot_pose_list = [
            RobotPose(
                robot_id=p.tag_id,
                x=p.x, y=p.y, theta=p.theta,
                timestamp=timestamp,
                source=PoseSource.CAMERA_ARTAG,
                confidence=p.confidence,
                localization_id=str(p.tag_id),
            )
            for p in robot_poses
        ]

        return AprilTagResult(
            robot_poses=robot_pose_list,
            boundary_arena_poses=boundary_poses,
            all_arena_poses=arena_poses,
            all_detections=detections,
            robot_detections=robot_dets,
            boundary_detections=boundary_dets,
            origin_visible=origin_det is not None,
            timestamp=timestamp,
        )

    def get_poses(self) -> list[RobotPose]:
        """LocalizationProvider interface — not used directly.

        Use detect() instead for the full result including boundary tags.
        """
        return []


class AprilTagResult:
    """Complete result from AprilTag detection.

    Contains both the unified RobotPose list (for the localization layer)
    and AprilTag-specific data (boundary tags, raw detections for debug overlay).
    """

    def __init__(
        self,
        robot_poses: list[RobotPose],
        boundary_arena_poses: list[ArenaPose],
        all_arena_poses: list[ArenaPose],
        all_detections: list[RawDetection],
        robot_detections: list[RawDetection],
        boundary_detections: list[RawDetection],
        origin_visible: bool,
        timestamp: float,
    ) -> None:
        self.robot_poses = robot_poses
        self.boundary_arena_poses = boundary_arena_poses
        self.all_arena_poses = all_arena_poses
        self.all_detections = all_detections
        self.robot_detections = robot_detections
        self.boundary_detections = boundary_detections
        self.origin_visible = origin_visible
        self.timestamp = timestamp

    @classmethod
    def empty(cls, timestamp: float) -> AprilTagResult:
        return cls([], [], [], [], [], [], False, timestamp)
