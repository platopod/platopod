"""Vision Node — camera capture, localization, and MJPEG streaming.

Thin ROS2 shell that manages the camera, delegates detection to a
LocalizationProvider (AprilTagProvider by default), and publishes
poses and camera streams.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from plato_pod_msgs.msg import HomographyMatrix, TagDetections, TagPose

from plato_pod.camera import create_camera
from plato_pod.config import (
    AprilTagSettings,
    CameraCalibration,
    default_calibration,
    load_apriltag_settings,
    load_camera_calibration,
)
from plato_pod.debug_overlay import draw_all
from plato_pod.homography import compute_homography
from plato_pod.mjpeg_server import FrameStore, start_mjpeg_server
from plato_pod.providers.apriltag_provider import AprilTagProvider

logger = logging.getLogger(__name__)


class VisionNode(Node):
    """ROS2 node for camera capture, localization, and pose publishing."""

    def __init__(self) -> None:
        super().__init__("vision_node")

        # Resolve default config paths
        share_dir = get_package_share_directory("plato_pod")
        default_apriltag_config = str(Path(share_dir) / "config" / "apriltag_settings.yaml")
        default_calibration_file = str(Path(share_dir) / "config" / "camera_calibration.yaml")

        # Declare parameters
        self.declare_parameter("camera_device", 0)
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("camera_autofocus", False)
        self.declare_parameter("camera_focus", -1)
        self.declare_parameter("camera_zoom", 100)
        self.declare_parameter("camera_pixel_format", "mjpg")
        self.declare_parameter("calibration_file", default_calibration_file)
        self.declare_parameter("apriltag_config", default_apriltag_config)
        self.declare_parameter("debug_overlay", False)
        self.declare_parameter("detection_enabled", True)
        self.declare_parameter("mjpeg_quality", 10)
        self.declare_parameter("mjpeg_port", 8081)

        # Read parameters
        camera_device = self.get_parameter("camera_device").value
        camera_width = self.get_parameter("camera_width").value
        camera_height = self.get_parameter("camera_height").value
        camera_fps = self.get_parameter("camera_fps").value
        camera_autofocus = self.get_parameter("camera_autofocus").value
        camera_zoom = self.get_parameter("camera_zoom").value
        camera_pixel_format = self.get_parameter("camera_pixel_format").value
        camera_focus = self.get_parameter("camera_focus").value
        calibration_file = self.get_parameter("calibration_file").value
        apriltag_config = self.get_parameter("apriltag_config").value
        self._debug_overlay = self.get_parameter("debug_overlay").value
        self._detection_enabled = self.get_parameter("detection_enabled").value
        mjpeg_quality = self.get_parameter("mjpeg_quality").value
        mjpeg_port = self.get_parameter("mjpeg_port").value

        # Load configuration
        self._settings = self._load_apriltag_settings(apriltag_config)
        self._calibration = self._load_calibration(
            calibration_file, camera_width, camera_height
        )

        # Camera setup
        self._camera = create_camera(
            camera_device, camera_width, camera_height, camera_fps,
            autofocus=camera_autofocus,
            focus=camera_focus if camera_focus >= 0 else None,
            zoom=camera_zoom,
            pixel_format=camera_pixel_format,
        )
        self._virtual_mode = not self._camera.is_opened() or (
            type(self._camera).__name__ == "VirtualCamera"
        )

        if self._virtual_mode:
            self.get_logger().warning("No camera available — running in virtual-only mode")
        else:
            self.get_logger().info(
                f"Camera opened: device={camera_device}, "
                f"{camera_width}x{camera_height}@{camera_fps}fps"
            )

        # Localization provider (AprilTag)
        self._provider: AprilTagProvider | None = None
        if not self._virtual_mode:
            self._provider = AprilTagProvider(self._settings, self._calibration)

        # State
        self._frame_id: int = 0
        self._last_homography: np.ndarray | None = None
        self._fps_counter = _FpsCounter()

        # Publishers
        self._pub_detections = self.create_publisher(TagDetections, "/tags/detections", 10)
        self._pub_homography = self.create_publisher(HomographyMatrix, "/arena/homography", 10)
        self._pub_camera_info = self.create_publisher(CameraInfo, "/camera/info", 10)

        # MJPEG server
        self._frame_store = FrameStore()
        self._frame_store._jpeg_quality = mjpeg_quality
        self._frame_store.update_camera_info(self._make_camera_info_dict())

        start_mjpeg_server(self._frame_store, port=mjpeg_port)
        self.get_logger().info(f"MJPEG server started on port {mjpeg_port}")

        # Timer — detection loop
        timer_period = 1.0 / self._settings.detection_fps
        self._timer = self.create_timer(timer_period, self._timer_callback)
        self.get_logger().info(f"Vision node ready ({self._settings.detection_fps:.0f} Hz)")

    def _load_apriltag_settings(self, config_path: str) -> AprilTagSettings:
        """Load AprilTag settings, falling back to defaults."""
        try:
            settings = load_apriltag_settings(config_path)
            self.get_logger().info(f"AprilTag settings loaded from {config_path}")
            return settings
        except (FileNotFoundError, KeyError) as e:
            self.get_logger().warning(
                f"Could not load apriltag config '{config_path}': {e} — using defaults"
            )
            return AprilTagSettings()

    def _load_calibration(
        self, calib_path: str, width: int, height: int
    ) -> CameraCalibration:
        """Load camera calibration, falling back to estimated defaults."""
        try:
            cal = load_camera_calibration(calib_path)
            self.get_logger().info(f"Camera calibration loaded from {calib_path}")
            return cal
        except (FileNotFoundError, KeyError) as e:
            self.get_logger().warning(
                f"No calibration file '{calib_path}': {e} — using estimated defaults"
            )
            return default_calibration(width, height)

    def _timer_callback(self) -> None:
        """Main detection loop — runs at detection_fps."""
        ok, frame = self._camera.read()
        if not ok or frame is None:
            self._publish_empty_detections()
            return

        self._frame_id += 1
        timestamp = time.time()

        # Update raw frame for MJPEG
        self._frame_store.update_raw(frame)

        # Run localization provider
        if self._provider is not None and self._detection_enabled:
            result = self._provider.detect(frame)

            # Compute homography from all tags with poses
            arena_pose_by_id: dict[int, object] = {}
            for pose in result.all_arena_poses:
                arena_pose_by_id[pose.tag_id] = pose

            paired_pixel = []
            paired_arena = []
            for det in result.all_detections:
                pose = arena_pose_by_id.get(det.tag_id)
                if pose is not None:
                    paired_pixel.append(det.center)
                    paired_arena.append([pose.x, pose.y])

            if len(paired_pixel) >= 4:
                pixel_pts = np.array(paired_pixel, dtype=np.float64)
                arena_pts = np.array(paired_arena, dtype=np.float64)
                H = compute_homography(pixel_pts, arena_pts)
                if H is not None:
                    self._last_homography = H
                    self._publish_homography(H, timestamp)
                    self._frame_store.update_homography({
                        "matrix": H.flatten().tolist(),
                        "valid": True,
                        "timestamp": timestamp,
                    })

            # Publish detection pixel positions for dashboard
            det_data = {
                "boundary_tags": [
                    {"tag_id": d.tag_id, "center": list(d.center),
                     "corners": [list(c) for c in d.corners]}
                    for d in result.boundary_detections
                ],
                "robot_tags": [
                    {"tag_id": d.tag_id, "center": list(d.center),
                     "corners": [list(c) for c in d.corners]}
                    for d in result.robot_detections
                ],
                "timestamp": timestamp,
            }
            self._frame_store.update_detections(det_data)

            # FPS
            fps = self._fps_counter.tick()

            # Debug overlay
            if self._debug_overlay:
                debug_frame = frame.copy()
                draw_all(
                    debug_frame,
                    result.all_detections,
                    result.boundary_detections,
                    self._calibration.camera_params,
                    fps,
                )
                self._frame_store.update_debug(debug_frame)

            # Publish detections (legacy TagDetections format — will be replaced)
            self._publish_detections(
                result.all_arena_poses,
                result.boundary_arena_poses,
                [p for p in result.all_arena_poses
                 if self._settings.tag_ranges.classify(p.tag_id).name == "ROBOT"],
                origin_visible=result.origin_visible,
                frame_id=self._frame_id,
                fps=fps,
            )
        else:
            fps = self._fps_counter.tick()
            if self._debug_overlay:
                self._frame_store.update_debug(frame.copy())
            self._publish_empty_detections()

    def _publish_detections(
        self,
        all_poses, boundary_poses, robot_poses,
        origin_visible, frame_id, fps,
    ) -> None:
        """Publish TagDetections message."""
        msg = TagDetections()
        msg.tags = [self._arena_pose_to_msg(p) for p in all_poses]
        msg.boundary_tags = [self._arena_pose_to_msg(p) for p in boundary_poses]
        msg.robot_tags = [self._arena_pose_to_msg(p) for p in robot_poses]
        msg.origin_visible = origin_visible
        msg.frame_id = frame_id
        msg.fps = fps
        self._pub_detections.publish(msg)

    def _publish_empty_detections(self) -> None:
        """Publish an empty detections message."""
        msg = TagDetections()
        msg.tags = []
        msg.boundary_tags = []
        msg.robot_tags = []
        msg.origin_visible = False
        msg.frame_id = self._frame_id
        msg.fps = 0.0
        self._pub_detections.publish(msg)

    def _publish_homography(self, H: np.ndarray, timestamp: float) -> None:
        """Publish HomographyMatrix message."""
        msg = HomographyMatrix()
        msg.matrix = H.flatten().tolist()
        msg.valid = True
        msg.timestamp = float(timestamp)
        self._pub_homography.publish(msg)

    @staticmethod
    def _arena_pose_to_msg(pose) -> TagPose:
        """Convert an ArenaPose to a TagPose ROS2 message."""
        msg = TagPose()
        msg.tag_id = pose.tag_id
        msg.x = float(pose.x)
        msg.y = float(pose.y)
        msg.theta = float(pose.theta)
        msg.confidence = float(pose.confidence)
        msg.timestamp = float(pose.timestamp)
        return msg

    def _make_camera_info_dict(self) -> dict:
        """Build camera info dict for REST endpoint."""
        cal = self._calibration
        return {
            "image_width": cal.image_width,
            "image_height": cal.image_height,
            "fx": cal.fx,
            "fy": cal.fy,
            "cx": cal.cx,
            "cy": cal.cy,
            "distortion": cal.distortion_coefficients.tolist(),
            "calibration_date": cal.calibration_date,
        }

    def destroy_node(self) -> None:
        """Clean up camera on shutdown."""
        self._camera.release()
        super().destroy_node()


class _FpsCounter:
    """Simple FPS counter using a rolling window."""

    def __init__(self, window: int = 30) -> None:
        self._timestamps: list[float] = []
        self._window = window

    def tick(self) -> float:
        """Record a frame and return the current FPS."""
        now = time.monotonic()
        self._timestamps.append(now)
        if len(self._timestamps) > self._window:
            self._timestamps = self._timestamps[-self._window:]
        if len(self._timestamps) < 2:
            return 0.0
        elapsed = self._timestamps[-1] - self._timestamps[0]
        if elapsed <= 0:
            return 0.0
        return (len(self._timestamps) - 1) / elapsed


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod vision_node."""
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
