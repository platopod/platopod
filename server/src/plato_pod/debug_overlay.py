"""Debug visualisation overlay for the camera feed.

Draws tag outlines, IDs, coordinate axes, boundary polygon, and FPS counter
on camera frames. All functions take and return numpy arrays — no ROS2 dependency.

Requires OpenCV for drawing primitives. Functions gracefully return the
unmodified frame if cv2 is not available.
"""

from __future__ import annotations

import logging

import numpy as np

from plato_pod.detection import RawDetection

logger = logging.getLogger(__name__)

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

# Colours (BGR)
_GREEN = (0, 255, 0)
_RED = (0, 0, 255)
_BLUE = (255, 0, 0)
_YELLOW = (0, 255, 255)
_WHITE = (255, 255, 255)
_CYAN = (255, 255, 0)


def draw_tag_outlines(
    frame: np.ndarray,
    detections: list[RawDetection],
    color: tuple[int, int, int] = _GREEN,
    thickness: int = 2,
) -> np.ndarray:
    """Draw quadrilateral outlines around detected tags.

    Args:
        frame: BGR uint8 image (modified in place and returned).
        detections: List of detected tags with corner pixel coordinates.
        color: BGR colour tuple.
        thickness: Line thickness in pixels.

    Returns:
        The frame with outlines drawn.
    """
    if not _HAS_CV2:
        return frame

    for det in detections:
        pts = np.array(det.corners, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=thickness)

    return frame


def draw_tag_ids(
    frame: np.ndarray,
    detections: list[RawDetection],
    color: tuple[int, int, int] = _WHITE,
    font_scale: float = 0.7,
) -> np.ndarray:
    """Label each detected tag with its ID at the tag centre.

    Args:
        frame: BGR uint8 image (modified in place and returned).
        detections: List of detected tags.
        color: BGR text colour.
        font_scale: Font size multiplier.

    Returns:
        The frame with IDs drawn.
    """
    if not _HAS_CV2:
        return frame

    for det in detections:
        cx, cy = int(det.center[0]), int(det.center[1])
        label = str(det.tag_id)
        text_size = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2
        )[0]
        # Centre the text on the tag
        tx = cx - text_size[0] // 2
        ty = cy + text_size[1] // 2
        cv2.putText(
            frame, label, (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2,
        )

    return frame


def draw_axes(
    frame: np.ndarray,
    detections: list[RawDetection],
    camera_params: tuple[float, float, float, float],
    axis_length: float = 0.03,
) -> np.ndarray:
    """Draw RGB coordinate axes on each detected tag (R=X, G=Y, B=Z).

    Args:
        frame: BGR uint8 image (modified in place and returned).
        detections: Detections with pose_R and pose_t set.
        camera_params: (fx, fy, cx, cy) camera intrinsics.
        axis_length: Length of each axis in metres.

    Returns:
        The frame with axes drawn.
    """
    if not _HAS_CV2:
        return frame

    fx, fy, cx, cy = camera_params
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1],
    ], dtype=np.float64)

    for det in detections:
        if det.pose_R is None or det.pose_t is None:
            continue

        rvec, _ = cv2.Rodrigues(det.pose_R)
        tvec = det.pose_t.reshape(3, 1)

        # Project 3D axis endpoints to 2D
        axis_points = np.array([
            [0, 0, 0],
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, axis_length],
        ], dtype=np.float64)

        img_points, _ = cv2.projectPoints(
            axis_points, rvec, tvec, camera_matrix, None
        )
        img_points = img_points.reshape(-1, 2).astype(int)

        origin = tuple(img_points[0])
        cv2.line(frame, origin, tuple(img_points[1]), _RED, 2)    # X = Red
        cv2.line(frame, origin, tuple(img_points[2]), _GREEN, 2)  # Y = Green
        cv2.line(frame, origin, tuple(img_points[3]), _BLUE, 2)   # Z = Blue

    return frame


def draw_boundary(
    frame: np.ndarray,
    boundary_detections: list[RawDetection],
    color: tuple[int, int, int] = _CYAN,
    thickness: int = 2,
) -> np.ndarray:
    """Draw the arena boundary polygon connecting boundary tag centres.

    Tags are sorted by ID to ensure consistent polygon winding.

    Args:
        frame: BGR uint8 image (modified in place and returned).
        boundary_detections: Boundary tag detections (IDs 100-103).
        color: BGR colour for the boundary polygon.
        thickness: Line thickness.

    Returns:
        The frame with boundary polygon drawn.
    """
    if not _HAS_CV2 or len(boundary_detections) < 3:
        return frame

    sorted_dets = sorted(boundary_detections, key=lambda d: d.tag_id)
    pts = np.array(
        [(int(d.center[0]), int(d.center[1])) for d in sorted_dets],
        dtype=np.int32,
    ).reshape((-1, 1, 2))

    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=thickness)

    return frame


def draw_fps(
    frame: np.ndarray,
    fps: float,
    color: tuple[int, int, int] = _YELLOW,
) -> np.ndarray:
    """Draw an FPS counter in the top-left corner.

    Args:
        frame: BGR uint8 image (modified in place and returned).
        fps: Current frames per second.
        color: BGR text colour.

    Returns:
        The frame with FPS counter drawn.
    """
    if not _HAS_CV2:
        return frame

    label = f"FPS: {fps:.1f}"
    cv2.putText(
        frame, label, (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2,
    )

    return frame


def draw_all(
    frame: np.ndarray,
    detections: list[RawDetection],
    boundary_detections: list[RawDetection],
    camera_params: tuple[float, float, float, float] | None,
    fps: float,
) -> np.ndarray:
    """Apply all debug overlays to a frame.

    Args:
        frame: BGR uint8 image (modified in place and returned).
        detections: All detected tags.
        boundary_detections: Boundary tags only.
        camera_params: Camera intrinsics for axis drawing, or None to skip axes.
        fps: Current FPS.

    Returns:
        Fully annotated frame.
    """
    draw_tag_outlines(frame, detections)
    draw_tag_ids(frame, detections)
    if camera_params is not None:
        draw_axes(frame, detections, camera_params)
    draw_boundary(frame, boundary_detections)
    draw_fps(frame, fps)
    return frame
