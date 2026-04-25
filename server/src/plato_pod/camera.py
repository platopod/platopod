"""Camera capture abstraction for the Plato Pod vision pipeline.

Provides a unified FrameSource protocol with three implementations:
- UsbCamera: real USB camera via OpenCV VideoCapture
- FileCamera: video file or image for testing/CI
- VirtualCamera: blank frames for virtual-only mode

No ROS2 dependency.
"""

from __future__ import annotations

import logging
from typing import Protocol

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)


class FrameSource(Protocol):
    """Protocol for camera frame sources."""

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Read a single frame. Returns (success, frame_bgr)."""
        ...

    def release(self) -> None:
        """Release the camera resource."""
        ...

    def is_opened(self) -> bool:
        """Check if the camera is available."""
        ...


class UsbCamera:
    """Real USB camera via OpenCV VideoCapture."""

    def __init__(
        self,
        device: int,
        width: int,
        height: int,
        fps: int,
        autofocus: bool = False,
        focus: int | None = None,
        zoom: int | None = None,
        pixel_format: str = "mjpg",
    ) -> None:
        # Use V4L2 backend explicitly
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

        if self._cap.isOpened():
            # Set pixel format: "mjpg" for camera-compressed, "yuyv" for raw
            if pixel_format.lower() == "mjpg":
                self._cap.set(cv2.CAP_PROP_FOURCC,
                              cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
            elif pixel_format.lower() == "yuyv":
                self._cap.set(cv2.CAP_PROP_FOURCC,
                              cv2.VideoWriter.fourcc('Y', 'U', 'Y', 'V'))
            # else: leave camera default

            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._cap.set(cv2.CAP_PROP_FPS, fps)

            # V4L2 focus/zoom controls
            self._cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofocus else 0)
            if not autofocus and focus is not None:
                self._cap.set(cv2.CAP_PROP_FOCUS, focus)
            if zoom is not None:
                self._cap.set(cv2.CAP_PROP_ZOOM, zoom)

            actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
            actual_focus = int(self._cap.get(cv2.CAP_PROP_FOCUS))
            actual_zoom = int(self._cap.get(cv2.CAP_PROP_ZOOM))
            # Read back actual fourcc to confirm format
            fourcc_int = int(self._cap.get(cv2.CAP_PROP_FOURCC))
            actual_fourcc = "".join(chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4))
            logger.info(
                "USB camera opened: device=%d, resolution=%dx%d, fps=%.1f, "
                "format=%s, autofocus=%s, focus=%d, zoom=%d",
                device, actual_w, actual_h, actual_fps,
                actual_fourcc, autofocus, actual_focus, actual_zoom,
            )
        else:
            logger.warning("USB camera device %d could not be opened", device)

    def read(self) -> tuple[bool, np.ndarray | None]:
        ret, frame = self._cap.read()
        if not ret:
            return False, None
        return True, frame

    def release(self) -> None:
        self._cap.release()

    def is_opened(self) -> bool:
        return self._cap.isOpened()


class FileCamera:
    """Video file or single image as a frame source (for testing/CI)."""

    def __init__(self, path: str, loop: bool = True) -> None:
        self._path = path
        self._loop = loop
        self._cap = cv2.VideoCapture(path)
        if not self._cap.isOpened():
            # Try as a single image
            img = cv2.imread(path)
            if img is not None:
                self._single_image = img
                self._is_image = True
                logger.info("FileCamera loaded single image: %s", path)
            else:
                self._single_image = None
                self._is_image = False
                logger.warning("FileCamera could not open: %s", path)
        else:
            self._single_image = None
            self._is_image = False
            logger.info("FileCamera opened video: %s", path)

    def read(self) -> tuple[bool, np.ndarray | None]:
        if self._is_image:
            if self._single_image is not None:
                return True, self._single_image.copy()
            return False, None

        ret, frame = self._cap.read()
        if not ret and self._loop:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self._cap.read()
        if not ret:
            return False, None
        return True, frame

    def release(self) -> None:
        self._cap.release()

    def is_opened(self) -> bool:
        if self._is_image:
            return self._single_image is not None
        return self._cap.isOpened()


class VirtualCamera:
    """Generates blank frames for virtual-only mode (no physical camera)."""

    def __init__(self, width: int = 1920, height: int = 1080) -> None:
        self._frame = np.zeros((height, width, 3), dtype=np.uint8)
        logger.info("VirtualCamera active: %dx%d (no physical camera)", width, height)

    def read(self) -> tuple[bool, np.ndarray | None]:
        return True, self._frame.copy()

    def release(self) -> None:
        pass

    def is_opened(self) -> bool:
        return True


def create_camera(
    device: int | str,
    width: int = 1280,
    height: int = 720,
    fps: int = 30,
    autofocus: bool = False,
    focus: int | None = None,
    zoom: int | None = None,
    pixel_format: str = "mjpg",
) -> FrameSource:
    """Factory: create the best available frame source.

    Tries the specified device. If it's a string, treats it as a file path.
    If it's an int, tries a USB camera. Falls back to VirtualCamera if nothing works.

    Args:
        device: Camera device index (int) or file path (str).
        width: Desired frame width.
        height: Desired frame height.
        fps: Desired frame rate.
        autofocus: Enable autofocus (default: off for calibration consistency).
        focus: Manual focus value (0-255, camera-dependent). Only used when autofocus=False.
        zoom: Zoom level (100=1x, 200=2x, etc.). None leaves the camera default.
        pixel_format: Camera pixel format — "mjpg" or "yuyv".

    Returns:
        A FrameSource instance.
    """
    if isinstance(device, str):
        cam = FileCamera(device)
        if cam.is_opened():
            return cam
        logger.warning("File camera '%s' failed, falling back to virtual", device)
        return VirtualCamera(width, height)

    cam = UsbCamera(device, width, height, fps, autofocus, focus, zoom, pixel_format)
    if cam.is_opened():
        return cam
    logger.warning("USB camera %d unavailable, falling back to virtual", device)
    return VirtualCamera(width, height)
