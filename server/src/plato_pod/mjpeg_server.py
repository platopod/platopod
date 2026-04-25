"""MJPEG streaming server and REST API for the Plato Pod vision pipeline.

Serves camera feeds (raw, debug) as MJPEG streams using a lightweight
HTTP server that writes directly to sockets (no ASGI overhead).
REST endpoints use FastAPI on a separate port for camera info and homography.
No ROS2 dependency — receives frames via a thread-safe FrameStore.
"""

from __future__ import annotations

import json
import logging
import threading
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import numpy as np

logger = logging.getLogger(__name__)

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore[assignment]
    _HAS_CV2 = False


@dataclass
class FrameStore:
    """Thread-safe container for the latest camera frames and detection state.

    Uses a Condition variable so stream handlers can efficiently wait for new frames.
    """
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _condition: threading.Condition = field(default=None, repr=False)
    _raw_frame: np.ndarray | None = field(default=None, repr=False)
    _debug_frame: np.ndarray | None = field(default=None, repr=False)
    _raw_jpeg: bytes | None = field(default=None, repr=False)
    _debug_jpeg: bytes | None = field(default=None, repr=False)
    _camera_info: dict | None = field(default=None, repr=False)
    _homography: dict | None = field(default=None, repr=False)
    _detections: dict | None = field(default=None, repr=False)
    _seq: int = field(default=0, repr=False)
    _jpeg_quality: int = field(default=80, repr=False)

    def __post_init__(self) -> None:
        self._condition = threading.Condition(self._lock)

    def update_raw(self, frame: np.ndarray) -> None:
        """Update the raw camera frame and pre-encode JPEG."""
        jpeg = _encode_jpeg(frame, self._jpeg_quality)
        with self._condition:
            self._raw_frame = frame
            self._raw_jpeg = jpeg
            self._seq += 1
            self._condition.notify_all()

    def update_debug(self, frame: np.ndarray) -> None:
        """Update the debug-annotated frame and pre-encode JPEG."""
        jpeg = _encode_jpeg(frame, self._jpeg_quality)
        with self._condition:
            self._debug_frame = frame
            self._debug_jpeg = jpeg
            self._seq += 1
            self._condition.notify_all()

    def update_camera_info(self, info: dict) -> None:
        with self._lock:
            self._camera_info = info

    def update_homography(self, data: dict) -> None:
        with self._lock:
            self._homography = data

    def get_raw(self) -> np.ndarray | None:
        with self._lock:
            return self._raw_frame

    def get_debug(self) -> np.ndarray | None:
        with self._lock:
            return self._debug_frame

    def get_raw_jpeg(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._raw_jpeg, self._seq

    def get_debug_jpeg(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._debug_jpeg, self._seq

    def get_camera_info(self) -> dict | None:
        with self._lock:
            return self._camera_info

    def get_homography(self) -> dict | None:
        with self._lock:
            return self._homography

    def update_detections(self, data: dict) -> None:
        with self._lock:
            self._detections = data

    def get_detections(self) -> dict | None:
        with self._lock:
            return self._detections

    def wait_for_new_frame(self, timeout: float = 0.5) -> bool:
        """Wait until any frame is updated."""
        with self._condition:
            return self._condition.wait(timeout=timeout)

    def get_raw_seq(self) -> int:
        with self._lock:
            return self._seq

    def get_debug_seq(self) -> int:
        with self._lock:
            return self._seq


def _encode_jpeg(frame: np.ndarray, quality: int = 80) -> bytes | None:
    """Encode a BGR frame as JPEG bytes."""
    if not _HAS_CV2:
        return None
    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        return None
    return jpeg.tobytes()


class _ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    """HTTP server that handles each request in a new thread."""
    daemon_threads = True
    allow_reuse_address = True


def _make_handler(frame_store: FrameStore):
    """Create an HTTP request handler class with access to the frame store."""

    class MJPEGHandler(BaseHTTPRequestHandler):
        """Handles MJPEG stream and REST requests with direct socket writes."""

        def do_GET(self) -> None:
            if self.path == "/camera/stream":
                self._serve_mjpeg(frame_store.get_raw_jpeg)
            elif self.path == "/camera/stream/debug":
                self._serve_mjpeg(frame_store.get_debug_jpeg)
            elif self.path == "/camera/info":
                self._serve_json(frame_store.get_camera_info())
            elif self.path == "/arena/homography":
                self._serve_json(frame_store.get_homography())
            elif self.path == "/tags/detections":
                self._serve_json(frame_store.get_detections())
            elif self.path == "/health":
                self._serve_json({
                    "status": "ok",
                    "camera_active": frame_store.get_raw() is not None,
                    "opencv_available": _HAS_CV2,
                })
            else:
                self.send_error(404)

        def _serve_mjpeg(self, get_jpeg_fn) -> None:
            """Stream MJPEG directly to socket — zero framework overhead."""
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control",
                             "no-cache, no-store, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Connection", "close")
            self.end_headers()
            # Disable TCP Nagle algorithm for lower latency
            self.connection.setsockopt(
                __import__('socket').IPPROTO_TCP,
                __import__('socket').TCP_NODELAY, 1
            )

            last_seq = -1
            try:
                while True:
                    jpeg_bytes, seq = get_jpeg_fn()

                    if jpeg_bytes is None or seq == last_seq:
                        frame_store.wait_for_new_frame(timeout=0.5)
                        continue

                    last_seq = seq
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(jpeg_bytes)}\r\n\r\n".encode()
                    )
                    self.wfile.write(jpeg_bytes)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def _serve_json(self, data) -> None:
            """Serve a JSON response."""
            if data is None:
                self.send_error(503)
                return
            body = json.dumps(data).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args) -> None:
            """Suppress default request logging."""
            pass

    return MJPEGHandler


def start_mjpeg_server(
    frame_store: FrameStore, port: int = 8081
) -> threading.Thread:
    """Start the MJPEG/REST server in a daemon thread.

    Args:
        frame_store: Shared frame store updated by the vision node.
        port: HTTP server port.

    Returns:
        The daemon thread (already started).
    """
    handler = _make_handler(frame_store)
    server = _ThreadingHTTPServer(("0.0.0.0", port), handler)

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return thread
