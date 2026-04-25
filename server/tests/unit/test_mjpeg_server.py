"""Tests for plato_pod.mjpeg_server — FrameStore and HTTP endpoints."""

from __future__ import annotations

import json
import threading
import time
import urllib.request

import numpy as np
import pytest

from plato_pod.mjpeg_server import FrameStore, start_mjpeg_server


# --- FrameStore ---

class TestFrameStore:
    def test_initial_state_is_none(self) -> None:
        store = FrameStore()
        assert store.get_raw() is None
        assert store.get_debug() is None
        assert store.get_camera_info() is None
        assert store.get_homography() is None

    def test_update_and_get_raw(self) -> None:
        store = FrameStore()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        store.update_raw(frame)
        result = store.get_raw()
        assert result is not None
        np.testing.assert_array_equal(result, frame)

    def test_update_and_get_debug(self) -> None:
        store = FrameStore()
        frame = np.ones((480, 640, 3), dtype=np.uint8) * 128
        store.update_debug(frame)
        result = store.get_debug()
        assert result is not None
        np.testing.assert_array_equal(result, frame)

    def test_update_and_get_camera_info(self) -> None:
        store = FrameStore()
        info = {"width": 1920, "height": 1080, "fx": 800.0}
        store.update_camera_info(info)
        assert store.get_camera_info() == info

    def test_update_and_get_homography(self) -> None:
        store = FrameStore()
        data = {"matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1], "valid": True}
        store.update_homography(data)
        assert store.get_homography() == data

    def test_wait_for_frame_notified_by_update(self) -> None:
        store = FrameStore()
        result = [False]

        def waiter():
            result[0] = store.wait_for_new_frame(timeout=1.0)

        t = threading.Thread(target=waiter)
        t.start()
        time.sleep(0.05)
        store.update_raw(np.zeros((10, 10, 3), dtype=np.uint8))
        t.join(timeout=2.0)
        assert result[0] is True

    def test_wait_for_frame_returns_false_on_timeout(self) -> None:
        store = FrameStore()
        assert store.wait_for_new_frame(timeout=0.01) is False

    def test_raw_jpeg_encoded(self) -> None:
        store = FrameStore()
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        store.update_raw(frame)
        jpeg, seq = store.get_raw_jpeg()
        if jpeg is not None:  # cv2 available
            assert len(jpeg) > 0
            assert seq > 0


# --- HTTP server endpoints ---

@pytest.fixture(scope="module")
def server_port():
    """Start MJPEG server on a random port for testing."""
    store = FrameStore()
    store.update_camera_info({"fx": 800.0, "fy": 800.0})
    store.update_homography({"matrix": list(range(9)), "valid": True})

    import socket
    with socket.socket() as s:
        s.bind(("", 0))
        port = s.getsockname()[1]

    start_mjpeg_server(store, port=port)
    time.sleep(0.1)  # let server start
    return port, store


class TestHealthEndpoint:
    def test_health(self, server_port) -> None:
        port, store = server_port
        resp = urllib.request.urlopen(f"http://localhost:{port}/health", timeout=2)
        data = json.loads(resp.read())
        assert data["status"] == "ok"

    def test_health_camera_active(self, server_port) -> None:
        port, store = server_port
        store.update_raw(np.zeros((10, 10, 3), dtype=np.uint8))
        resp = urllib.request.urlopen(f"http://localhost:{port}/health", timeout=2)
        data = json.loads(resp.read())
        assert data["camera_active"] is True


class TestCameraInfoEndpoint:
    def test_returns_info(self, server_port) -> None:
        port, store = server_port
        resp = urllib.request.urlopen(f"http://localhost:{port}/camera/info", timeout=2)
        data = json.loads(resp.read())
        assert data["fx"] == 800.0


class TestHomographyEndpoint:
    def test_returns_data(self, server_port) -> None:
        port, store = server_port
        resp = urllib.request.urlopen(
            f"http://localhost:{port}/arena/homography", timeout=2
        )
        data = json.loads(resp.read())
        assert data["valid"] is True
