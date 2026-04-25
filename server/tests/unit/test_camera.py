"""Tests for plato_pod.camera — frame source abstraction."""

from __future__ import annotations

import numpy as np

from plato_pod.camera import VirtualCamera


class TestVirtualCamera:
    def test_is_always_opened(self) -> None:
        cam = VirtualCamera(640, 480)
        assert cam.is_opened()

    def test_read_returns_success(self) -> None:
        cam = VirtualCamera(640, 480)
        ok, frame = cam.read()
        assert ok is True
        assert frame is not None

    def test_read_returns_correct_shape(self) -> None:
        cam = VirtualCamera(640, 480)
        _, frame = cam.read()
        assert frame.shape == (480, 640, 3)

    def test_read_returns_black_frame(self) -> None:
        cam = VirtualCamera(320, 240)
        _, frame = cam.read()
        assert frame.dtype == np.uint8
        assert np.all(frame == 0)

    def test_read_returns_copies(self) -> None:
        cam = VirtualCamera(320, 240)
        _, frame1 = cam.read()
        _, frame2 = cam.read()
        frame1[0, 0, 0] = 255
        assert frame2[0, 0, 0] == 0

    def test_release_is_noop(self) -> None:
        cam = VirtualCamera()
        cam.release()
        assert cam.is_opened()

    def test_default_resolution(self) -> None:
        cam = VirtualCamera()
        _, frame = cam.read()
        assert frame.shape == (1080, 1920, 3)
