"""Tests for plato_pod.debug_overlay — visualisation overlay functions."""

from __future__ import annotations

import numpy as np
import pytest

from plato_pod.debug_overlay import (
    _HAS_CV2,
    draw_all,
    draw_boundary,
    draw_fps,
    draw_tag_ids,
    draw_tag_outlines,
)
from plato_pod.detection import RawDetection

needs_cv2 = pytest.mark.skipif(not _HAS_CV2, reason="OpenCV not installed")


def _make_detection(tag_id: int, cx: float = 500.0, cy: float = 500.0) -> RawDetection:
    """Helper to create a detection at a given centre."""
    offset = 20.0
    return RawDetection(
        tag_id=tag_id,
        center=(cx, cy),
        corners=(
            (cx - offset, cy - offset),
            (cx + offset, cy - offset),
            (cx + offset, cy + offset),
            (cx - offset, cy + offset),
        ),
        decision_margin=100.0,
        hamming=0,
    )


def _blank_frame(h: int = 480, w: int = 640) -> np.ndarray:
    return np.zeros((h, w, 3), dtype=np.uint8)


class TestDrawTagOutlines:
    def test_returns_frame_same_shape(self) -> None:
        frame = _blank_frame()
        det = _make_detection(0)
        result = draw_tag_outlines(frame, [det])
        assert result.shape == frame.shape

    def test_empty_detections_no_change(self) -> None:
        frame = _blank_frame()
        original = frame.copy()
        draw_tag_outlines(frame, [])
        np.testing.assert_array_equal(frame, original)

    def test_modifies_frame_in_place(self) -> None:
        frame = _blank_frame()
        det = _make_detection(0, 100.0, 100.0)
        result = draw_tag_outlines(frame, [det])
        # result should be the same object
        assert result is frame


class TestDrawTagIds:
    def test_returns_frame_same_shape(self) -> None:
        frame = _blank_frame()
        det = _make_detection(42)
        result = draw_tag_ids(frame, [det])
        assert result.shape == frame.shape


class TestDrawBoundary:
    def test_needs_at_least_3_tags(self) -> None:
        frame = _blank_frame()
        original = frame.copy()
        dets = [_make_detection(100), _make_detection(101)]
        draw_boundary(frame, dets)
        np.testing.assert_array_equal(frame, original)

    @needs_cv2
    def test_draws_with_4_tags(self) -> None:
        frame = _blank_frame()
        dets = [
            _make_detection(100, 100, 100),
            _make_detection(101, 500, 100),
            _make_detection(102, 500, 400),
            _make_detection(103, 100, 400),
        ]
        draw_boundary(frame, dets)
        # Should have drawn something (frame not all zeros)
        assert np.any(frame != 0)


class TestDrawFps:
    def test_returns_frame_same_shape(self) -> None:
        frame = _blank_frame()
        result = draw_fps(frame, 30.0)
        assert result.shape == frame.shape


class TestDrawAll:
    def test_does_not_crash_on_empty(self) -> None:
        frame = _blank_frame()
        result = draw_all(frame, [], [], None, 0.0)
        assert result.shape == frame.shape

    def test_does_not_crash_with_detections(self) -> None:
        frame = _blank_frame()
        dets = [_make_detection(5, 100, 100)]
        boundary = [
            _make_detection(100, 50, 50),
            _make_detection(101, 590, 50),
            _make_detection(102, 590, 430),
            _make_detection(103, 50, 430),
        ]
        result = draw_all(frame, dets + boundary, boundary, None, 25.0)
        assert result.shape == frame.shape
