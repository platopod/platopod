"""Tests for plato_pod.detection — tag classification and detection utilities."""

from __future__ import annotations

import numpy as np
import pytest

from plato_pod.config import AprilTagSettings, TagCategory, TagRanges, TagSizes
from plato_pod.detection import (
    RawDetection,
    build_tag_size_map,
    classify_detections,
    frame_to_gray,
)


def _make_detection(tag_id: int, margin: float = 100.0) -> RawDetection:
    """Helper to create a RawDetection with minimal data."""
    return RawDetection(
        tag_id=tag_id,
        center=(500.0, 500.0),
        corners=(
            (480.0, 480.0),
            (520.0, 480.0),
            (520.0, 520.0),
            (480.0, 520.0),
        ),
        decision_margin=margin,
        hamming=0,
    )


# --- RawDetection.confidence ---

class TestRawDetectionConfidence:
    def test_high_margin_gives_high_confidence(self) -> None:
        det = _make_detection(0, margin=200.0)
        assert det.confidence == 1.0

    def test_very_high_margin_clamped_to_one(self) -> None:
        det = _make_detection(0, margin=400.0)
        assert det.confidence == 1.0

    def test_zero_margin_gives_zero_confidence(self) -> None:
        det = _make_detection(0, margin=0.0)
        assert det.confidence == 0.0

    def test_mid_margin_gives_half_confidence(self) -> None:
        det = _make_detection(0, margin=100.0)
        assert det.confidence == pytest.approx(0.5)


# --- build_tag_size_map ---

class TestBuildTagSizeMap:
    def test_contains_all_robot_tags(self) -> None:
        settings = AprilTagSettings()
        sizes = build_tag_size_map(settings)
        for tag_id in range(0, 16):
            assert tag_id in sizes
            assert sizes[tag_id] == 0.050

    def test_contains_all_boundary_tags(self) -> None:
        settings = AprilTagSettings()
        sizes = build_tag_size_map(settings)
        for tag_id in range(100, 104):
            assert tag_id in sizes
            assert sizes[tag_id] == 0.070

    def test_does_not_contain_unknown_tags(self) -> None:
        settings = AprilTagSettings()
        sizes = build_tag_size_map(settings)
        assert 50 not in sizes
        assert 200 not in sizes

    def test_total_count(self) -> None:
        settings = AprilTagSettings()
        sizes = build_tag_size_map(settings)
        # 16 robot (0-15) + 4 boundary (100-103)
        assert len(sizes) == 20


# --- classify_detections ---

class TestClassifyDetections:
    def test_empty_input_returns_empty(self, tag_ranges: TagRanges) -> None:
        robots, boundary, unknown = classify_detections([], tag_ranges)
        assert robots == []
        assert boundary == []
        assert unknown == []

    def test_robot_tags_classified(self, tag_ranges: TagRanges) -> None:
        dets = [_make_detection(0), _make_detection(5), _make_detection(15)]
        robots, boundary, unknown = classify_detections(dets, tag_ranges)
        assert len(robots) == 3
        assert len(boundary) == 0
        assert len(unknown) == 0

    def test_boundary_tags_classified(self, tag_ranges: TagRanges) -> None:
        dets = [_make_detection(100), _make_detection(101), _make_detection(103)]
        robots, boundary, unknown = classify_detections(dets, tag_ranges)
        assert len(robots) == 0
        assert len(boundary) == 3

    def test_origin_is_boundary(self, tag_ranges: TagRanges) -> None:
        """Tag 101 (origin) is classified with boundary tags."""
        dets = [_make_detection(101)]
        robots, boundary, unknown = classify_detections(dets, tag_ranges)
        assert len(boundary) == 1
        assert boundary[0].tag_id == 101

    def test_unknown_tags_classified(self, tag_ranges: TagRanges) -> None:
        dets = [_make_detection(50), _make_detection(200)]
        robots, boundary, unknown = classify_detections(dets, tag_ranges)
        assert len(unknown) == 2

    def test_mixed_tags_split_correctly(self, tag_ranges: TagRanges) -> None:
        dets = [
            _make_detection(3),    # robot
            _make_detection(101),  # boundary (origin)
            _make_detection(50),   # unknown
            _make_detection(100),  # boundary
        ]
        robots, boundary, unknown = classify_detections(dets, tag_ranges)
        assert len(robots) == 1
        assert len(boundary) == 2
        assert len(unknown) == 1


# --- frame_to_gray ---

class TestFrameToGray:
    def test_bgr_to_gray_shape(self) -> None:
        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        gray = frame_to_gray(bgr)
        assert gray.shape == (480, 640)
        assert gray.dtype == np.uint8

    def test_already_gray_passthrough(self) -> None:
        gray_in = np.ones((480, 640), dtype=np.uint8) * 128
        gray_out = frame_to_gray(gray_in)
        assert gray_out.shape == (480, 640)
        np.testing.assert_array_equal(gray_in, gray_out)

    def test_white_bgr_produces_gray_white(self) -> None:
        bgr = np.full((10, 10, 3), 255, dtype=np.uint8)
        gray = frame_to_gray(bgr)
        # B*0.114 + G*0.587 + R*0.299 = 255*(0.114+0.587+0.299) = 255
        assert np.all(gray == 255)

    def test_output_is_contiguous(self) -> None:
        bgr = np.zeros((100, 100, 3), dtype=np.uint8)
        gray = frame_to_gray(bgr)
        assert gray.flags['C_CONTIGUOUS']
