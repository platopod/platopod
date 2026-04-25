"""Tests for plato_pod.kinematics — differential-drive kinematics."""

from __future__ import annotations

import math

import pytest

from plato_pod.kinematics import (
    add_pose_drift,
    add_velocity_noise,
    apply_acceleration_limit,
    clamp_to_boundary,
    normalize_angle,
)

UNIT_BOUNDARY = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))


# --- normalize_angle ---

class TestNormalizeAngle:
    def test_already_normal(self) -> None:
        assert normalize_angle(0.5) == pytest.approx(0.5)

    def test_positive_wrap(self) -> None:
        assert normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)

    def test_negative_wrap(self) -> None:
        assert normalize_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)

    def test_pi(self) -> None:
        assert normalize_angle(math.pi) == pytest.approx(math.pi)

    def test_negative_pi(self) -> None:
        assert normalize_angle(-math.pi) == pytest.approx(-math.pi)

    def test_large_positive(self) -> None:
        result = normalize_angle(5 * math.pi)
        assert -math.pi <= result <= math.pi

    def test_zero(self) -> None:
        assert normalize_angle(0.0) == 0.0


# --- apply_acceleration_limit ---
# (diff_drive_update tests moved to test_kinematics_model.py)

class TestApplyAccelerationLimit:
    def test_zero_limit_instant(self) -> None:
        assert apply_acceleration_limit(0.0, 0.2, 0.0, 0.02) == 0.2

    def test_accelerate_within_limit(self) -> None:
        result = apply_acceleration_limit(0.0, 0.1, 1.0, 0.02)
        assert result == pytest.approx(0.02)  # 1.0 * 0.02

    def test_decelerate_within_limit(self) -> None:
        result = apply_acceleration_limit(0.1, 0.0, 1.0, 0.02)
        assert result == pytest.approx(0.08)  # 0.1 - 0.02

    def test_target_within_one_step(self) -> None:
        result = apply_acceleration_limit(0.19, 0.2, 1.0, 0.02)
        assert result == pytest.approx(0.2)

    def test_negative_target(self) -> None:
        result = apply_acceleration_limit(0.0, -0.1, 1.0, 0.02)
        assert result == pytest.approx(-0.02)


# --- add_velocity_noise ---

class TestAddVelocityNoise:
    def test_zero_stddev_no_change(self) -> None:
        lin, ang = add_velocity_noise(0.1, 0.5, 0.0)
        assert lin == 0.1
        assert ang == 0.5

    def test_nonzero_stddev_changes_values(self) -> None:
        # With noise, values should differ (probabilistically)
        results = set()
        for _ in range(20):
            lin, _ = add_velocity_noise(0.1, 0.0, 0.01)
            results.add(round(lin, 6))
        assert len(results) > 1  # should produce different values


# --- add_pose_drift ---

class TestAddPoseDrift:
    def test_zero_stddev_no_change(self) -> None:
        x, y, th = add_pose_drift(1.0, 2.0, 0.5, 0.0)
        assert x == 1.0
        assert y == 2.0
        assert th == 0.5

    def test_nonzero_stddev_changes_values(self) -> None:
        results = set()
        for _ in range(20):
            x, _, _ = add_pose_drift(1.0, 2.0, 0.5, 0.001)
            results.add(round(x, 6))
        assert len(results) > 1


# --- clamp_to_boundary ---

class TestClampToBoundary:
    def test_inside_unchanged(self) -> None:
        x, y = clamp_to_boundary(0.5, 0.5, 0.028, UNIT_BOUNDARY)
        assert x == pytest.approx(0.5)
        assert y == pytest.approx(0.5)

    def test_outside_clamped(self) -> None:
        x, y = clamp_to_boundary(1.5, 0.5, 0.028, UNIT_BOUNDARY)
        assert x < 1.5  # should move back inside
        assert 0 <= x <= 1.0

    def test_no_boundary_unchanged(self) -> None:
        x, y = clamp_to_boundary(5.0, 5.0, 0.028, ())
        assert x == 5.0
        assert y == 5.0

    def test_slightly_outside_returns_inside(self) -> None:
        x, y = clamp_to_boundary(1.001, 0.5, 0.028, UNIT_BOUNDARY)
        assert x < 1.0
