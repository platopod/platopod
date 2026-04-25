"""Tests for plato_pod.kinematics_model — pluggable robot motion models."""

from __future__ import annotations

import math

import pytest

from plato_pod.kinematics_model import (
    DifferentialDrive,
    Omnidirectional,
    get_kinematics_model,
)


class TestDifferentialDrive:
    def test_straight_forward(self) -> None:
        m = DifferentialDrive()
        x, y, th = m.predict(0, 0, 0, 0.1, 0.0, 1.0)
        assert x == pytest.approx(0.1)
        assert y == pytest.approx(0.0)
        assert th == pytest.approx(0.0)

    def test_pure_rotation(self) -> None:
        m = DifferentialDrive()
        x, y, th = m.predict(0.5, 0.5, 0, 0.0, 1.0, 0.5)
        assert x == pytest.approx(0.5)
        assert y == pytest.approx(0.5)
        assert th == pytest.approx(0.5)

    def test_theta_normalized(self) -> None:
        m = DifferentialDrive()
        _, _, th = m.predict(0, 0, 3.0, 0, 1.0, 1.0)
        assert -math.pi <= th <= math.pi

    def test_backward(self) -> None:
        m = DifferentialDrive()
        x, _, _ = m.predict(1.0, 0, 0, -0.1, 0, 1.0)
        assert x == pytest.approx(0.9)

    def test_name(self) -> None:
        assert DifferentialDrive.name == "differential_drive"


class TestOmnidirectional:
    def test_forward(self) -> None:
        m = Omnidirectional()
        x, y, th = m.predict(0, 0, 0, 0.1, 0.0, 1.0)
        assert x == pytest.approx(0.1)
        assert y == pytest.approx(0.0)

    def test_name(self) -> None:
        assert Omnidirectional.name == "omnidirectional"


class TestGetKinematicsModel:
    def test_get_differential_drive(self) -> None:
        m = get_kinematics_model("differential_drive")
        assert m.name == "differential_drive"

    def test_get_omnidirectional(self) -> None:
        m = get_kinematics_model("omnidirectional")
        assert m.name == "omnidirectional"

    def test_unknown_raises(self) -> None:
        with pytest.raises(KeyError):
            get_kinematics_model("unknown_model")
