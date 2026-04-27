"""Tests for plato_pod.health — casualty and mobility model."""

from __future__ import annotations

import pytest

from plato_pod.health import (
    HEALTH_DESTROYED,
    HEALTH_WOUNDED,
    apply_damage,
    fire_capability,
    gas_exposure_damage,
    heal,
    incapacitate,
    mobility_factor,
    status_for_health,
)
from plato_pod.robot import (
    Robot,
    STATUS_ACTIVE,
    STATUS_DESTROYED,
    STATUS_INCAPACITATED,
    STATUS_WOUNDED,
)


def _robot(health: float = 1.0, status: str = STATUS_ACTIVE) -> Robot:
    return Robot(
        robot_id=1, deployment="virtual", x=0.0, y=0.0,
        health=health, status=status,
    )


class TestMobilityFactor:
    def test_full_health_full_mobility(self) -> None:
        assert mobility_factor(1.0) == 1.0

    def test_destroyed_no_mobility(self) -> None:
        assert mobility_factor(0.0) == 0.0
        assert mobility_factor(-0.1) == 0.0

    def test_wounded_threshold(self) -> None:
        # at exactly 0.5 we expect 0.5
        assert mobility_factor(0.5) == pytest.approx(0.5)

    def test_below_wounded_reduced(self) -> None:
        # h=0.3 → 0.1 + (0.3/0.5)*0.4 = 0.34
        assert mobility_factor(0.3) == pytest.approx(0.34)

    def test_monotonic_increasing(self) -> None:
        prev = -1.0
        for h in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
            f = mobility_factor(h)
            assert f >= prev
            prev = f


class TestFireCapability:
    def test_full_health_full_capability(self) -> None:
        assert fire_capability(1.0) == 1.0

    def test_destroyed_zero_capability(self) -> None:
        assert fire_capability(0.0) == 0.0

    def test_critical_degradation(self) -> None:
        # h=0.1 (below 0.2 critical) → 0.1/0.2 * 0.5 = 0.25
        assert fire_capability(0.1) == pytest.approx(0.25)

    def test_full_above_wounded(self) -> None:
        assert fire_capability(0.6) == 1.0
        assert fire_capability(0.5) == 1.0

    def test_wounded_range(self) -> None:
        # h=0.35 (between 0.2 and 0.5) → 0.5 + (0.35-0.2)/(0.3) * 0.5 = 0.75
        assert fire_capability(0.35) == pytest.approx(0.75)


class TestStatusForHealth:
    def test_active_at_full_health(self) -> None:
        assert status_for_health(STATUS_ACTIVE, 1.0) == STATUS_ACTIVE
        assert status_for_health(STATUS_ACTIVE, 0.6) == STATUS_ACTIVE

    def test_wounded_below_threshold(self) -> None:
        assert status_for_health(STATUS_ACTIVE, 0.4) == STATUS_WOUNDED
        assert status_for_health(STATUS_ACTIVE, 0.1) == STATUS_WOUNDED

    def test_destroyed_at_zero_health(self) -> None:
        assert status_for_health(STATUS_ACTIVE, 0.0) == STATUS_DESTROYED
        assert status_for_health(STATUS_ACTIVE, -0.1) == STATUS_DESTROYED

    def test_incapacitated_persists(self) -> None:
        # Incapacitated overrides health-based status
        assert status_for_health(STATUS_INCAPACITATED, 1.0) == STATUS_INCAPACITATED
        assert status_for_health(STATUS_INCAPACITATED, 0.5) == STATUS_INCAPACITATED


class TestApplyDamage:
    def test_reduces_health(self) -> None:
        r = _robot(health=1.0)
        r2 = apply_damage(r, 0.3)
        assert r2.health == pytest.approx(0.7)
        assert r2.status == STATUS_ACTIVE

    def test_status_transitions_to_wounded(self) -> None:
        r = _robot(health=1.0)
        r2 = apply_damage(r, 0.6)
        assert r2.health == pytest.approx(0.4)
        assert r2.status == STATUS_WOUNDED

    def test_status_transitions_to_destroyed(self) -> None:
        r = _robot(health=0.3)
        r2 = apply_damage(r, 0.5)
        assert r2.health == 0.0
        assert r2.status == STATUS_DESTROYED

    def test_clamped_to_zero(self) -> None:
        r = _robot(health=0.1)
        r2 = apply_damage(r, 999.0)
        assert r2.health == 0.0
        assert r2.status == STATUS_DESTROYED

    def test_negative_damage_ignored(self) -> None:
        r = _robot(health=0.5)
        r2 = apply_damage(r, -0.2)
        assert r2.health == 0.5

    def test_immutable(self) -> None:
        r = _robot(health=1.0)
        apply_damage(r, 0.3)
        # original unchanged
        assert r.health == 1.0


class TestHeal:
    def test_restores_health(self) -> None:
        r = _robot(health=0.3, status=STATUS_WOUNDED)
        r2 = heal(r, 0.4)
        assert r2.health == pytest.approx(0.7)
        assert r2.status == STATUS_ACTIVE

    def test_clamped_to_one(self) -> None:
        r = _robot(health=0.8)
        r2 = heal(r, 999.0)
        assert r2.health == 1.0


class TestIncapacitate:
    def test_sets_incapacitated_status(self) -> None:
        r = _robot(health=1.0)
        r2 = incapacitate(r)
        assert r2.status == STATUS_INCAPACITATED
        assert r2.health == 1.0  # health unchanged


class TestGasExposureDamage:
    def test_below_threshold_no_damage(self) -> None:
        assert gas_exposure_damage(50.0, threshold=100.0) == 0.0

    def test_above_threshold_proportional(self) -> None:
        # overage = 200 - 100 = 100
        # damage = 100 * 0.001 * 0.1 = 0.01
        d = gas_exposure_damage(200.0, threshold=100.0,
                                 rate_per_unit_per_second=0.001, dt=0.1)
        assert d == pytest.approx(0.01)

    def test_zero_dt_no_damage(self) -> None:
        assert gas_exposure_damage(500.0, threshold=100.0, dt=0.0) == 0.0


class TestRobotIsOperational:
    def test_active_is_operational(self) -> None:
        assert _robot(status=STATUS_ACTIVE).is_operational()

    def test_wounded_still_operational(self) -> None:
        # Wounded units can still move (slowly) and fire (poorly)
        assert _robot(status=STATUS_WOUNDED).is_operational()

    def test_destroyed_not_operational(self) -> None:
        assert not _robot(status=STATUS_DESTROYED).is_operational()

    def test_incapacitated_not_operational(self) -> None:
        assert not _robot(status=STATUS_INCAPACITATED).is_operational()
