"""Tests for plato_pod.engagement — weapon engagement evaluation."""

from __future__ import annotations

from random import Random

import pytest

from plato_pod.engagement import (
    Civilian,
    EngagementOutcome,
    WeaponSpec,
    evaluate_fire,
    weapon_from_dict,
)
from plato_pod.line_of_sight import CoverPolygon
from plato_pod.robot import Robot
from plato_pod.weather import WeatherState


def _robot(robot_id: int = 1, x: float = 0.0, y: float = 0.0,
           health: float = 1.0) -> Robot:
    return Robot(robot_id=robot_id, deployment="virtual", x=x, y=y, health=health)


def _m4() -> WeaponSpec:
    return WeaponSpec(
        name="M4_carbine",
        max_range_m=400.0,
        base_pok_at_100m=0.6,
        falloff_per_100m=0.15,
        damage=1.0,
        ammo_capacity=30,
    )


def _artillery() -> WeaponSpec:
    return WeaponSpec(
        name="105mm",
        max_range_m=10000.0,
        base_pok_at_100m=0.8,
        falloff_per_100m=0.0,
        damage=1.0,
        ammo_capacity=10,
        suppress_radius_m=20.0,
    )


class TestRangeAndPoK:
    def test_short_range_high_pok(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 50, 0)
        # PoK at 50m (treated as <=100m): 0.6
        # Use rng that always rolls below 0.6 to force a hit
        outcome = evaluate_fire(actor, target, _m4(), rng=Random(0))
        # Random(0).random() = 0.844… so will miss with p=0.6
        # Use seed that produces low value
        rng = Random()
        rng.seed(123)
        # Just check effective_pok value, not the specific roll
        assert 0.5 < outcome.effective_pok <= 0.6 or outcome.effective_pok == 0.6

    def test_falloff_with_distance(self) -> None:
        actor = _robot(1, 0, 0)
        target_close = _robot(2, 100, 0)
        target_far = _robot(3, 300, 0)
        weapon = _m4()
        out_close = evaluate_fire(actor, target_close, weapon, rng=Random(0))
        out_far = evaluate_fire(actor, target_far, weapon, rng=Random(0))
        assert out_close.effective_pok > out_far.effective_pok

    def test_beyond_max_range(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 500, 0)  # beyond M4's 400m max
        outcome = evaluate_fire(actor, target, _m4(), rng=Random(0))
        assert not outcome.hit
        assert outcome.blocked
        assert outcome.rationale == "out_of_range"

    def test_distance_recorded(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 200, 0)
        outcome = evaluate_fire(actor, target, _m4(), rng=Random(0))
        assert outcome.distance_m == pytest.approx(200.0)


class TestCoverBlocking:
    def test_full_cover_blocks(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 100, 0)
        cover = CoverPolygon(
            vertices=[(40, -5), (60, -5), (60, 5), (40, 5)],
            cover_value=1.0,
            label="wall",
        )
        outcome = evaluate_fire(actor, target, _m4(), cover_polygons=[cover], rng=Random(0))
        assert not outcome.hit
        assert outcome.blocked
        assert "wall" in outcome.rationale

    def test_partial_cover_reduces_pok(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 100, 0)
        cover = CoverPolygon(
            vertices=[(40, -5), (60, -5), (60, 5), (40, 5)],
            cover_value=0.5,
        )
        out_no_cover = evaluate_fire(actor, target, _m4(), rng=Random(0))
        out_cover = evaluate_fire(actor, target, _m4(),
                                   cover_polygons=[cover], rng=Random(0))
        assert out_cover.effective_pok < out_no_cover.effective_pok


class TestHealthEffects:
    def test_wounded_actor_lower_pok(self) -> None:
        actor_full = _robot(1, 0, 0, health=1.0)
        actor_wounded = _robot(1, 0, 0, health=0.3)
        target = _robot(2, 100, 0)
        out_full = evaluate_fire(actor_full, target, _m4(), rng=Random(0))
        out_wounded = evaluate_fire(actor_wounded, target, _m4(), rng=Random(0))
        assert out_wounded.effective_pok < out_full.effective_pok


class TestROECivilian:
    def test_no_civilians_no_violation(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 100, 0)
        outcome = evaluate_fire(actor, target, _m4(), rng=Random(0))
        assert not outcome.civilian_violation

    def test_civilian_near_target_flagged(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 100, 0)
        civ = Civilian(position=(105, 0), label="bystander")
        outcome = evaluate_fire(actor, target, _m4(), civilians=[civ],
                                 civilian_proximity_m=10.0, rng=Random(0))
        assert outcome.civilian_violation
        assert "bystander" in outcome.civilians_at_risk

    def test_civilian_far_from_target_no_flag(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 100, 0)
        civ = Civilian(position=(200, 200), label="far_away")
        outcome = evaluate_fire(actor, target, _m4(), civilians=[civ],
                                 civilian_proximity_m=10.0, rng=Random(0))
        assert not outcome.civilian_violation


class TestIndirectFire:
    def test_no_actor_artillery(self) -> None:
        # Indirect fire: no actor, just target position
        target = _robot(2, 50, 50)
        outcome = evaluate_fire(None, target, _artillery(), rng=Random(0))
        # PoK at 100m: 0.8 (treated as standard)
        assert outcome.effective_pok > 0.5

    def test_artillery_area_effect(self) -> None:
        target = _robot(2, 50, 50)
        nearby = _robot(3, 55, 55)  # within 20m suppress radius (~7m)
        far = _robot(4, 200, 200)
        outcome = evaluate_fire(
            None, target, _artillery(),
            other_units=[nearby, far],
            rng=Random(0),
        )
        assert 3 in outcome.suppressed_targets
        assert 4 not in outcome.suppressed_targets


class TestDeterminism:
    def test_same_seed_same_outcome(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 100, 0)
        out1 = evaluate_fire(actor, target, _m4(), rng=Random(42))
        out2 = evaluate_fire(actor, target, _m4(), rng=Random(42))
        assert out1.hit == out2.hit
        assert out1.damage == out2.damage


class TestWeaponFromDict:
    def test_full_dict(self) -> None:
        spec = weapon_from_dict("PKM", {
            "max_range_m": 800,
            "base_pok_at_100m": 0.5,
            "falloff_per_100m": 0.08,
            "damage": 1.0,
            "ammo_capacity": 100,
            "suppress_radius_m": 5.0,
        })
        assert spec.name == "PKM"
        assert spec.max_range_m == 800.0
        assert spec.suppress_radius_m == 5.0

    def test_minimal_dict_uses_defaults(self) -> None:
        spec = weapon_from_dict("X", {})
        assert spec.max_range_m == 100.0
        assert spec.damage == 1.0


class TestOutcomeFields:
    def test_fired_always_true_when_evaluated(self) -> None:
        actor = _robot(1, 0, 0)
        target = _robot(2, 100, 0)
        outcome = evaluate_fire(actor, target, _m4(), rng=Random(0))
        assert outcome.fired is True
        assert isinstance(outcome, EngagementOutcome)
