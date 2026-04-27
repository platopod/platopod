"""Tests for plato_pod.logistics — fuel/ammo/water consumption + resupply."""

from __future__ import annotations

import pytest

from plato_pod.logistics import (
    Logistics,
    can_fire,
    consume_ammo,
    consume_fuel,
    consume_water,
    is_immobile,
    logistics_from_dict,
    resupply,
)
from plato_pod.robot import Robot


def _bot(logistics: Logistics | None = None) -> Robot:
    return Robot(robot_id=1, deployment="virtual", logistics=logistics)


class TestFuel:
    def test_consume_fuel_decreases(self) -> None:
        bot = _bot(Logistics(fuel=1.0, fuel_rate_per_km=0.1))
        new = consume_fuel(bot, 1000.0)  # 1 km → 0.1 used
        assert new.logistics is not None
        assert new.logistics.fuel == pytest.approx(0.9)

    def test_no_logistics_noop(self) -> None:
        bot = _bot(None)
        assert consume_fuel(bot, 5000.0) is bot

    def test_fuel_clamped_to_zero(self) -> None:
        bot = _bot(Logistics(fuel=0.05, fuel_rate_per_km=0.1))
        new = consume_fuel(bot, 5000.0)  # would use 0.5
        assert new.logistics is not None
        assert new.logistics.fuel == 0.0

    def test_zero_distance_noop(self) -> None:
        bot = _bot(Logistics(fuel=0.5))
        assert consume_fuel(bot, 0) is bot


class TestAmmo:
    def test_consume_ammo(self) -> None:
        bot = _bot(Logistics(ammo={"M4": 30}))
        new = consume_ammo(bot, "M4", 5)
        assert new.logistics is not None
        assert new.logistics.ammo["M4"] == 25

    def test_consume_unknown_weapon(self) -> None:
        bot = _bot(Logistics(ammo={"M4": 30}))
        new = consume_ammo(bot, "PKM", 1)
        assert new.logistics is not None
        assert new.logistics.ammo["PKM"] == 0
        assert new.logistics.ammo["M4"] == 30

    def test_ammo_clamped_to_zero(self) -> None:
        bot = _bot(Logistics(ammo={"M4": 3}))
        new = consume_ammo(bot, "M4", 10)
        assert new.logistics is not None
        assert new.logistics.ammo["M4"] == 0


class TestWater:
    def test_consume_water(self) -> None:
        bot = _bot(Logistics(water=1.0))
        new = consume_water(bot, 0.3)
        assert new.logistics is not None
        assert new.logistics.water == pytest.approx(0.7)


class TestPredicates:
    def test_immobile_when_fuel_empty(self) -> None:
        bot = _bot(Logistics(fuel=0.0))
        assert is_immobile(bot)

    def test_not_immobile_with_fuel(self) -> None:
        bot = _bot(Logistics(fuel=0.5))
        assert not is_immobile(bot)

    def test_no_logistics_not_immobile(self) -> None:
        bot = _bot(None)
        assert not is_immobile(bot)

    def test_can_fire_with_ammo(self) -> None:
        bot = _bot(Logistics(ammo={"M4": 5}))
        assert can_fire(bot, "M4")

    def test_cannot_fire_without_ammo(self) -> None:
        bot = _bot(Logistics(ammo={"M4": 0}))
        assert not can_fire(bot, "M4")

    def test_no_logistics_can_fire(self) -> None:
        bot = _bot(None)
        assert can_fire(bot, "M4")  # untracked = unlimited


class TestResupply:
    def test_resupply_fuel(self) -> None:
        bot = _bot(Logistics(fuel=0.2))
        new = resupply(bot, {"fuel": 1.0})
        assert new.logistics is not None
        assert new.logistics.fuel == 1.0

    def test_resupply_ammo_additive(self) -> None:
        bot = _bot(Logistics(ammo={"M4": 5}))
        new = resupply(bot, {"ammo": {"M4": 25, "PKM": 100}})
        assert new.logistics is not None
        assert new.logistics.ammo["M4"] == 30
        assert new.logistics.ammo["PKM"] == 100

    def test_resupply_water(self) -> None:
        bot = _bot(Logistics(water=0.3))
        new = resupply(bot, {"water": 1.0})
        assert new.logistics is not None
        assert new.logistics.water == 1.0

    def test_resupply_without_logistics_noop(self) -> None:
        bot = _bot(None)
        assert resupply(bot, {"fuel": 1.0}) is bot

    def test_resupply_clamps_above_one(self) -> None:
        bot = _bot(Logistics(fuel=0.5))
        new = resupply(bot, {"fuel": 5.0})
        assert new.logistics is not None
        assert new.logistics.fuel == 1.0


class TestFromDict:
    def test_minimal(self) -> None:
        log = logistics_from_dict({})
        assert log.fuel == 1.0
        assert log.water == 1.0
        assert log.ammo == {}

    def test_full(self) -> None:
        log = logistics_from_dict({
            "fuel": 0.7,
            "water": 0.5,
            "fuel_rate_per_km": 0.08,
            "ammo": {"M4_carbine": 30, "frag": 4},
        })
        assert log.fuel == 0.7
        assert log.water == 0.5
        assert log.fuel_rate_per_km == 0.08
        assert log.ammo == {"M4_carbine": 30, "frag": 4}
