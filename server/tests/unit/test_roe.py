"""Tests for plato_pod.roe — rules of engagement evaluation."""

from __future__ import annotations

from plato_pod.engagement import Civilian
from plato_pod.robot import Robot
from plato_pod.roe import (
    ROERules,
    ROEViolation,
    WEAPONS_FREE,
    WEAPONS_HOLD,
    WEAPONS_TIGHT,
    check_fire_roe,
    is_blocking,
    roe_from_dict,
)


def _bot(rid: int, x: float = 0.0, y: float = 0.0,
         team: str | None = "blue") -> Robot:
    return Robot(robot_id=rid, deployment="virtual", x=x, y=y, team=team)


class TestPermissionLevels:
    def test_weapons_free_no_violation(self) -> None:
        actor = _bot(1, team="blue")
        target = _bot(2, x=100, team="red")
        v = check_fire_roe(actor, (100, 0), ROERules(), target=target)
        assert v == []

    def test_weapons_hold_blocks(self) -> None:
        actor = _bot(1, team="blue")
        target = _bot(2, x=100, team="red")
        v = check_fire_roe(
            actor, (100, 0),
            ROERules(fire_permission=WEAPONS_HOLD),
            target=target,
        )
        assert any(x.rule == "weapons_hold" for x in v)
        assert is_blocking(v)

    def test_weapons_tight_requires_clearance(self) -> None:
        actor = _bot(1, team="blue")
        target = _bot(2, x=100, team="red")
        v = check_fire_roe(
            actor, (100, 0),
            ROERules(fire_permission=WEAPONS_TIGHT),
            target=target, cleared_targets=set(),
        )
        assert any(x.rule == "unconfirmed_target" for x in v)

    def test_weapons_tight_with_clearance_ok(self) -> None:
        actor = _bot(1, team="blue")
        target = _bot(2, x=100, team="red")
        v = check_fire_roe(
            actor, (100, 0),
            ROERules(fire_permission=WEAPONS_TIGHT),
            target=target, cleared_targets={2},
        )
        assert v == []


class TestFriendlyFire:
    def test_same_team_flagged(self) -> None:
        actor = _bot(1, team="blue")
        friend = _bot(2, x=100, team="blue")
        v = check_fire_roe(actor, (100, 0), ROERules(), target=friend)
        assert any(x.rule == "friendly_fire" for x in v)

    def test_different_team_ok(self) -> None:
        actor = _bot(1, team="blue")
        enemy = _bot(2, x=100, team="red")
        v = check_fire_roe(actor, (100, 0), ROERules(), target=enemy)
        assert not any(x.rule == "friendly_fire" for x in v)

    def test_critical_severity_blocks(self) -> None:
        actor = _bot(1, team="blue")
        friend = _bot(2, x=100, team="blue")
        v = check_fire_roe(
            actor, (100, 0),
            ROERules(friendly_fire_severity="critical"),
            target=friend,
        )
        assert is_blocking(v)


class TestCivilianProximity:
    def test_civilian_close_flagged(self) -> None:
        actor = _bot(1, team="blue")
        target = _bot(2, x=100, team="red")
        civs = [Civilian(position=(105, 0), label="bystander")]
        v = check_fire_roe(actor, (100, 0), ROERules(), target=target,
                            civilians=civs)
        assert any(x.rule == "civilian_proximity" for x in v)
        assert any("bystander" in x.description for x in v)

    def test_civilian_far_no_flag(self) -> None:
        actor = _bot(1, team="blue")
        target = _bot(2, x=100, team="red")
        civs = [Civilian(position=(500, 500))]
        v = check_fire_roe(actor, (100, 0), ROERules(), target=target,
                            civilians=civs)
        assert not any(x.rule == "civilian_proximity" for x in v)

    def test_no_civilians_no_flag(self) -> None:
        actor = _bot(1, team="blue")
        target = _bot(2, x=100, team="red")
        v = check_fire_roe(actor, (100, 0), ROERules(), target=target)
        assert not any(x.rule == "civilian_proximity" for x in v)


class TestNoActor:
    def test_indirect_fire_skips_friendly_check(self) -> None:
        # Actor=None (indirect fire / artillery)
        v = check_fire_roe(None, (100, 0), ROERules(), target=None)
        assert v == []


class TestIsBlocking:
    def test_warning_only_not_blocking(self) -> None:
        v = [ROEViolation("a", "warning", "x")]
        assert not is_blocking(v)

    def test_violation_only_not_blocking(self) -> None:
        v = [ROEViolation("a", "violation", "x")]
        assert not is_blocking(v)

    def test_critical_blocks(self) -> None:
        v = [ROEViolation("a", "critical", "x"),
             ROEViolation("b", "warning", "y")]
        assert is_blocking(v)


class TestFromDict:
    def test_minimal(self) -> None:
        r = roe_from_dict({})
        assert r.fire_permission == WEAPONS_FREE

    def test_full(self) -> None:
        r = roe_from_dict({
            "fire_permission": "weapons_tight",
            "civilian_proximity_m": 25,
            "require_target_id": True,
            "friendly_fire": "critical",
        })
        assert r.fire_permission == "weapons_tight"
        assert r.civilian_proximity_m == 25.0
        assert r.require_target_id
        assert r.friendly_fire_severity == "critical"
