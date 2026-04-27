"""Tests for plato_pod.comms — communications state evaluation."""

from __future__ import annotations

import pytest

from plato_pod.comms import (
    CommsConfig,
    DeadZonePolygon,
    JammingZone,
    comms_config_from_dict,
    dead_zone_from_dict,
    evaluate_comms,
    jamming_zone_from_dict,
)
from plato_pod.line_of_sight import CoverPolygon
from plato_pod.robot import Robot


def _bot(rid: int, x: float = 0.0, y: float = 0.0,
         status: str = "active") -> Robot:
    return Robot(robot_id=rid, deployment="virtual", x=x, y=y,
                  team="blue", status=status)


class TestBasicLink:
    def test_close_teammate_links(self) -> None:
        a = _bot(1, 0, 0)
        b = _bot(2, 100, 0)
        s = evaluate_comms(a, [b], CommsConfig(max_range_m=1000))
        assert s.linked
        assert s.relay_id == 2
        assert 0 < s.quality <= 1

    def test_no_teammates(self) -> None:
        a = _bot(1, 0, 0)
        s = evaluate_comms(a, [], CommsConfig())
        assert not s.linked
        assert "no_reachable_teammate" in s.rationale

    def test_self_excluded(self) -> None:
        a = _bot(1, 0, 0)
        s = evaluate_comms(a, [a], CommsConfig())
        assert not s.linked

    def test_destroyed_teammate_excluded(self) -> None:
        a = _bot(1, 0, 0)
        b = _bot(2, 50, 0, status="destroyed")
        s = evaluate_comms(a, [b], CommsConfig())
        assert not s.linked


class TestRange:
    def test_quality_decreases_with_distance(self) -> None:
        a = _bot(1, 0, 0)
        near = _bot(2, 100, 0)
        far = _bot(3, 800, 0)
        cfg = CommsConfig(max_range_m=1000)
        q_near = evaluate_comms(a, [near], cfg).quality
        q_far = evaluate_comms(a, [far], cfg).quality
        assert q_near > q_far

    def test_beyond_range_no_link(self) -> None:
        a = _bot(1, 0, 0)
        b = _bot(2, 2000, 0)
        s = evaluate_comms(a, [b], CommsConfig(max_range_m=1000))
        assert not s.linked

    def test_picks_best_relay(self) -> None:
        a = _bot(1, 0, 0)
        far = _bot(2, 800, 0)
        near = _bot(3, 100, 0)
        s = evaluate_comms(a, [far, near], CommsConfig(max_range_m=1000))
        assert s.relay_id == 3   # higher quality


class TestCoverAndLos:
    def test_cover_blocks_link(self) -> None:
        a = _bot(1, 0, 0)
        b = _bot(2, 200, 0)
        cover = [CoverPolygon(
            vertices=[(80, -50), (120, -50), (120, 50), (80, 50)],
            cover_value=1.0, label="ridge",
        )]
        s = evaluate_comms(a, [b], CommsConfig(max_range_m=1000),
                            cover_polygons=cover)
        assert not s.linked

    def test_disable_los_check(self) -> None:
        a = _bot(1, 0, 0)
        b = _bot(2, 200, 0)
        cover = [CoverPolygon(
            vertices=[(80, -50), (120, -50), (120, 50), (80, 50)],
            cover_value=1.0,
        )]
        s = evaluate_comms(
            a, [b], CommsConfig(max_range_m=1000, require_los=False),
            cover_polygons=cover,
        )
        assert s.linked


class TestJamming:
    def test_jammed_unit_cannot_communicate(self) -> None:
        a = _bot(1, 50, 0)
        b = _bot(2, 200, 0)
        jammers = [JammingZone(position=(50.0, 0.0), radius_m=20.0,
                                strength=1.0, label="X")]
        s = evaluate_comms(a, [b], CommsConfig(),
                            jamming_zones=jammers)
        assert not s.linked
        assert "jammed:X" in s.rationale

    def test_partial_jamming_degrades_quality(self) -> None:
        a = _bot(1, 0, 0)
        b = _bot(2, 100, 0)
        no_jam = evaluate_comms(a, [b], CommsConfig())
        jammers = [JammingZone(position=(0.0, 0.0), radius_m=50.0,
                                strength=0.5)]
        with_jam = evaluate_comms(a, [b], CommsConfig(),
                                   jamming_zones=jammers)
        assert with_jam.quality < no_jam.quality
        assert with_jam.linked   # still linked but degraded


class TestDeadZones:
    def test_dead_zone_blocks(self) -> None:
        a = _bot(1, 50, 50)
        b = _bot(2, 200, 200)
        dz = [DeadZonePolygon(
            vertices=[(0, 0), (100, 0), (100, 100), (0, 100)],
            label="ridge_shadow",
        )]
        s = evaluate_comms(a, [b], CommsConfig(), dead_zones=dz)
        assert not s.linked
        assert "dead_zone:ridge_shadow" in s.rationale


class TestFromDict:
    def test_jamming_from_dict(self) -> None:
        z = jamming_zone_from_dict({
            "position": [10.0, 20.0],
            "radius_m": 50,
            "strength": 0.8,
            "label": "EW1",
        })
        assert z.position == (10.0, 20.0)
        assert z.strength == pytest.approx(0.8)
        assert z.label == "EW1"

    def test_dead_zone_from_dict(self) -> None:
        z = dead_zone_from_dict({
            "vertices": [[0, 0], [10, 0], [10, 10], [0, 10]],
            "label": "valley",
        })
        assert len(z.vertices) == 4
        assert z.label == "valley"

    def test_comms_config_from_dict(self) -> None:
        c = comms_config_from_dict({"max_range_m": 500})
        assert c.max_range_m == 500.0
