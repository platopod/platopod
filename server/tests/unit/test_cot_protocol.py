"""Tests for plato_pod.cot_protocol — CoT XML generation and parsing."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from plato_pod.cot_protocol import (
    VEHICLE_ROLE_TO_COT_TYPE,
    make_contact_detail,
    make_cot_event,
    make_sensor_detail,
    make_shape_event,
    make_track_detail,
    parse_cot_event,
    parse_nav_goal,
    robot_id_from_uid,
)


class TestMakeCotEvent:
    def test_produces_valid_xml(self) -> None:
        xml = make_cot_event("test-1", "a-f-G", -35.3, 149.1)
        root = ET.fromstring(xml)
        assert root.tag == "event"

    def test_required_attributes(self) -> None:
        xml = make_cot_event("uid-1", "a-f-G", 0.0, 0.0)
        root = ET.fromstring(xml)
        assert root.get("version") == "2.0"
        assert root.get("uid") == "uid-1"
        assert root.get("type") == "a-f-G"
        assert root.get("how") == "m-g"
        assert root.get("time") is not None
        assert root.get("start") is not None
        assert root.get("stale") is not None

    def test_point_element(self) -> None:
        xml = make_cot_event("uid-1", "a-f-G", -35.2975, 149.1012, hae=580.0, ce=5.0, le=3.0)
        root = ET.fromstring(xml)
        point = root.find("point")
        assert point is not None
        assert float(point.get("lat")) == pytest.approx(-35.2975, abs=0.001)
        assert float(point.get("lon")) == pytest.approx(149.1012, abs=0.001)
        assert float(point.get("hae")) == pytest.approx(580.0)
        assert float(point.get("ce")) == pytest.approx(5.0)
        assert float(point.get("le")) == pytest.approx(3.0)

    def test_stale_after_start(self) -> None:
        xml = make_cot_event("uid-1", "a-f-G", 0.0, 0.0, stale_seconds=60.0)
        root = ET.fromstring(xml)
        start = root.get("start")
        stale = root.get("stale")
        assert start is not None and stale is not None
        assert stale > start

    def test_detail_xml_included(self) -> None:
        detail = '<contact callsign="BLUE-01"/>'
        xml = make_cot_event("uid-1", "a-f-G", 0.0, 0.0, detail_xml=detail)
        root = ET.fromstring(xml)
        contact = root.find(".//contact")
        assert contact is not None
        assert contact.get("callsign") == "BLUE-01"

    def test_timestamp_format(self) -> None:
        xml = make_cot_event("uid-1", "a-f-G", 0.0, 0.0)
        root = ET.fromstring(xml)
        time_str = root.get("time")
        assert time_str.endswith("Z")
        assert "T" in time_str
        assert len(time_str) == 24  # "2026-04-24T10:30:00.000Z"


class TestMakeContactDetail:
    def test_produces_valid_xml(self) -> None:
        xml = make_contact_detail("BLUE-01")
        root = ET.fromstring(xml)
        assert root.tag == "contact"
        assert root.get("callsign") == "BLUE-01"


class TestMakeTrackDetail:
    def test_produces_valid_xml(self) -> None:
        xml = make_track_detail(45.0, 1.5)
        root = ET.fromstring(xml)
        assert root.tag == "track"
        assert float(root.get("course")) == pytest.approx(45.0)
        assert float(root.get("speed")) == pytest.approx(1.5)


class TestMakeSensorDetail:
    def test_produces_valid_xml(self) -> None:
        xml = make_sensor_detail({"acetone": 420.0, "ethanol": 0.5}, model_name="test")
        root = ET.fromstring(xml)
        assert root.tag == "sensor"
        assert root.get("model") == "test"
        readings = root.findall("reading")
        assert len(readings) == 2


class TestMakeShapeEvent:
    def test_produces_valid_xml(self) -> None:
        points = [(-35.297, 149.101), (-35.298, 149.101), (-35.298, 149.102)]
        xml = make_shape_event("arena-1", points, label="Arena")
        root = ET.fromstring(xml)
        assert root.tag == "event"
        assert root.get("type") == "u-d-f"

    def test_correct_vertex_count(self) -> None:
        points = [(-35.0, 149.0), (-35.0, 149.1), (-35.1, 149.1), (-35.1, 149.0)]
        xml = make_shape_event("arena-1", points)
        root = ET.fromstring(xml)
        links = root.findall(".//link")
        assert len(links) == 4


class TestParseCotEvent:
    def test_round_trip(self) -> None:
        xml = make_cot_event("test-rt", "a-f-G", -35.3, 149.1, hae=100.0)
        result = parse_cot_event(xml)
        assert result is not None
        assert result["uid"] == "test-rt"
        assert result["type"] == "a-f-G"
        assert result["lat"] == pytest.approx(-35.3, abs=0.001)
        assert result["lon"] == pytest.approx(149.1, abs=0.001)

    def test_malformed_xml_returns_none(self) -> None:
        assert parse_cot_event("not xml at all") is None

    def test_wrong_root_returns_none(self) -> None:
        assert parse_cot_event("<notanevent/>") is None

    def test_missing_point_returns_none(self) -> None:
        xml = '<event version="2.0" uid="x" type="a" time="t" start="s" stale="l" how="m"/>'
        assert parse_cot_event(xml) is None


class TestParseNavGoal:
    def test_waypoint_type(self) -> None:
        xml = make_cot_event("wp-1", "b-m-p-w", -35.3, 149.1)
        parsed = parse_cot_event(xml)
        goal = parse_nav_goal(parsed)
        assert goal is not None
        assert goal[0] == pytest.approx(-35.3, abs=0.001)

    def test_poi_type(self) -> None:
        xml = make_cot_event("poi-1", "b-m-p-s-p-i", -35.3, 149.1)
        parsed = parse_cot_event(xml)
        goal = parse_nav_goal(parsed)
        assert goal is not None

    def test_non_waypoint_returns_none(self) -> None:
        xml = make_cot_event("track-1", "a-f-G", -35.3, 149.1)
        parsed = parse_cot_event(xml)
        assert parse_nav_goal(parsed) is None


class TestVehicleRoleMapping:
    def test_all_roles_defined(self) -> None:
        expected = {"default", "tank", "apc", "recon", "cbrn_recon",
                    "artillery", "hostile", "hostile_tank", "unknown", "sensor"}
        assert set(VEHICLE_ROLE_TO_COT_TYPE.keys()) == expected

    def test_friendly_prefix(self) -> None:
        for role in ["default", "tank", "apc", "recon"]:
            assert VEHICLE_ROLE_TO_COT_TYPE[role].startswith("a-f-")

    def test_hostile_prefix(self) -> None:
        for role in ["hostile", "hostile_tank"]:
            assert VEHICLE_ROLE_TO_COT_TYPE[role].startswith("a-h-")


class TestRobotIdFromUid:
    def test_valid_uid(self) -> None:
        assert robot_id_from_uid("platopod-1") == 1
        assert robot_id_from_uid("platopod-42") == 42
        assert robot_id_from_uid("platopod-0") == 0

    def test_unknown_uid_returns_negative(self) -> None:
        assert robot_id_from_uid("atak-device-abc") == -1
        assert robot_id_from_uid("") == -1
        assert robot_id_from_uid("platopod-") == -1

    def test_non_integer_suffix(self) -> None:
        assert robot_id_from_uid("platopod-abc") == -1

    def test_prefix_only(self) -> None:
        assert robot_id_from_uid("platopod") == -1
