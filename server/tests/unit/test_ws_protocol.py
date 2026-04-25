"""Tests for plato_pod.ws_protocol — WebSocket JSON message parsing/formatting."""

from __future__ import annotations

import json

from plato_pod.ws_protocol import (
    format_environment_update,
    format_error,
    format_event,
    format_inject_event_ack,
    format_pose,
    format_subscribe_ack,
    parse_client_message,
    validate_cmd_vel,
    validate_inject_event,
    validate_subscribe,
)


# --- parse_client_message ---

class TestParseClientMessage:
    def test_valid_cmd_vel(self) -> None:
        raw = json.dumps({"type": "cmd_vel", "robot_id": 1, "linear_x": 0.1, "angular_z": 0.5})
        result = parse_client_message(raw)
        assert result is not None
        msg_type, fields = result
        assert msg_type == "cmd_vel"
        assert fields["robot_id"] == 1

    def test_valid_subscribe(self) -> None:
        raw = json.dumps({"type": "subscribe", "robot_ids": [1, 3]})
        result = parse_client_message(raw)
        assert result is not None
        assert result[0] == "subscribe"

    def test_valid_unsubscribe(self) -> None:
        raw = json.dumps({"type": "unsubscribe", "robot_ids": [1]})
        result = parse_client_message(raw)
        assert result is not None
        assert result[0] == "unsubscribe"

    def test_invalid_json_returns_none(self) -> None:
        assert parse_client_message("not json") is None

    def test_not_dict_returns_none(self) -> None:
        assert parse_client_message('"just a string"') is None

    def test_missing_type_returns_none(self) -> None:
        assert parse_client_message('{"robot_id": 1}') is None

    def test_type_not_string_returns_none(self) -> None:
        assert parse_client_message('{"type": 123}') is None


# --- validate_cmd_vel ---

class TestValidateCmdVel:
    def test_valid(self) -> None:
        ok, _ = validate_cmd_vel({"robot_id": 1, "linear_x": 0.1, "angular_z": 0.5})
        assert ok is True

    def test_integer_velocities_valid(self) -> None:
        ok, _ = validate_cmd_vel({"robot_id": 1, "linear_x": 0, "angular_z": 0})
        assert ok is True

    def test_missing_robot_id(self) -> None:
        ok, msg = validate_cmd_vel({"linear_x": 0.1, "angular_z": 0.5})
        assert ok is False
        assert "robot_id" in msg

    def test_robot_id_not_int(self) -> None:
        ok, _ = validate_cmd_vel({"robot_id": "one", "linear_x": 0.1, "angular_z": 0.5})
        assert ok is False

    def test_missing_linear_x(self) -> None:
        ok, _ = validate_cmd_vel({"robot_id": 1, "angular_z": 0.5})
        assert ok is False

    def test_missing_angular_z(self) -> None:
        ok, _ = validate_cmd_vel({"robot_id": 1, "linear_x": 0.1})
        assert ok is False

    def test_velocity_not_number(self) -> None:
        ok, _ = validate_cmd_vel({"robot_id": 1, "linear_x": "fast", "angular_z": 0})
        assert ok is False


# --- validate_subscribe ---

class TestValidateSubscribe:
    def test_valid(self) -> None:
        ok, _ = validate_subscribe({"robot_ids": [1, 3, 5]})
        assert ok is True

    def test_empty_list_valid(self) -> None:
        ok, _ = validate_subscribe({"robot_ids": []})
        assert ok is True

    def test_missing_robot_ids(self) -> None:
        ok, _ = validate_subscribe({})
        assert ok is False

    def test_not_list(self) -> None:
        ok, _ = validate_subscribe({"robot_ids": 1})
        assert ok is False

    def test_non_int_in_list(self) -> None:
        ok, _ = validate_subscribe({"robot_ids": [1, "two"]})
        assert ok is False


# --- format functions ---

class TestFormatPose:
    def test_format_and_parse(self) -> None:
        raw = format_pose(1, 0.25, 0.31, 1.62, 0.1, 0.0)
        data = json.loads(raw)
        assert data["type"] == "pose"
        assert data["robot_id"] == 1
        assert data["x"] == 0.25
        assert data["current_linear_x"] == 0.1


class TestFormatEvent:
    def test_boundary_contact(self) -> None:
        raw = format_event(1, "boundary_contact", {"element": "perimeter", "clamped": True})
        data = json.loads(raw)
        assert data["type"] == "event"
        assert data["event"] == "boundary_contact"
        assert data["detail"]["element"] == "perimeter"


class TestFormatError:
    def test_with_robot_id(self) -> None:
        raw = format_error("robot_busy", 1, "Robot is controlled by another client")
        data = json.loads(raw)
        assert data["type"] == "error"
        assert data["code"] == "robot_busy"
        assert data["robot_id"] == 1

    def test_without_robot_id(self) -> None:
        raw = format_error("invalid_command", None, "Bad message")
        data = json.loads(raw)
        assert "robot_id" not in data


class TestFormatSubscribeAck:
    def test_mixed_results(self) -> None:
        raw = format_subscribe_ack([1, 3], [{"robot_id": 99, "reason": "not_found"}])
        data = json.loads(raw)
        assert data["type"] == "subscribe_ack"
        assert data["subscribed"] == [1, 3]
        assert len(data["failed"]) == 1
        assert data["failed"][0]["robot_id"] == 99


# --- Event injection ---

class TestValidateInjectEvent:
    def test_valid_place_gas_source(self) -> None:
        fields = {
            "event_type": "place_gas_source",
            "admin_token": "secret",
            "data": {"x": 0.5, "y": 0.3},
        }
        valid, err = validate_inject_event(fields)
        assert valid is True

    def test_missing_event_type(self) -> None:
        valid, err = validate_inject_event({"admin_token": "x"})
        assert valid is False
        assert "event_type" in err

    def test_unknown_event_type(self) -> None:
        valid, err = validate_inject_event({
            "event_type": "launch_missile",
            "admin_token": "x",
        })
        assert valid is False
        assert "unknown" in err

    def test_missing_admin_token(self) -> None:
        valid, err = validate_inject_event({
            "event_type": "place_gas_source",
        })
        assert valid is False
        assert "admin_token" in err

    def test_data_must_be_dict(self) -> None:
        valid, err = validate_inject_event({
            "event_type": "update_wind",
            "admin_token": "x",
            "data": "not a dict",
        })
        assert valid is False

    def test_data_optional(self) -> None:
        valid, err = validate_inject_event({
            "event_type": "reset_exercise",
            "admin_token": "x",
        })
        assert valid is True

    def test_all_valid_event_types(self) -> None:
        for event_type in [
            "place_gas_source", "update_wind", "place_obstacle",
            "trigger_strike", "update_field", "reset_exercise",
        ]:
            valid, _ = validate_inject_event({
                "event_type": event_type,
                "admin_token": "x",
            })
            assert valid is True, f"{event_type} should be valid"


class TestFormatInjectEventAck:
    def test_success(self) -> None:
        raw = format_inject_event_ack("place_gas_source", "ok")
        data = json.loads(raw)
        assert data["type"] == "inject_event_ack"
        assert data["event_type"] == "place_gas_source"
        assert data["status"] == "ok"
        assert "detail" not in data

    def test_with_detail(self) -> None:
        raw = format_inject_event_ack("update_wind", "error", "wind too strong")
        data = json.loads(raw)
        assert data["detail"] == "wind too strong"


class TestFormatEnvironmentUpdate:
    def test_broadcast(self) -> None:
        raw = format_environment_update("place_gas_source", {"x": 0.5, "y": 0.3})
        data = json.loads(raw)
        assert data["type"] == "environment_update"
        assert data["update_type"] == "place_gas_source"
        assert data["data"]["x"] == 0.5
