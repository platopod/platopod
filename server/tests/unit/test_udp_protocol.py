"""Tests for plato_pod.udp_protocol — UDP message parsing and formatting."""

from __future__ import annotations

import pytest

from plato_pod.udp_protocol import (
    clamp_velocity,
    format_display,
    format_heartbeat,
    format_led,
    format_move,
    format_ping,
    format_reg_response_deferred,
    format_reg_response_err_duplicate,
    format_reg_response_ok,
    format_rgb,
    format_stop,
    parse_incoming,
)


# --- parse_incoming ---

class TestParseIncoming:
    def test_parse_reg_message(self) -> None:
        result = parse_incoming(b"REG 3 28")
        assert result is not None
        cmd, params = result
        assert cmd == "REG"
        assert params["tag_id"] == 3
        assert params["radius_mm"] == 28

    def test_parse_reg_large_values(self) -> None:
        result = parse_incoming(b"REG 15 50")
        assert result is not None
        assert result[1]["tag_id"] == 15
        assert result[1]["radius_mm"] == 50

    def test_parse_ok(self) -> None:
        result = parse_incoming(b"OK")
        assert result == ("OK", {})

    def test_parse_pong(self) -> None:
        result = parse_incoming(b"PONG")
        assert result == ("PONG", {})

    def test_parse_malformed_returns_none(self) -> None:
        assert parse_incoming(b"GIBBERISH") is None

    def test_parse_reg_missing_fields(self) -> None:
        assert parse_incoming(b"REG 3") is None

    def test_parse_empty_returns_none(self) -> None:
        assert parse_incoming(b"") is None

    def test_parse_reg_non_numeric(self) -> None:
        assert parse_incoming(b"REG abc def") is None

    def test_parse_case_insensitive(self) -> None:
        result = parse_incoming(b"reg 5 28")
        assert result is not None
        assert result[0] == "REG"

    def test_parse_with_trailing_whitespace(self) -> None:
        result = parse_incoming(b"REG 3 28\r\n")
        assert result is not None
        assert result[1]["tag_id"] == 3


# --- format responses ---

class TestFormatResponses:
    def test_format_reg_ok(self) -> None:
        assert format_reg_response_ok(5) == b"OK 5"

    def test_format_reg_ok_id_1(self) -> None:
        assert format_reg_response_ok(1) == b"OK 1"

    def test_format_deferred(self) -> None:
        assert format_reg_response_deferred() == b"DEFERRED"

    def test_format_err_duplicate(self) -> None:
        assert format_reg_response_err_duplicate() == b"ERR DUPLICATE"


# --- format commands ---

class TestFormatCommands:
    def test_format_move(self) -> None:
        assert format_move(0.1, 0.5) == b"M 0.10 0.50"

    def test_format_move_negative(self) -> None:
        assert format_move(-0.15, -1.0) == b"M -0.15 -1.00"

    def test_format_stop(self) -> None:
        assert format_stop() == b"S"

    def test_format_led_on(self) -> None:
        assert format_led(True) == b"L1"

    def test_format_led_off(self) -> None:
        assert format_led(False) == b"L0"

    def test_format_rgb(self) -> None:
        assert format_rgb(255, 0, 128) == b"C 255 0 128"

    def test_format_display(self) -> None:
        assert format_display("Hello") == b"D Hello"

    def test_format_heartbeat(self) -> None:
        assert format_heartbeat() == b"H"

    def test_format_ping(self) -> None:
        assert format_ping() == b"P"


# --- clamp_velocity ---

class TestClampVelocity:
    def test_within_limits_unchanged(self) -> None:
        assert clamp_velocity(0.1, 0.5) == (0.1, 0.5)

    def test_exceeds_linear_clamped(self) -> None:
        lin, ang = clamp_velocity(0.5, 0.0)
        assert lin == pytest.approx(0.3)

    def test_negative_linear_clamped(self) -> None:
        lin, ang = clamp_velocity(-0.5, 0.0)
        assert lin == pytest.approx(-0.3)

    def test_exceeds_angular_clamped(self) -> None:
        lin, ang = clamp_velocity(0.0, 3.0)
        assert ang == pytest.approx(2.0)

    def test_negative_angular_clamped(self) -> None:
        lin, ang = clamp_velocity(0.0, -3.0)
        assert ang == pytest.approx(-2.0)

    def test_both_clamped(self) -> None:
        lin, ang = clamp_velocity(1.0, -5.0)
        assert lin == pytest.approx(0.3)
        assert ang == pytest.approx(-2.0)

    def test_custom_limits(self) -> None:
        lin, ang = clamp_velocity(0.5, 3.0, max_linear=0.2, max_angular=1.0)
        assert lin == pytest.approx(0.2)
        assert ang == pytest.approx(1.0)

    def test_zero_unchanged(self) -> None:
        assert clamp_velocity(0.0, 0.0) == (0.0, 0.0)
