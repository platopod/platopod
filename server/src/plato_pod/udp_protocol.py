"""UDP protocol for communication with ESP32 robots.

Encapsulates all message parsing and formatting for the Plato Pod UDP
protocol. No ROS2 dependency, no socket dependency — pure string/bytes
manipulation.

Protocol reference: firmware/main/main.c
Server port: 9999
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


def parse_incoming(data: bytes) -> tuple[str, dict] | None:
    """Parse an incoming UDP message from a robot.

    Args:
        data: Raw bytes received from the UDP socket.

    Returns:
        Tuple of (command_type, params) or None if unparseable.
        Command types: "REG", "OK", "PONG"
    """
    try:
        text = data.decode("utf-8").strip()
    except UnicodeDecodeError:
        return None

    if not text:
        return None

    parts = text.split()
    cmd = parts[0].upper()

    if cmd == "REG" and len(parts) >= 3:
        try:
            tag_id = int(parts[1])
            radius_mm = int(parts[2])
            return ("REG", {"tag_id": tag_id, "radius_mm": radius_mm})
        except ValueError:
            return None

    if cmd == "OK":
        return ("OK", {})

    if cmd == "PONG":
        return ("PONG", {})

    return None


def format_reg_response_ok(robot_id: int) -> bytes:
    """Format 'OK <robot_id>' registration response."""
    return f"OK {robot_id}".encode("utf-8")


def format_reg_response_deferred() -> bytes:
    """Format 'DEFERRED' registration response."""
    return b"DEFERRED"


def format_reg_response_err_duplicate() -> bytes:
    """Format 'ERR DUPLICATE' registration response."""
    return b"ERR DUPLICATE"


def format_move(linear: float, angular: float) -> bytes:
    """Format 'M <linear> <angular>' motor command."""
    return f"M {linear:.2f} {angular:.2f}".encode("utf-8")


def format_stop() -> bytes:
    """Format 'S' stop command."""
    return b"S"


def format_led(on: bool) -> bytes:
    """Format 'L1' (on) or 'L0' (off) LED command."""
    return b"L1" if on else b"L0"


def format_rgb(r: int, g: int, b: int) -> bytes:
    """Format 'C <r> <g> <b>' RGB colour command."""
    return f"C {r} {g} {b}".encode("utf-8")


def format_display(text: str) -> bytes:
    """Format 'D <text>' display command."""
    return f"D {text}".encode("utf-8")


def format_heartbeat() -> bytes:
    """Format 'H' heartbeat command."""
    return b"H"


def format_ping() -> bytes:
    """Format 'P' ping command."""
    return b"P"


def clamp_velocity(
    linear: float,
    angular: float,
    max_linear: float = 0.3,
    max_angular: float = 2.0,
) -> tuple[float, float]:
    """Clamp velocity to firmware-enforced limits.

    Args:
        linear: Desired linear velocity (m/s).
        angular: Desired angular velocity (rad/s).
        max_linear: Maximum linear speed (default 0.3 m/s).
        max_angular: Maximum angular speed (default 2.0 rad/s).

    Returns:
        Tuple of (clamped_linear, clamped_angular).
    """
    clamped_linear = max(-max_linear, min(max_linear, linear))
    clamped_angular = max(-max_angular, min(max_angular, angular))
    return (clamped_linear, clamped_angular)
