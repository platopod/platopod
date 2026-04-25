"""WebSocket JSON protocol for the Plato Pod control API.

Parses and formats JSON messages exchanged over the WebSocket connection.
No ROS2 dependency, no WebSocket library dependency — pure string manipulation.
"""

from __future__ import annotations

import json


def parse_client_message(raw: str) -> tuple[str, dict] | None:
    """Parse a JSON message from a WebSocket client.

    Args:
        raw: Raw JSON string.

    Returns:
        Tuple of (message_type, fields) or None on parse error.
    """
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None

    if not isinstance(data, dict):
        return None

    msg_type = data.get("type")
    if not isinstance(msg_type, str):
        return None

    return msg_type, data


def validate_cmd_vel(fields: dict) -> tuple[bool, str]:
    """Validate a cmd_vel message has required fields.

    Returns:
        (valid, error_message).
    """
    robot_id = fields.get("robot_id")
    if not isinstance(robot_id, int):
        return False, "robot_id must be an integer"

    linear_x = fields.get("linear_x")
    if not isinstance(linear_x, (int, float)):
        return False, "linear_x must be a number"

    angular_z = fields.get("angular_z")
    if not isinstance(angular_z, (int, float)):
        return False, "angular_z must be a number"

    return True, ""


def validate_subscribe(fields: dict) -> tuple[bool, str]:
    """Validate a subscribe/unsubscribe message.

    Returns:
        (valid, error_message).
    """
    robot_ids = fields.get("robot_ids")
    if not isinstance(robot_ids, list):
        return False, "robot_ids must be a list"
    if not all(isinstance(rid, int) for rid in robot_ids):
        return False, "robot_ids must contain only integers"
    return True, ""


def format_pose(
    robot_id: int, x: float, y: float, theta: float,
    current_linear_x: float, current_angular_z: float,
) -> str:
    """Format a pose update message."""
    return json.dumps({
        "type": "pose",
        "robot_id": robot_id,
        "x": x,
        "y": y,
        "theta": theta,
        "current_linear_x": current_linear_x,
        "current_angular_z": current_angular_z,
    })


def format_event(robot_id: int, event: str, detail: dict) -> str:
    """Format an event notification message."""
    return json.dumps({
        "type": "event",
        "robot_id": robot_id,
        "event": event,
        "detail": detail,
    })


def format_error(code: str, robot_id: int | None, message: str) -> str:
    """Format an error message."""
    msg: dict = {
        "type": "error",
        "code": code,
        "message": message,
    }
    if robot_id is not None:
        msg["robot_id"] = robot_id
    return json.dumps(msg)


def format_subscribe_ack(
    subscribed: list[int],
    failed: list[dict],
) -> str:
    """Format a subscribe acknowledgement message.

    Args:
        subscribed: List of successfully subscribed robot IDs.
        failed: List of dicts with 'robot_id' and 'reason'.
    """
    return json.dumps({
        "type": "subscribe_ack",
        "subscribed": subscribed,
        "failed": failed,
    })


# --- Sensor messages ---

def validate_subscribe_sensors(fields: dict) -> tuple[bool, str]:
    """Validate a subscribe_sensors message."""
    if not isinstance(fields.get("robot_id"), int):
        return False, "robot_id must be an integer"
    sensors = fields.get("sensors")
    if not isinstance(sensors, list) or not all(isinstance(s, str) for s in sensors):
        return False, "sensors must be a list of strings"
    return True, ""


def validate_configure_sensor(fields: dict) -> tuple[bool, str]:
    """Validate a configure_sensor message."""
    if not isinstance(fields.get("robot_id"), int):
        return False, "robot_id must be an integer"
    if not isinstance(fields.get("sensor"), str):
        return False, "sensor must be a string"
    if not isinstance(fields.get("config"), dict):
        return False, "config must be a dict"
    return True, ""


def validate_apply_preset(fields: dict) -> tuple[bool, str]:
    """Validate an apply_sensor_preset message."""
    if not isinstance(fields.get("robot_id"), int):
        return False, "robot_id must be an integer"
    if not isinstance(fields.get("preset"), str):
        return False, "preset must be a string"
    return True, ""


def format_sensor_data(
    robot_id: int, sensor_name: str, timestamp: float, data: dict
) -> str:
    """Format a continuous sensor data message."""
    return json.dumps({
        "type": "sensor",
        "robot_id": robot_id,
        "sensor": sensor_name,
        "timestamp": timestamp,
        "data": data,
    })


def format_sensor_event(
    robot_id: int, sensor_name: str, timestamp: float, data: dict
) -> str:
    """Format an event-driven sensor notification."""
    return json.dumps({
        "type": "sensor_event",
        "robot_id": robot_id,
        "sensor": sensor_name,
        "timestamp": timestamp,
        "data": data,
    })


def format_subscribe_sensors_ack(
    robot_id: int,
    subscribed: list[str],
    failed: list[dict],
) -> str:
    """Format a sensor subscribe acknowledgement."""
    return json.dumps({
        "type": "subscribe_sensors_ack",
        "robot_id": robot_id,
        "subscribed": subscribed,
        "failed": failed,
    })


def format_configure_sensor_ack(
    robot_id: int, sensor_name: str, status: str
) -> str:
    """Format a sensor configuration acknowledgement."""
    return json.dumps({
        "type": "configure_sensor_ack",
        "robot_id": robot_id,
        "sensor": sensor_name,
        "status": status,
    })


# --- Event injection messages ---

VALID_INJECT_EVENTS = {
    "place_gas_source",
    "update_wind",
    "place_obstacle",
    "trigger_strike",
    "update_field",
    "reset_exercise",
}


def validate_inject_event(fields: dict) -> tuple[bool, str]:
    """Validate an inject_event message.

    Required fields:
        event_type (str): One of the VALID_INJECT_EVENTS.
        admin_token (str): Must match server admin token.

    Optional fields:
        data (dict): Event-specific parameters.
    """
    if not isinstance(fields.get("event_type"), str):
        return False, "event_type must be a string"
    if fields["event_type"] not in VALID_INJECT_EVENTS:
        return False, f"unknown event_type: {fields['event_type']}"
    if not isinstance(fields.get("admin_token"), str):
        return False, "admin_token must be a string"
    data = fields.get("data")
    if data is not None and not isinstance(data, dict):
        return False, "data must be a dict if provided"
    return True, ""


def format_inject_event_ack(event_type: str, status: str, detail: str = "") -> str:
    """Format an event injection acknowledgement."""
    msg: dict = {
        "type": "inject_event_ack",
        "event_type": event_type,
        "status": status,
    }
    if detail:
        msg["detail"] = detail
    return json.dumps(msg)


def format_environment_update(update_type: str, data: dict) -> str:
    """Format an environment update broadcast to all clients."""
    return json.dumps({
        "type": "environment_update",
        "update_type": update_type,
        "data": data,
    })
