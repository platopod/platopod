"""API Gateway server — FastAPI application for REST and WebSocket endpoints.

Contains no ROS2 imports. Receives callback functions from the ROS2 node
for operations that need the ROS2 graph (publishing cmd_vel, calling services).
"""

from __future__ import annotations

import json
import logging
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable

import urllib.request

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from starlette.websockets import WebSocketState

from plato_pod.command_pipeline import PipelineEvent, run_pipeline
from plato_pod.robot import Robot
from plato_pod.control_manager import ControlManager
from plato_pod.ws_protocol import (
    format_configure_sensor_ack,
    format_environment_update,
    format_error,
    format_event,
    format_inject_event_ack,
    format_pose,
    format_subscribe_ack,
    format_subscribe_sensors_ack,
    parse_client_message,
    validate_apply_preset,
    validate_cmd_vel,
    validate_configure_sensor,
    validate_fire_weapon,
    validate_inject_event,
    validate_report_observation,
    validate_subscribe,
    validate_subscribe_sensors,
)

logger = logging.getLogger(__name__)


@dataclass
class RobotStatusStore:
    """Thread-safe cache of latest robot statuses."""
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _robots: dict[int, dict] = field(default_factory=dict, repr=False)

    def update(self, robots: list[dict]) -> None:
        with self._lock:
            self._robots = {r["robot_id"]: r for r in robots}

    def get_all(self) -> list[dict]:
        with self._lock:
            return list(self._robots.values())

    def get(self, robot_id: int) -> dict | None:
        with self._lock:
            return self._robots.get(robot_id)


@dataclass
class ArenaStateStore:
    """Thread-safe cache of arena boundary and obstacles for pipeline use."""
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _boundary: tuple[tuple[float, float], ...] = field(default=(), repr=False)
    _obstacles: list[tuple[tuple[float, float], ...]] = field(
        default_factory=list, repr=False
    )
    _model_dict: dict | None = field(default=None, repr=False)

    def update(
        self,
        boundary: tuple[tuple[float, float], ...],
        obstacles: list[tuple[tuple[float, float], ...]],
        model_dict: dict,
    ) -> None:
        with self._lock:
            self._boundary = boundary
            self._obstacles = obstacles
            self._model_dict = model_dict

    def get_boundary(self) -> tuple[tuple[float, float], ...]:
        with self._lock:
            return self._boundary

    def get_obstacles(self) -> list[tuple[tuple[float, float], ...]]:
        with self._lock:
            return list(self._obstacles)

    def get_model_dict(self) -> dict | None:
        with self._lock:
            return self._model_dict


class ConnectionManager:
    """Manages active WebSocket connections and subscriptions."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._connections: dict[str, WebSocket] = {}  # client_id -> ws
        self._subscriptions: dict[str, set[int]] = {}  # client_id -> robot_ids

    def connect(self, client_id: str, ws: WebSocket) -> None:
        with self._lock:
            self._connections[client_id] = ws
            self._subscriptions[client_id] = set()

    def disconnect(self, client_id: str) -> None:
        with self._lock:
            self._connections.pop(client_id, None)
            self._subscriptions.pop(client_id, None)

    def subscribe(self, client_id: str, robot_ids: list[int]) -> None:
        with self._lock:
            subs = self._subscriptions.get(client_id, set())
            subs.update(robot_ids)
            self._subscriptions[client_id] = subs

    def unsubscribe(self, client_id: str, robot_ids: list[int]) -> None:
        with self._lock:
            subs = self._subscriptions.get(client_id, set())
            subs -= set(robot_ids)

    def get_subscribers(self, robot_id: int) -> list[tuple[str, WebSocket]]:
        """Get all (client_id, ws) pairs subscribed to a robot."""
        with self._lock:
            result = []
            for cid, subs in self._subscriptions.items():
                if robot_id in subs:
                    ws = self._connections.get(cid)
                    if ws is not None:
                        result.append((cid, ws))
            return result

    def get_connection(self, client_id: str) -> WebSocket | None:
        with self._lock:
            return self._connections.get(client_id)

    def all_connections(self) -> list[tuple[str, WebSocket]]:
        with self._lock:
            return list(self._connections.items())


def create_gateway_app(
    robot_store: RobotStatusStore,
    arena_store: ArenaStateStore,
    control_manager: ControlManager,
    connection_manager: ConnectionManager,
    on_cmd_vel: Callable[[int, float, float], None],
    on_spawn: Callable[[float, float, float, float], dict],
    on_remove: Callable[[int], dict],
    on_reset: Callable[[int], dict],
    on_list_robots: Callable[[], list[dict]],
    on_configure_sensor: Callable[[int, str, dict], tuple[bool, str]] | None = None,
    on_apply_preset: Callable[[int, str], tuple[bool, str]] | None = None,
    on_get_robot_sensors: Callable[[int], list[str]] | None = None,
    on_inject_event: Callable[[str, dict], tuple[bool, str]] | None = None,
    on_fire_intent: Callable[[int, int | None, str, tuple[float, float] | None], None] | None = None,
    on_observation: Callable[[int, int, str], None] | None = None,
    admin_token: str = "",
    max_linear: float = 0.2,
    max_angular: float = 2.0,
    lookahead_dt: float = 0.1,
    vision_server_url: str = "http://localhost:8081",
    static_dir: str | None = None,
) -> FastAPI:
    """Create the FastAPI application for the API gateway.

    All ROS2 operations are injected as callbacks — this module has no ROS2 imports.
    """
    app = FastAPI(title="Plato Pod API Gateway", docs_url=None, redoc_url=None)

    # --- REST endpoints ---

    @app.get("/robots")
    async def list_robots():
        robots = on_list_robots()
        return JSONResponse(robots)

    @app.post("/robots/spawn")
    async def spawn_robot(body: dict):
        x = float(body.get("x", 0))
        y = float(body.get("y", 0))
        theta = float(body.get("theta", 0))
        radius = float(body.get("radius", 0))
        result = on_spawn(x, y, theta, radius)
        status = 200 if result.get("success") else 400
        return JSONResponse(result, status_code=status)

    @app.delete("/robots/{robot_id}")
    async def remove_robot(robot_id: int):
        result = on_remove(robot_id)
        status = 200 if result.get("success") else 400
        return JSONResponse(result, status_code=status)

    @app.post("/robots/{robot_id}/reset")
    async def reset_robot(robot_id: int):
        result = on_reset(robot_id)
        status = 200 if result.get("success") else 400
        return JSONResponse(result, status_code=status)

    @app.get("/arena/model")
    async def arena_model():
        model = arena_store.get_model_dict()
        if model is None:
            return JSONResponse(
                {"error": "Arena model not available"}, status_code=503
            )
        return JSONResponse(model)

    @app.get("/health")
    async def health():
        robots = robot_store.get_all()
        boundary = arena_store.get_boundary()
        return JSONResponse({
            "status": "ok",
            "robots_count": len(robots),
            "boundary_valid": len(boundary) >= 3,
        })

    # --- Proxy routes (camera + homography from vision node) ---

    def _proxy_mjpeg(path: str):
        """Stream MJPEG from the vision server to the browser."""
        def generate():
            url = f"{vision_server_url}{path}"
            try:
                resp = urllib.request.urlopen(url, timeout=10)
                while True:
                    chunk = resp.read(8192)
                    if not chunk:
                        break
                    yield chunk
            except Exception as e:
                logger.debug("MJPEG proxy stream ended: %s", e)
        return generate

    @app.get("/camera/stream")
    async def camera_stream_proxy():
        return StreamingResponse(
            _proxy_mjpeg("/camera/stream")(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/camera/stream/debug")
    async def camera_stream_debug_proxy():
        return StreamingResponse(
            _proxy_mjpeg("/camera/stream/debug")(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/arena/homography")
    async def arena_homography_proxy():
        try:
            url = f"{vision_server_url}/arena/homography"
            resp = urllib.request.urlopen(url, timeout=2)
            data = json.loads(resp.read().decode())
            return JSONResponse(data)
        except Exception:
            return JSONResponse(
                {"error": "Homography not available"}, status_code=503
            )

    @app.get("/tags/detections")
    async def tags_detections_proxy():
        try:
            url = f"{vision_server_url}/tags/detections"
            resp = urllib.request.urlopen(url, timeout=2)
            data = json.loads(resp.read().decode())
            return JSONResponse(data)
        except Exception:
            return JSONResponse(
                {"error": "Detections not available"}, status_code=503
            )

    @app.get("/camera/info")
    async def camera_info_proxy():
        try:
            url = f"{vision_server_url}/camera/info"
            resp = urllib.request.urlopen(url, timeout=2)
            data = json.loads(resp.read().decode())
            return JSONResponse(data)
        except Exception:
            return JSONResponse(
                {"error": "Camera info not available"}, status_code=503
            )

    # --- WebSocket endpoint ---

    @app.websocket("/api/control")
    async def websocket_control(ws: WebSocket):
        await ws.accept()
        client_id = str(uuid.uuid4())
        connection_manager.connect(client_id, ws)
        logger.info("WebSocket client connected: %s", client_id)

        try:
            while True:
                raw = await ws.receive_text()
                parsed = parse_client_message(raw)
                if parsed is None:
                    await ws.send_text(format_error(
                        "invalid_command", None, "Invalid JSON message"
                    ))
                    continue

                msg_type, fields = parsed

                if msg_type == "cmd_vel":
                    await _handle_cmd_vel(ws, client_id, fields)
                elif msg_type == "subscribe":
                    await _handle_subscribe(ws, client_id, fields)
                elif msg_type == "unsubscribe":
                    await _handle_unsubscribe(client_id, fields)
                elif msg_type == "subscribe_sensors":
                    await _handle_subscribe_sensors(ws, client_id, fields)
                elif msg_type == "unsubscribe_sensors":
                    await _handle_unsubscribe_sensors(client_id, fields)
                elif msg_type == "configure_sensor":
                    await _handle_configure_sensor(ws, fields)
                elif msg_type == "apply_sensor_preset":
                    await _handle_apply_preset(ws, fields)
                elif msg_type == "inject_event":
                    await _handle_inject_event(ws, fields)
                elif msg_type == "fire_weapon":
                    await _handle_fire_weapon(ws, fields)
                elif msg_type == "report_observation":
                    await _handle_report_observation(ws, fields)
                else:
                    await ws.send_text(format_error(
                        "invalid_command", None,
                        f"Unknown message type: {msg_type}"
                    ))

        except WebSocketDisconnect:
            pass
        finally:
            released = control_manager.release_all(client_id)
            for rid in released:
                on_cmd_vel(rid, 0.0, 0.0)  # stop released robots
            connection_manager.disconnect(client_id)
            logger.info("WebSocket client disconnected: %s", client_id)

    async def _handle_cmd_vel(
        ws: WebSocket, client_id: str, fields: dict
    ) -> None:
        valid, err_msg = validate_cmd_vel(fields)
        if not valid:
            await ws.send_text(format_error("invalid_command", None, err_msg))
            return

        robot_id = fields["robot_id"]
        linear_x = float(fields["linear_x"])
        angular_z = float(fields["angular_z"])

        # Check robot exists and is active
        robot_info = robot_store.get(robot_id)
        if robot_info is None:
            await ws.send_text(format_error("not_found", robot_id, "Robot not found"))
            return
        if robot_info.get("status") != "active":
            await ws.send_text(format_error(
                "robot_inactive", robot_id, "Robot is not active"
            ))
            return

        # Try to acquire control
        ok, err_code = control_manager.try_acquire(client_id, robot_id)
        if not ok:
            await ws.send_text(format_error(
                err_code, robot_id, "Robot is controlled by another client"
            ))
            return

        # Auto-subscribe
        connection_manager.subscribe(client_id, [robot_id])

        # Build robot state for pipeline
        robot_state = Robot(
            robot_id=robot_id,
            deployment=robot_info.get("type", "virtual"),
            x=robot_info["x"],
            y=robot_info["y"],
            theta=robot_info["theta"],
            radius=robot_info["radius"],
        )

        # Get other robots for collision filter
        all_robots = robot_store.get_all()
        other_states = [
            Robot(
                robot_id=r["robot_id"],
                deployment=r.get("type", "virtual"),
                x=r["x"], y=r["y"],
                theta=r["theta"], radius=r["radius"],
            )
            for r in all_robots
            if r["robot_id"] != robot_id and r.get("status") == "active"
        ]

        # Get arena state
        boundary = arena_store.get_boundary()
        obstacles = arena_store.get_obstacles()

        # Run command pipeline
        result = run_pipeline(
            linear_x, angular_z, robot_state, other_states,
            boundary, obstacles, max_linear, max_angular, lookahead_dt,
        )

        # Record command and publish
        now = time.monotonic()
        control_manager.record_command(
            client_id, robot_id, result.linear_x, result.angular_z, now
        )
        on_cmd_vel(robot_id, result.linear_x, result.angular_z)

        # Send pipeline events to subscribers
        for event in result.events:
            await _broadcast_event(event)

    async def _handle_subscribe(
        ws: WebSocket, client_id: str, fields: dict
    ) -> None:
        valid, err_msg = validate_subscribe(fields)
        if not valid:
            await ws.send_text(format_error("invalid_command", None, err_msg))
            return

        robot_ids = fields["robot_ids"]
        subscribed = []
        failed = []

        for rid in robot_ids:
            robot_info = robot_store.get(rid)
            if robot_info is not None:
                subscribed.append(rid)
            else:
                failed.append({"robot_id": rid, "reason": "not_found"})

        connection_manager.subscribe(client_id, subscribed)
        await ws.send_text(format_subscribe_ack(subscribed, failed))

    async def _handle_unsubscribe(client_id: str, fields: dict) -> None:
        valid, _ = validate_subscribe(fields)
        if not valid:
            return
        connection_manager.unsubscribe(client_id, fields["robot_ids"])

    # --- Sensor handlers ---

    # Per-client sensor subscriptions: client_id -> {robot_id -> set of sensor names}
    _sensor_subs: dict[str, dict[int, set[str]]] = {}

    async def _handle_subscribe_sensors(
        ws: WebSocket, client_id: str, fields: dict
    ) -> None:
        valid, err_msg = validate_subscribe_sensors(fields)
        if not valid:
            await ws.send_text(format_error("invalid_command", None, err_msg))
            return

        robot_id = fields["robot_id"]
        sensors = fields["sensors"]

        robot_info = robot_store.get(robot_id)
        if robot_info is None:
            await ws.send_text(format_error("not_found", robot_id, "Robot not found"))
            return

        # Check which sensors are available
        available = on_get_robot_sensors(robot_id) if on_get_robot_sensors else []
        subscribed = []
        failed = []
        for s in sensors:
            if s in available or on_get_robot_sensors is None:
                subscribed.append(s)
            else:
                failed.append({"sensor": s, "reason": "not_configured"})

        # Track subscription
        if client_id not in _sensor_subs:
            _sensor_subs[client_id] = {}
        if robot_id not in _sensor_subs[client_id]:
            _sensor_subs[client_id][robot_id] = set()
        _sensor_subs[client_id][robot_id].update(subscribed)

        # Also subscribe to robot for pose updates
        connection_manager.subscribe(client_id, [robot_id])

        await ws.send_text(format_subscribe_sensors_ack(robot_id, subscribed, failed))

    async def _handle_unsubscribe_sensors(client_id: str, fields: dict) -> None:
        valid, _ = validate_subscribe_sensors(fields)
        if not valid:
            return
        robot_id = fields["robot_id"]
        sensors = fields["sensors"]
        if client_id in _sensor_subs and robot_id in _sensor_subs[client_id]:
            _sensor_subs[client_id][robot_id] -= set(sensors)

    async def _handle_configure_sensor(ws: WebSocket, fields: dict) -> None:
        valid, err_msg = validate_configure_sensor(fields)
        if not valid:
            await ws.send_text(format_error("invalid_command", None, err_msg))
            return

        robot_id = fields["robot_id"]
        sensor_name = fields["sensor"]
        config = fields["config"]

        if on_configure_sensor:
            ok, msg = on_configure_sensor(robot_id, sensor_name, config)
            status = "ok" if ok else msg
        else:
            status = "ok"

        await ws.send_text(format_configure_sensor_ack(robot_id, sensor_name, status))

    async def _handle_apply_preset(ws: WebSocket, fields: dict) -> None:
        valid, err_msg = validate_apply_preset(fields)
        if not valid:
            await ws.send_text(format_error("invalid_command", None, err_msg))
            return

        robot_id = fields["robot_id"]
        preset_name = fields["preset"]

        if on_apply_preset:
            ok, msg = on_apply_preset(robot_id, preset_name)
            status = "ok" if ok else msg
        else:
            status = "ok"

        await ws.send_text(format_configure_sensor_ack(
            robot_id, f"preset:{preset_name}", status
        ))

    async def _handle_inject_event(ws: WebSocket, fields: dict) -> None:
        valid, err_msg = validate_inject_event(fields)
        if not valid:
            await ws.send_text(format_error("invalid_command", None, err_msg))
            return

        # Verify admin token
        if admin_token and fields.get("admin_token") != admin_token:
            await ws.send_text(format_error(
                "unauthorized", None, "Invalid admin token"
            ))
            return

        event_type = fields["event_type"]
        data = fields.get("data", {})

        if on_inject_event:
            ok, msg = on_inject_event(event_type, data)
            if ok:
                await ws.send_text(format_inject_event_ack(event_type, "ok"))
                # Broadcast environment update to all connected clients
                env_msg = format_environment_update(event_type, data)
                for _, cws in connection_manager.all_connections():
                    try:
                        if cws.client_state == WebSocketState.CONNECTED:
                            await cws.send_text(env_msg)
                    except Exception:
                        pass
            else:
                await ws.send_text(format_inject_event_ack(event_type, "error", msg))
        else:
            await ws.send_text(format_inject_event_ack(
                event_type, "error", "Event injection not configured"
            ))

    async def _handle_fire_weapon(ws: WebSocket, fields: dict) -> None:
        valid, err_msg = validate_fire_weapon(fields)
        if not valid:
            await ws.send_text(format_error("invalid_command", None, err_msg))
            return
        if on_fire_intent is None:
            await ws.send_text(format_error(
                "unsupported", fields.get("robot_id"),
                "fire_weapon not configured (engagement_node not running?)",
            ))
            return
        target_pos = fields.get("target_position")
        pos_tuple = (
            (float(target_pos[0]), float(target_pos[1]))
            if target_pos else None
        )
        on_fire_intent(
            int(fields["robot_id"]),
            fields.get("target_id"),
            str(fields["weapon"]),
            pos_tuple,
        )

    async def _handle_report_observation(ws: WebSocket, fields: dict) -> None:
        valid, err_msg = validate_report_observation(fields)
        if not valid:
            await ws.send_text(format_error("invalid_command", None, err_msg))
            return
        if on_observation is None:
            return
        on_observation(
            int(fields["robot_id"]),
            int(fields["target_id"]),
            str(fields["classification"]),
        )

    async def _broadcast_event(event: PipelineEvent) -> None:
        """Send an event to all clients subscribed to the robot."""
        msg = format_event(event.robot_id, event.event, event.detail)
        subscribers = connection_manager.get_subscribers(event.robot_id)
        for _, ws in subscribers:
            try:
                if ws.client_state == WebSocketState.CONNECTED:
                    await ws.send_text(msg)
            except Exception:
                pass

    # --- Static file serving (must be last — catches all unmatched routes) ---
    if static_dir is not None:
        import os
        if os.path.isdir(static_dir):
            app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")

    return app


async def broadcast_poses(
    connection_manager: ConnectionManager,
    robot_store: RobotStatusStore,
    control_manager: ControlManager,
) -> None:
    """Send pose updates to all subscribed clients. Called from the node's timer."""
    robots = robot_store.get_all()

    for robot_info in robots:
        robot_id = robot_info["robot_id"]
        subscribers = connection_manager.get_subscribers(robot_id)
        if not subscribers:
            continue

        # Get current commanded velocity from control manager
        controller = control_manager.get_controller(robot_id)
        linear_x = 0.0
        angular_z = 0.0
        # Velocity info would need to come from the control entry,
        # but we don't expose that directly. Use zeros for now —
        # actual velocity tracking requires integration with the control manager.

        msg = format_pose(
            robot_id,
            robot_info["x"], robot_info["y"], robot_info["theta"],
            linear_x, angular_z,
        )

        for _, ws in subscribers:
            try:
                if ws.client_state == WebSocketState.CONNECTED:
                    await ws.send_text(msg)
            except Exception:
                pass
