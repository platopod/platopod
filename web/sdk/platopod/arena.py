"""Arena SDK — connect to the Plato Pod server and control robots.

Students write synchronous code; internally the SDK runs a background
asyncio thread for WebSocket communication.

Example:
    from platopod import Arena

    arena = Arena("ws://192.168.4.1:8080/api/control")
    robots = arena.list_robots()
    arena.cmd_vel(robot_id=1, linear_x=0.1, angular_z=0.0)
    arena.sleep(1.0)
    arena.cmd_vel(robot_id=1, linear_x=0.0, angular_z=0.0)
    arena.close()
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from typing import Callable
from urllib.parse import urljoin, urlparse

logger = logging.getLogger(__name__)


class Arena:
    """Connection to a Plato Pod arena server."""

    def __init__(self, url: str = "ws://localhost:8080/api/control") -> None:
        """Connect to the arena server.

        Args:
            url: WebSocket URL for the control endpoint.
        """
        self._ws_url = url
        self._base_http = self._ws_to_http(url)
        self._poses: dict[int, dict] = {}  # robot_id -> latest pose
        self._sensor_data: dict[str, dict] = {}  # "robot_id:sensor" -> latest data
        self._sensor_handlers: dict[str, list[Callable]] = {}  # "robot_id:sensor" -> callbacks
        self._event_handlers: dict[str, list[Callable]] = {}
        self._error_handler: Callable | None = None
        self._ws = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._connected = threading.Event()
        self._closed = False

        # Start background asyncio loop
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        # Wait for connection
        if not self._connected.wait(timeout=5.0):
            logger.warning("WebSocket connection timeout — some features may not work")

    @staticmethod
    def _ws_to_http(ws_url: str) -> str:
        """Convert ws:// URL to http:// base URL."""
        parsed = urlparse(ws_url)
        scheme = "https" if parsed.scheme == "wss" else "http"
        return f"{scheme}://{parsed.netloc}"

    def _run_loop(self) -> None:
        """Background thread: run the asyncio event loop."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._ws_main())

    async def _ws_main(self) -> None:
        """Main WebSocket connection loop."""
        try:
            import websockets
        except ImportError:
            logger.error("websockets package required: pip install websockets")
            return

        try:
            async with websockets.connect(self._ws_url) as ws:
                self._ws = ws
                self._connected.set()
                logger.info("Connected to %s", self._ws_url)

                async for raw in ws:
                    self._handle_message(raw)
        except Exception as e:
            logger.error("WebSocket error: %s", e)
        finally:
            self._ws = None

    def _handle_message(self, raw: str) -> None:
        """Process a message from the server."""
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        msg_type = data.get("type")

        if msg_type == "pose":
            robot_id = data.get("robot_id")
            if robot_id is not None:
                self._poses[robot_id] = data

        elif msg_type == "event":
            event_name = data.get("event", "")
            for handler in self._event_handlers.get(event_name, []):
                try:
                    handler(data)
                except Exception as e:
                    logger.error("Event handler error: %s", e)

        elif msg_type == "error":
            if self._error_handler:
                try:
                    self._error_handler(data)
                except Exception as e:
                    logger.error("Error handler error: %s", e)

        elif msg_type == "sensor":
            robot_id = data.get("robot_id")
            sensor_name = data.get("sensor", "")
            key = f"{robot_id}:{sensor_name}"
            self._sensor_data[key] = data.get("data", {})
            for handler in self._sensor_handlers.get(key, []):
                try:
                    handler(data.get("data", {}))
                except Exception as e:
                    logger.error("Sensor handler error: %s", e)

        elif msg_type == "sensor_event":
            robot_id = data.get("robot_id")
            sensor_name = data.get("sensor", "")
            key = f"{robot_id}:{sensor_name}"
            for handler in self._sensor_handlers.get(key, []):
                try:
                    handler(data.get("data", {}))
                except Exception as e:
                    logger.error("Sensor event handler error: %s", e)

        elif msg_type in ("subscribe_ack", "subscribe_sensors_ack",
                          "configure_sensor_ack"):
            pass  # Could store or log

    def _send(self, data: dict) -> None:
        """Send a JSON message over WebSocket."""
        if self._ws is None or self._loop is None:
            logger.warning("Not connected — message dropped")
            return

        raw = json.dumps(data)
        asyncio.run_coroutine_threadsafe(self._ws.send(raw), self._loop)

    # --- Public API ---

    def list_robots(self) -> list[dict]:
        """List all registered robots (REST call).

        Returns:
            List of robot status dicts.
        """
        import urllib.request
        url = f"{self._base_http}/robots"
        try:
            with urllib.request.urlopen(url, timeout=5) as resp:
                return json.loads(resp.read().decode())
        except Exception as e:
            logger.error("list_robots failed: %s", e)
            return []

    def get_world_state(self) -> dict:
        """Snapshot of the tactical world state (REST call).

        Returns a dict with keys:
          civilians     — list of {x, y, label, movement, count, radius_m}
          ied_zones     — list of {x, y, detectability_radius_m, label}
          cover         — list of {vertices, cover_value, label}
          ew_emitters   — list of {x, y, frequency_mhz, signal_strength, label}
          jamming_zones — list of {x, y, radius_m, strength, label}
          dead_zones    — list of {vertices, label}
          weather       — {visibility_m, wind_speed, wind_direction, fog_density, time_of_day} or None
          roe           — {fire_permission, civilian_proximity_m, …} or None
          weapons       — list of {name, max_range_m, …}

        Empty lists / None mean the platform's world_state_node hasn't
        published that section (or no exercise scenario is loaded).
        """
        import urllib.request
        url = f"{self._base_http}/world"
        try:
            with urllib.request.urlopen(url, timeout=5) as resp:
                return json.loads(resp.read().decode())
        except Exception as e:
            logger.warning("get_world_state failed: %s", e)
            return {}

    def get_civilians(self) -> list[dict]:
        """Convenience wrapper: return just the civilian list."""
        return self.get_world_state().get("civilians", []) or []

    def get_ied_zones(self) -> list[dict]:
        """Convenience wrapper: return just the IED hazard list."""
        return self.get_world_state().get("ied_zones", []) or []

    def spawn(
        self, x: float, y: float, theta: float = 0.0, radius: float = 0.0
    ) -> dict:
        """Spawn a virtual robot (REST call).

        Args:
            x: Spawn X position (metres).
            y: Spawn Y position (metres).
            theta: Spawn heading (radians).
            radius: Robot radius (0 for default).

        Returns:
            Result dict with success, robot_id, message.
        """
        import urllib.request
        url = f"{self._base_http}/robots/spawn"
        body = json.dumps({"x": x, "y": y, "theta": theta, "radius": radius}).encode()
        req = urllib.request.Request(url, data=body, method="POST")
        req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                return json.loads(resp.read().decode())
        except Exception as e:
            logger.error("spawn failed: %s", e)
            return {"success": False, "message": str(e)}

    def remove(self, robot_id: int) -> dict:
        """Remove a virtual robot (REST call)."""
        import urllib.request
        url = f"{self._base_http}/robots/{robot_id}"
        req = urllib.request.Request(url, method="DELETE")
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                return json.loads(resp.read().decode())
        except Exception as e:
            return {"success": False, "message": str(e)}

    def cmd_vel(
        self, robot_id: int, linear_x: float = 0.0, angular_z: float = 0.0
    ) -> None:
        """Send a velocity command to a robot.

        Automatically subscribes to the robot's updates.

        Args:
            robot_id: Target robot ID.
            linear_x: Forward/backward speed (m/s).
            angular_z: Rotational speed (rad/s, positive = CCW).
        """
        self._send({
            "type": "cmd_vel",
            "robot_id": robot_id,
            "linear_x": linear_x,
            "angular_z": angular_z,
        })

    def subscribe(self, robot_ids: list[int]) -> None:
        """Subscribe to robot pose updates and events.

        Args:
            robot_ids: List of robot IDs to observe.
        """
        self._send({"type": "subscribe", "robot_ids": robot_ids})

    def unsubscribe(self, robot_ids: list[int]) -> None:
        """Unsubscribe from robot updates.

        Args:
            robot_ids: List of robot IDs to stop observing.
        """
        self._send({"type": "unsubscribe", "robot_ids": robot_ids})

    def release(self, robot_id: int) -> None:
        """Release control of a robot.

        The robot stops and becomes available for other clients.
        """
        self.cmd_vel(robot_id, 0.0, 0.0)
        self.unsubscribe([robot_id])

    def get_pose(self, robot_id: int) -> dict | None:
        """Get the latest cached pose for a robot.

        Returns:
            Pose dict with x, y, theta, or None if no pose received yet.
        """
        return self._poses.get(robot_id)

    def on(self, event_type: str, callback: Callable) -> None:
        """Register a callback for a specific event type.

        Args:
            event_type: Event name (e.g. "boundary_contact", "collision_contact").
            callback: Function called with the event dict.
        """
        self._event_handlers.setdefault(event_type, []).append(callback)

    def on_error(self, callback: Callable) -> None:
        """Register an error callback.

        Args:
            callback: Function called with error dicts.
        """
        self._error_handler = callback

    # --- Sensor API ---

    def subscribe_sensors(
        self, robot_id: int, sensors: list[str], rate_hz: float = 10.0
    ) -> None:
        """Subscribe to sensor data for a robot.

        Args:
            robot_id: Robot to subscribe to.
            sensors: List of sensor names (e.g. ["gps", "lidar_2d"]).
            rate_hz: Desired update rate.
        """
        self._send({
            "type": "subscribe_sensors",
            "robot_id": robot_id,
            "sensors": sensors,
            "rate_hz": rate_hz,
        })

    def unsubscribe_sensors(self, robot_id: int, sensors: list[str]) -> None:
        """Unsubscribe from sensor data."""
        self._send({
            "type": "unsubscribe_sensors",
            "robot_id": robot_id,
            "sensors": sensors,
        })

    def get_sensor(self, robot_id: int, sensor: str) -> dict | None:
        """Get the latest cached sensor reading.

        Returns:
            Sensor data dict, or None if no data received yet.
        """
        return self._sensor_data.get(f"{robot_id}:{sensor}")

    def on_sensor(
        self, sensor: str, robot_id: int, callback: Callable
    ) -> None:
        """Register a callback for sensor data.

        Args:
            sensor: Sensor name (e.g. "gps", "lidar_2d").
            robot_id: Robot ID.
            callback: Function called with sensor data dict.
        """
        key = f"{robot_id}:{sensor}"
        self._sensor_handlers.setdefault(key, []).append(callback)

    def configure_sensor(
        self, robot_id: int, sensor: str, config: dict
    ) -> None:
        """Configure sensor parameters.

        Args:
            robot_id: Target robot.
            sensor: Sensor name.
            config: Configuration dict (e.g. {"noise_stddev": 0.01, "range_max": 2.0}).
        """
        self._send({
            "type": "configure_sensor",
            "robot_id": robot_id,
            "sensor": sensor,
            "config": config,
        })

    def apply_preset(self, robot_id: int, preset: str) -> None:
        """Apply a named sensor preset to a robot.

        Args:
            robot_id: Target robot.
            preset: Preset name (e.g. "minimal", "basic_scout", "full_suite").
        """
        self._send({
            "type": "apply_sensor_preset",
            "robot_id": robot_id,
            "preset": preset,
        })

    def sleep(self, seconds: float) -> None:
        """Sleep for the specified duration.

        Convenience method for control loops.
        """
        time.sleep(seconds)

    def close(self) -> None:
        """Close the connection and stop the background loop."""
        self._closed = True
        if self._ws and self._loop:
            asyncio.run_coroutine_threadsafe(self._ws.close(), self._loop)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
