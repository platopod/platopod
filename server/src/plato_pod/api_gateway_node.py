"""API Gateway Node — WebSocket + REST bridge to the ROS2 graph.

Thin ROS2 shell wiring the pure-function gateway server to ROS2 topics,
publishers, and service clients.
"""

from __future__ import annotations

import asyncio
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from plato_pod_msgs.msg import (
    ArenaModel as ArenaModelMsg,
    FireIntent as FireIntentMsg,
    Observation as ObservationMsg,
    RobotStatusList as RobotStatusListMsg,
)
from plato_pod_msgs.srv import ListRobots, RemoveRobot, ResetRobot, SpawnVirtual

from plato_pod.api_gateway_server import (
    ArenaStateStore,
    ConnectionManager,
    RobotStatusStore,
    broadcast_poses,
    create_gateway_app,
)
from plato_pod.control_manager import ControlManager
from plato_pod.sensor_engine import SensorEngine
from plato_pod.sensor_plugins.base import EnvironmentContext
from plato_pod.spatial_field import GaussianPlumeField
from plato_pod.virtual_layer_loader import load_virtual_layers


class ApiGatewayNode(Node):
    """ROS2 node for the API gateway."""

    def __init__(self) -> None:
        super().__init__("api_gateway_node")

        # Parameters
        self.declare_parameter("port", 8080)
        self.declare_parameter("max_linear_speed", 0.2)
        self.declare_parameter("max_angular_speed", 2.0)
        self.declare_parameter("watchdog_timeout_ms", 500)
        self.declare_parameter("control_timeout_sec", 30)
        self.declare_parameter("pose_rate_hz", 10.0)
        self.declare_parameter("lookahead_dt", 0.1)
        self.declare_parameter("admin_token", "")
        self.declare_parameter("exercise_file", "")

        port = self.get_parameter("port").value
        self._max_linear = self.get_parameter("max_linear_speed").value
        self._max_angular = self.get_parameter("max_angular_speed").value
        self._watchdog_timeout = self.get_parameter("watchdog_timeout_ms").value / 1000.0
        self._control_timeout = self.get_parameter("control_timeout_sec").value
        pose_rate = self.get_parameter("pose_rate_hz").value
        lookahead_dt = self.get_parameter("lookahead_dt").value
        admin_token = self.get_parameter("admin_token").value or ""

        # Read admin token from environment if not set via parameter
        import os
        if not admin_token:
            admin_token = os.environ.get("PLATOPOD_ADMIN_TOKEN", "")

        # Shared state
        self._robot_store = RobotStatusStore()
        self._arena_store = ArenaStateStore()
        self._control_manager = ControlManager()
        self._connection_manager = ConnectionManager()
        self._sensor_engine = SensorEngine()

        # Load virtual layers from exercise YAML if provided
        exercise_file = self.get_parameter("exercise_file").value
        if exercise_file:
            try:
                import yaml
                with open(exercise_file) as f:
                    exercise_config = yaml.safe_load(f)
                env = load_virtual_layers(exercise_config)
                if env is not None:
                    self._sensor_engine.set_environment(env)
                    n_fields = len(env.fields)
                    self.get_logger().info(
                        f"Loaded {n_fields} virtual layer(s) from {exercise_file}"
                    )
            except Exception as e:
                self.get_logger().warning(f"Failed to load virtual layers: {e}")

        # Dynamic cmd_vel publishers: robot_id -> Publisher
        self._cmd_vel_pubs: dict[int, object] = {}

        # Service clients
        self._spawn_client = self.create_client(
            SpawnVirtual, "/registry_node/spawn_virtual"
        )
        self._remove_client = self.create_client(
            RemoveRobot, "/registry_node/remove_robot"
        )
        self._reset_client = self.create_client(
            ResetRobot, "/registry_node/reset_robot"
        )
        self._list_client = self.create_client(
            ListRobots, "/registry_node/list_robots"
        )

        # Subscribers
        self.create_subscription(
            RobotStatusListMsg, "/robots/status",
            self._robot_status_callback, 10,
        )
        self.create_subscription(
            ArenaModelMsg, "/arena/model",
            self._arena_model_callback, 10,
        )

        # Create FastAPI app
        # Resolve static files directory (web/static/ relative to workspace)
        import os
        static_dir = None
        for candidate in ["/ros2_ws/web/static", "web/static", os.path.expanduser("~/web/static")]:
            if os.path.isdir(candidate):
                static_dir = os.path.abspath(candidate)
                break
        if static_dir:
            self.get_logger().info(f"Serving static files from {static_dir}")
        else:
            self.get_logger().warning("No static files directory found — dashboard will not be available")

        app = create_gateway_app(
            robot_store=self._robot_store,
            arena_store=self._arena_store,
            control_manager=self._control_manager,
            connection_manager=self._connection_manager,
            on_cmd_vel=self._publish_cmd_vel,
            on_spawn=self._call_spawn,
            on_remove=self._call_remove,
            on_reset=self._call_reset,
            on_list_robots=self._call_list_robots,
            on_inject_event=self._handle_inject_event,
            on_fire_intent=self._publish_fire_intent,
            on_observation=self._publish_observation,
            admin_token=admin_token,
            max_linear=self._max_linear,
            max_angular=self._max_angular,
            lookahead_dt=lookahead_dt,
            vision_server_url="http://localhost:8081",
            static_dir=static_dir,
        )

        # Start uvicorn in a daemon thread
        self._server_thread = threading.Thread(
            target=self._run_server, args=(app, port), daemon=True
        )
        self._server_thread.start()
        self.get_logger().info(f"API gateway started on port {port}")

        # Pose broadcast timer
        pose_period = 1.0 / pose_rate
        self._pose_timer = self.create_timer(pose_period, self._pose_tick)

        # Watchdog/control timeout timer (20 Hz)
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_tick)

        self.get_logger().info("API gateway node ready")

    def _robot_status_callback(self, msg: RobotStatusListMsg) -> None:
        """Update robot status cache from registry."""
        robots = []
        for r in msg.robots:
            robots.append({
                "robot_id": r.robot_id,
                "type": r.deployment,
                "x": r.x,
                "y": r.y,
                "theta": r.theta,
                "radius": r.radius,
                "status": r.status,
                "localization_id": r.localization_id,
                "localization_source": r.localization_source,
            })
        self._robot_store.update(robots)

    def _arena_model_callback(self, msg: ArenaModelMsg) -> None:
        """Update arena state cache."""
        boundary = tuple(
            (float(x), float(y))
            for x, y in zip(msg.boundary_x, msg.boundary_y)
        )
        obstacles = []
        for obs in msg.obstacles:
            verts = tuple(
                (float(x), float(y))
                for x, y in zip(obs.vertices_x, obs.vertices_y)
            )
            obstacles.append(verts)

        model_dict = {
            "boundary": [list(v) for v in boundary],
            "boundary_valid": msg.boundary_valid,
            "origin_tag": msg.origin_tag,
            "obstacles": [
                {
                    "label": obs.label,
                    "vertices": [list(v) for v in zip(obs.vertices_x, obs.vertices_y)],
                    "type": obs.original_type,
                }
                for obs in msg.obstacles
            ],
            "zones": [
                {
                    "name": z.name,
                    "team": z.team if z.team else None,
                    "vertices": [list(v) for v in zip(z.vertices_x, z.vertices_y)],
                    "type": z.original_type,
                    "hold_time_seconds": z.hold_time_seconds,
                }
                for z in msg.zones
            ],
        }
        self._arena_store.update(boundary, obstacles, model_dict)

    def _publish_cmd_vel(self, robot_id: int, linear_x: float, angular_z: float) -> None:
        """Publish a Twist message to /robot_{id}/cmd_vel."""
        if robot_id not in self._cmd_vel_pubs:
            topic = f"/robot_{robot_id}/cmd_vel"
            self._cmd_vel_pubs[robot_id] = self.create_publisher(Twist, topic, 10)

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_vel_pubs[robot_id].publish(msg)

    def _ensure_fire_intent_pub(self):
        if not hasattr(self, "_fire_intent_pub_obj"):
            self._fire_intent_pub_obj = self.create_publisher(
                FireIntentMsg, "/fire_weapon", 10,
            )
            self._next_fire_seq = 0
        return self._fire_intent_pub_obj

    def _ensure_observation_pub(self):
        if not hasattr(self, "_observation_pub_obj"):
            self._observation_pub_obj = self.create_publisher(
                ObservationMsg, "/report_observation", 10,
            )
        return self._observation_pub_obj

    def _publish_fire_intent(
        self, robot_id: int, target_id: int | None,
        weapon: str, target_position: tuple[float, float] | None,
    ) -> None:
        """Forward a WebSocket fire_weapon as a typed FireIntent message."""
        pub = self._ensure_fire_intent_pub()
        intent = FireIntentMsg()
        intent.actor_id = int(robot_id)
        intent.target_id = int(target_id) if target_id is not None else -1
        intent.weapon = str(weapon)
        if target_position is not None:
            intent.has_target_position = True
            intent.target_x = float(target_position[0])
            intent.target_y = float(target_position[1])
        else:
            intent.has_target_position = False
            intent.target_x = 0.0
            intent.target_y = 0.0
        self._next_fire_seq += 1
        intent.source_seq = self._next_fire_seq
        pub.publish(intent)

    def _publish_observation(
        self, robot_id: int, target_id: int, classification: str,
    ) -> None:
        """Forward a report_observation event as a typed Observation message."""
        pub = self._ensure_observation_pub()
        obs = ObservationMsg()
        obs.actor_id = int(robot_id)
        obs.target_id = int(target_id)
        obs.classification = str(classification)
        obs.confidence = 1.0
        pub.publish(obs)

    @staticmethod
    def _wait_for_future(future, timeout: float = 2.0):
        """Wait for a ROS2 future to complete without re-spinning the executor.

        The executor is already spinning in main(). Service call futures are
        processed by the running executor. We just poll until done.
        """
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return None
            time.sleep(0.01)
        return future.result()

    def _call_spawn(self, x: float, y: float, theta: float, radius: float) -> dict:
        """Call the spawn_virtual service."""
        if not self._spawn_client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "robot_id": -1, "message": "Registry unavailable"}

        req = SpawnVirtual.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.radius = radius

        future = self._spawn_client.call_async(req)
        r = self._wait_for_future(future)
        if r is not None:
            return {"success": r.success, "robot_id": r.robot_id, "message": r.message}
        return {"success": False, "robot_id": -1, "message": "Service call timeout"}

    def _call_remove(self, robot_id: int) -> dict:
        """Call the remove_robot service."""
        if not self._remove_client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": "Registry unavailable"}

        req = RemoveRobot.Request()
        req.robot_id = robot_id

        future = self._remove_client.call_async(req)
        r = self._wait_for_future(future)
        if r is not None:
            return {"success": r.success, "message": r.message}
        return {"success": False, "message": "Service call timeout"}

    def _call_reset(self, robot_id: int) -> dict:
        """Call the reset_robot service."""
        if not self._reset_client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": "Registry unavailable"}

        req = ResetRobot.Request()
        req.robot_id = robot_id

        future = self._reset_client.call_async(req)
        r = self._wait_for_future(future)
        if r is not None:
            return {"success": r.success, "message": r.message}
        return {"success": False, "message": "Service call timeout"}

    def _call_list_robots(self) -> list[dict]:
        """Call the list_robots service."""
        if not self._list_client.wait_for_service(timeout_sec=1.0):
            return []

        req = ListRobots.Request()
        future = self._list_client.call_async(req)
        r = self._wait_for_future(future)
        if r is not None:
            return [
                {
                    "robot_id": rob.robot_id,
                    "type": rob.deployment,
                    "x": rob.x,
                    "y": rob.y,
                    "theta": rob.theta,
                    "radius": rob.radius,
                    "status": rob.status,
                    "localization_id": rob.localization_id,
                    "localization_source": rob.localization_source,
                }
                for rob in r.robots
            ]
        return []

    def _handle_inject_event(
        self, event_type: str, data: dict
    ) -> tuple[bool, str]:
        """Handle an injected event from a WebSocket admin client.

        Updates the sensor engine's environment based on the event type.
        """
        if event_type == "place_gas_source":
            x = data.get("x", 0.0)
            y = data.get("y", 0.0)
            rate = data.get("release_rate", 100.0)
            wind_speed = data.get("wind_speed", 2.0)
            wind_dir = data.get("wind_direction", 0.0)
            diffusion = data.get("diffusion_coeff", 0.05)
            name = data.get("name", "gas")

            field = GaussianPlumeField(
                source_x=x, source_y=y,
                release_rate=rate,
                wind_speed=wind_speed,
                wind_direction=wind_dir,
                diffusion_coeff=diffusion,
            )
            self._sensor_engine.update_field(name, field)
            self.get_logger().info(
                f"Gas source placed at ({x}, {y}), rate={rate}"
            )
            return True, "ok"

        elif event_type == "update_wind":
            env = self._sensor_engine.get_environment()
            if env is None:
                env = EnvironmentContext()
            env.wind_speed = data.get("speed", env.wind_speed)
            env.wind_direction = data.get("direction", env.wind_direction)
            self._sensor_engine.set_environment(env)
            self.get_logger().info(
                f"Wind updated: speed={env.wind_speed}, dir={env.wind_direction}"
            )
            return True, "ok"

        elif event_type == "update_field":
            field_name = data.get("field_name")
            if not field_name:
                return False, "field_name required"
            params = data.get("params", {})
            # For now, only support updating wind parameters on gas fields
            env = self._sensor_engine.get_environment()
            if env and field_name in env.fields:
                field = env.fields[field_name]
                if hasattr(field, "wind_speed") and "wind_speed" in params:
                    field.wind_speed = params["wind_speed"]
                if hasattr(field, "wind_direction") and "wind_direction" in params:
                    field.wind_direction = params["wind_direction"]
                if hasattr(field, "release_rate") and "release_rate" in params:
                    field.release_rate = params["release_rate"]
                return True, "ok"
            return False, f"Field '{field_name}' not found"

        elif event_type == "reset_exercise":
            self._sensor_engine.reset_state()
            self._sensor_engine.set_environment(EnvironmentContext())
            self.get_logger().info("Exercise state reset")
            return True, "ok"

        elif event_type == "trigger_strike":
            # Log the event; actual damage model is exercise-specific
            robot_id = data.get("robot_id")
            self.get_logger().info(f"Strike triggered on robot {robot_id}")
            return True, "ok"

        elif event_type == "place_obstacle":
            # Dynamic obstacles need arena model integration (future)
            self.get_logger().warning("place_obstacle not yet implemented")
            return False, "not implemented"

        return False, f"Unhandled event type: {event_type}"

    def _pose_tick(self) -> None:
        """Broadcast pose updates to subscribed WebSocket clients."""
        # Run the async broadcast in a new event loop since we're in a ROS2 thread
        try:
            loop = asyncio.new_event_loop()
            loop.run_until_complete(broadcast_poses(
                self._connection_manager,
                self._robot_store,
                self._control_manager,
            ))
            loop.close()
        except Exception:
            pass

    def _watchdog_tick(self) -> None:
        """Process velocity watchdog and control timeouts."""
        now = time.monotonic()
        watchdog_events, release_events = self._control_manager.tick(
            now, self._watchdog_timeout, self._control_timeout
        )

        # Zero velocity for watchdog timeouts
        for wd in watchdog_events:
            self._publish_cmd_vel(wd.robot_id, 0.0, 0.0)

        # Zero velocity for control releases
        for cr in release_events:
            self._publish_cmd_vel(cr.robot_id, 0.0, 0.0)

    @staticmethod
    def _run_server(app, port: int) -> None:
        """Run uvicorn in a background thread."""
        import uvicorn
        uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod api_gateway_node."""
    rclpy.init(args=args)
    node = ApiGatewayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
