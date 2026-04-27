"""CoT Bridge Node — translates Plato Pod data to Cursor on Target XML.

Subscribes to /robots/status, /arena/model, and per-robot sensor topics.
Converts robot poses, arena elements, sensor readings, and virtual layer
overlays to CoT events and sends them to ATAK via UDP/TCP. Listens for
inbound ATAK waypoints on a configurable UDP port.
"""

from __future__ import annotations

import json
import math
import socket
import threading

import rclpy
from rclpy.node import Node

from plato_pod_msgs.msg import (
    ArenaModel as ArenaModelMsg,
    EngagementOutcome as EngagementOutcomeMsg,
    NavGoal,
    RobotStatusList as RobotStatusListMsg,
    SensorReading as SensorReadingMsg,
)

from plato_pod.cot_protocol import (
    VEHICLE_ROLE_TO_COT_TYPE,
    make_casualty_event,
    make_contact_detail,
    make_cot_event,
    make_engagement_event,
    make_sensor_detail,
    make_shape_event,
    make_track_detail,
    parse_cot_event,
    parse_nav_goal,
    robot_id_from_uid,
)
from plato_pod.cot_transport import CotTransport, create_transport
from plato_pod.geo_reference import GeoReference


class CotBridgeNode(Node):
    """ROS2 node bridging Plato Pod to ATAK via CoT protocol."""

    def __init__(self) -> None:
        super().__init__("cot_bridge_node")

        # Parameters
        # Use dynamic_typing so ROS2 accepts any type for parameter overrides
        # (avoids int/float/string type conflicts between launch files and node defaults)
        from rcl_interfaces.msg import ParameterDescriptor
        _dyn = ParameterDescriptor(dynamic_typing=True)

        for name, default in [
            ("geo_origin_lat", -35.2942),
            ("geo_origin_lon", 149.164),
            ("geo_origin_alt", 580.0),
            ("geo_rotation_deg", 0.0),
            ("transport", "udp_unicast"),
            ("target_host", "192.168.1.100"),
            ("target_port", 6969),
            ("publish_rate_hz", 2.0),
            ("stale_seconds", 30.0),
            ("arena_republish_seconds", 60.0),
            ("inbound_port", 4242),
            ("default_vehicle_role", "recon"),
            ("exercise_file", ""),
            ("scale_factor", 1.0),
            ("publish_sensor_data", True),
            ("publish_zones", True),
            ("publish_obstacles", True),
        ]:
            self.declare_parameter(name, default, _dyn)

        # Read parameters with explicit type conversion
        origin_lat = float(self.get_parameter("geo_origin_lat").value)
        origin_lon = float(self.get_parameter("geo_origin_lon").value)
        origin_alt = float(self.get_parameter("geo_origin_alt").value)
        rotation_deg = float(self.get_parameter("geo_rotation_deg").value)
        transport_mode = str(self.get_parameter("transport").value)
        target_host = str(self.get_parameter("target_host").value)
        target_port = int(float(self.get_parameter("target_port").value))
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self._stale_seconds = float(self.get_parameter("stale_seconds").value)
        arena_republish = float(self.get_parameter("arena_republish_seconds").value)
        inbound_port = int(float(self.get_parameter("inbound_port").value))
        self._default_role = str(self.get_parameter("default_vehicle_role").value)
        scale_factor = float(self.get_parameter("scale_factor").value)
        self._publish_sensor_data = bool(self.get_parameter("publish_sensor_data").value)
        self._publish_zones = bool(self.get_parameter("publish_zones").value)
        self._publish_obstacles = bool(self.get_parameter("publish_obstacles").value)

        # Geo reference
        self._geo = GeoReference(
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            origin_alt=origin_alt,
            rotation_deg=rotation_deg,
            scale_factor=scale_factor,
        )

        # Transport
        self._transport: CotTransport = create_transport(
            transport_mode, host=target_host, port=target_port
        )

        # Cached state
        self._robots: list[dict] = []
        self._arena_boundary: list[tuple[float, float]] = []
        self._arena_zones: list[dict] = []
        self._arena_obstacles: list[dict] = []
        self._robot_lock = threading.Lock()
        self._sensor_lock = threading.Lock()
        self._robot_sensors: dict[int, dict[str, dict]] = {}
        # robot_id -> {sensor_name -> latest reading dict}

        # Vehicle role and callsign overrides — loaded from exercise YAML
        self._vehicle_roles: dict[int, str] = {}
        self._callsigns: dict[int, str] = {}

        exercise_file = str(self.get_parameter("exercise_file").value)
        if exercise_file:
            self._load_roles_from_exercise(exercise_file)

        # Known robot IDs for dynamic sensor subscriptions
        self._subscribed_robots: set[int] = set()

        # Subscribers
        self.create_subscription(
            RobotStatusListMsg, "/robots/status",
            self._robot_status_callback, 10,
        )
        self.create_subscription(
            ArenaModelMsg, "/arena/model",
            self._arena_model_callback, 10,
        )
        self.create_subscription(
            EngagementOutcomeMsg, "/engagement_events",
            self._engagement_event_callback, 10,
        )

        # Publisher for inbound nav goals
        self._pub_nav_goal = self.create_publisher(NavGoal, "/cot/nav_goal", 10)

        # Outbound timers
        publish_period = 1.0 / publish_rate
        self._pose_timer = self.create_timer(publish_period, self._publish_robots)
        self._arena_timer = self.create_timer(arena_republish, self._publish_arena)

        # Inbound listener (UDP)
        self._inbound_running = True
        self._inbound_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._inbound_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._inbound_sock.bind(("0.0.0.0", inbound_port))
        self._inbound_sock.settimeout(0.5)

        self._inbound_thread = threading.Thread(
            target=self._inbound_loop, daemon=True
        )
        self._inbound_thread.start()

        self.get_logger().info(
            f"CoT bridge ready: {transport_mode} → {target_host}:{target_port}, "
            f"inbound on :{inbound_port}, "
            f"geo origin ({origin_lat:.4f}, {origin_lon:.4f})"
        )

    def _load_roles_from_exercise(self, exercise_file: str) -> None:
        """Load vehicle roles and callsigns from exercise YAML."""
        try:
            import yaml
            with open(exercise_file) as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warning(f"Cannot load exercise for roles: {e}")
            return

        exercise = config.get("exercise", config)
        robots = exercise.get("robots", {})
        if isinstance(robots, dict):
            all_robots = robots.get("physical", []) + robots.get("virtual", [])
        else:
            all_robots = robots

        for r in all_robots:
            rid = r.get("tag_id", r.get("id", r.get("robot_id")))
            if rid is None:
                continue
            role = r.get("vehicle_role", "")
            if role:
                self._vehicle_roles[rid] = role
            team = r.get("team", "")
            callsign = r.get("callsign", "")
            if callsign:
                self._callsigns[rid] = callsign
            elif team:
                self._callsigns[rid] = f"{team.upper()}-{rid}"

        if self._vehicle_roles:
            self.get_logger().info(
                f"Loaded vehicle roles: {self._vehicle_roles}"
            )

    # --- Callbacks ---

    def _robot_status_callback(self, msg: RobotStatusListMsg) -> None:
        """Cache latest robot statuses and subscribe to new sensor topics."""
        current_ids = set()
        with self._robot_lock:
            self._robots = [
                {
                    "robot_id": r.robot_id,
                    "x": r.x, "y": r.y, "theta": r.theta,
                    "radius": r.radius,
                    "deployment": r.deployment,
                    "status": r.status,
                    "team": r.team,
                    "vehicle_role": r.vehicle_role,
                    "health": r.health if r.health > 0.0 else 1.0,
                }
                for r in msg.robots
                if r.status == "active"
            ]
            current_ids = {r["robot_id"] for r in self._robots}

        # Subscribe to sensor topics for new robots
        if self._publish_sensor_data:
            for rid in current_ids - self._subscribed_robots:
                self._subscribe_robot_sensors(rid)
            self._subscribed_robots = current_ids

    def _subscribe_robot_sensors(self, robot_id: int) -> None:
        """Subscribe to typed SensorReading topics for a robot."""
        for sensor_name in ["gas", "gps", "lidar_2d", "sonar", "fof"]:
            topic = f"/robot_{robot_id}/sensors/{sensor_name}"

            def callback(msg: SensorReadingMsg, rid=robot_id, sname=sensor_name) -> None:
                try:
                    payload = (
                        json.loads(msg.payload)
                        if msg.payload_format == "json" and msg.payload
                        else {}
                    )
                except (json.JSONDecodeError, TypeError):
                    payload = {}
                with self._sensor_lock:
                    if rid not in self._robot_sensors:
                        self._robot_sensors[rid] = {}
                    self._robot_sensors[rid][sname] = payload

            self.create_subscription(SensorReadingMsg, topic, callback, 10)

    def _arena_model_callback(self, msg: ArenaModelMsg) -> None:
        """Cache arena boundary, zones, and obstacles."""
        if msg.boundary_valid:
            self._arena_boundary = [
                (float(x), float(y))
                for x, y in zip(msg.boundary_x, msg.boundary_y)
            ]

        self._arena_zones = [
            {
                "name": z.name,
                "team": z.team,
                "vertices": list(zip(
                    [float(x) for x in z.vertices_x],
                    [float(y) for y in z.vertices_y],
                )),
            }
            for z in msg.zones
        ]

        self._arena_obstacles = [
            {
                "label": obs.label,
                "vertices": list(zip(
                    [float(x) for x in obs.vertices_x],
                    [float(y) for y in obs.vertices_y],
                )),
            }
            for obs in msg.obstacles
        ]

    def _engagement_event_callback(self, msg: EngagementOutcomeMsg) -> None:
        """Translate a typed EngagementOutcome into CoT alert + casualty marker."""
        if not msg.fired:
            return  # nothing to render for blocked/skipped intents

        actor = int(msg.actor_id)
        target = int(msg.target_id)
        weapon = str(msg.weapon)

        # Resolve target position from cached robots
        with self._robot_lock:
            tgt = next(
                (r for r in self._robots if r.get("robot_id") == target),
                None,
            )
        if tgt is None:
            return
        try:
            lat, lon = self._geo.arena_to_latlon(tgt["x"], tgt["y"])
        except Exception as e:
            self.get_logger().warning(f"Geo conversion failed: {e}")
            return

        verdict = "hit" if msg.hit else ("blocked" if msg.blocked else "miss")

        try:
            xml = make_engagement_event(
                actor_uid=f"platopod-{actor}" if actor >= 0 else "indirect",
                target_uid=f"platopod-{target}",
                weapon=weapon,
                outcome=verdict,
                lat=lat, lon=lon,
                rationale=str(msg.rationale),
            )
            self._transport.send(xml)
        except Exception as e:
            self.get_logger().warning(f"Failed to send engagement CoT: {e}")

        if msg.hit and msg.damage > 0:
            new_health = max(0.0, float(tgt.get("health", 1.0)) - float(msg.damage))
            status = "destroyed" if new_health <= 0.0 else "wounded"
            try:
                xml = make_casualty_event(
                    robot_uid=f"platopod-{target}-cas",
                    callsign=str(tgt.get("callsign", f"BOT-{target}")),
                    lat=lat, lon=lon,
                    status=status,
                    health=new_health,
                )
                self._transport.send(xml)
            except Exception as e:
                self.get_logger().warning(f"Failed to send casualty CoT: {e}")

    # --- Outbound publishing ---

    def _publish_robots(self) -> None:
        """Send CoT events for all active robots with sensor data."""
        with self._robot_lock:
            robots = list(self._robots)

        for r in robots:
            robot_id = r["robot_id"]
            lat, lon = self._geo.arena_to_latlon(r["x"], r["y"])

            # Heading: arena theta (radians, 0=+x) → CoT course (degrees CW from north)
            course = math.degrees(-r["theta"]) % 360.0

            # Prefer live RobotStatus fields over the YAML overrides — the
            # registry is authoritative now that team/vehicle_role are
            # propagated via the typed contract. Fall back to YAML overrides
            # for legacy configs that don't set them per-spawn.
            live_role = r.get("vehicle_role") or ""
            live_team = r.get("team") or ""
            role = (
                live_role
                or self._vehicle_roles.get(robot_id)
                or self._default_role
            )

            # Map team to CoT affiliation prefix when role doesn't already
            # encode hostility. team=red → 'a-h-…', team=blue/green → 'a-f-…'.
            base_cot_type = VEHICLE_ROLE_TO_COT_TYPE.get(role, "a-f-G")
            if live_team == "red" and base_cot_type.startswith("a-f"):
                cot_type = "a-h" + base_cot_type[3:]
            elif live_team in ("blue", "green") and base_cot_type.startswith("a-h"):
                cot_type = "a-f" + base_cot_type[3:]
            else:
                cot_type = base_cot_type

            callsign = (
                self._callsigns.get(robot_id)
                or (f"{live_team.upper()}-{robot_id}" if live_team else f"POD-{robot_id}")
            )
            uid = f"platopod-{robot_id}"

            detail = make_contact_detail(callsign)
            detail += make_track_detail(course, 0.0)

            # Include sensor readings if available
            if self._publish_sensor_data:
                with self._sensor_lock:
                    sensors = self._robot_sensors.get(robot_id, {})

                if sensors:
                    # Build a flat readings dict from all sensors
                    readings: dict[str, float] = {}
                    for sensor_name, data in sensors.items():
                        for key, value in data.items():
                            if isinstance(value, (int, float)):
                                readings[f"{sensor_name}.{key}"] = float(value)

                    if readings:
                        detail += make_sensor_detail(readings)

                    # Add remarks with human-readable sensor summary
                    remarks_parts = []
                    gas_data = sensors.get("gas", {})
                    if gas_data:
                        conc = gas_data.get("concentration", 0.0)
                        res = gas_data.get("raw_resistance", 0.0)
                        remarks_parts.append(
                            f"Gas: {conc:.1f} ppm, R={res:.3f}"
                        )
                    if remarks_parts:
                        remarks_text = " | ".join(remarks_parts)
                        detail += f'<remarks>{remarks_text}</remarks>'

            xml = make_cot_event(
                uid=uid,
                cot_type=cot_type,
                lat=lat, lon=lon,
                hae=self._geo.origin_alt,
                ce=0.5,
                le=1.0,
                stale_seconds=self._stale_seconds,
                detail_xml=detail,
            )

            self._transport.send(xml)

    def _publish_arena(self) -> None:
        """Send CoT shape events for arena boundary, zones, and obstacles."""
        # Arena boundary
        if self._arena_boundary:
            geo_points = [
                self._geo.arena_to_latlon(x, y)
                for x, y in self._arena_boundary
            ]
            xml = make_shape_event(
                uid="platopod-arena",
                points=geo_points,
                color="ff00ff00",  # green
                label="Arena",
                stale_seconds=120.0,
            )
            self._transport.send(xml)

        # Scoring zones
        if self._publish_zones:
            for i, zone in enumerate(self._arena_zones):
                if not zone["vertices"]:
                    continue
                geo_points = [
                    self._geo.arena_to_latlon(x, y)
                    for x, y in zone["vertices"]
                ]
                team = zone.get("team", "")
                # Team colours: blue=friendly, red=hostile, default=yellow
                if "blue" in team.lower():
                    color = "663B82F6"  # semi-transparent blue
                elif "red" in team.lower():
                    color = "66EF4444"  # semi-transparent red
                else:
                    color = "66FFFF00"  # semi-transparent yellow
                xml = make_shape_event(
                    uid=f"platopod-zone-{i}",
                    points=geo_points,
                    color=color,
                    label=zone.get("name", f"Zone {i}"),
                    stale_seconds=120.0,
                )
                self._transport.send(xml)

        # Obstacles
        if self._publish_obstacles:
            for i, obs in enumerate(self._arena_obstacles):
                if not obs["vertices"]:
                    continue
                geo_points = [
                    self._geo.arena_to_latlon(x, y)
                    for x, y in obs["vertices"]
                ]
                xml = make_shape_event(
                    uid=f"platopod-obstacle-{i}",
                    points=geo_points,
                    color="ff808080",  # grey
                    label=obs.get("label", f"Obstacle {i}"),
                    stale_seconds=120.0,
                )
                self._transport.send(xml)

    # --- Inbound ---

    def _inbound_loop(self) -> None:
        """Listen for inbound CoT events from ATAK."""
        while self._inbound_running:
            try:
                data, _ = self._inbound_sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                xml_str = data.decode("utf-8")
            except UnicodeDecodeError:
                continue

            parsed = parse_cot_event(xml_str)
            if parsed is None:
                continue

            goal = parse_nav_goal(parsed)
            if goal is not None:
                lat, lon = goal
                x, y = self._geo.latlon_to_arena(lat, lon)

                uid = parsed.get("uid", "")
                robot_id = robot_id_from_uid(uid)

                msg = NavGoal()
                msg.robot_id = robot_id
                msg.x = x
                msg.y = y
                msg.source_uid = uid
                msg.stamp = self.get_clock().now().to_msg()

                self._pub_nav_goal.publish(msg)
                self.get_logger().info(
                    f"Nav goal from ATAK: ({x:.3f}, {y:.3f}) "
                    f"uid={uid} → robot_id={robot_id}"
                )

    def destroy_node(self) -> None:
        self._inbound_running = False
        self._inbound_sock.close()
        self._transport.close()
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod cot_bridge_node."""
    rclpy.init(args=args)
    node = CotBridgeNode()
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
