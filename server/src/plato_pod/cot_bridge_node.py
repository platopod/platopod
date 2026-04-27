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
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

_LATCHED_QOS = QoSProfile(
    depth=1, history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

from plato_pod_msgs.msg import (
    ArenaModel as ArenaModelMsg,
    CivilianList as CivilianListMsg,
    EngagementOutcome as EngagementOutcomeMsg,
    IedZoneList as IedZoneListMsg,
    NavGoal,
    RobotStatusList as RobotStatusListMsg,
    SensorReading as SensorReadingMsg,
)

from plato_pod.cot_protocol import (
    VEHICLE_ROLE_TO_COT_TYPE,
    make_casualty_event,
    make_civilian_marker,
    make_contact_detail,
    make_cot_event,
    make_engagement_event,
    make_ied_marker,
    make_plume_contour_event,
    make_plume_ellipse_event,
    make_sensor_detail,
    make_shape_event,
    make_tombstone_event,
    make_track_detail,
    parse_cot_event,
    parse_nav_goal,
    robot_id_from_uid,
)
from plato_pod.cot_transport import CotTransport, create_transport
from plato_pod.geo_reference import GeoReference
from plato_pod.plume_contour import extract_contours, fit_ellipse
from plato_pod.virtual_layer_loader import load_virtual_layers


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
        # World-state caches (filled from latched /world/* topics)
        self._civilians: list[dict] = []     # {x, y, label}
        self._ied_zones: list[dict] = []     # {x, y, detect_radius, label}
        # Track UIDs we've published per category so we can tombstone
        # any that disappear (scenario switch, runtime deletion, etc.).
        self._published_civilian_uids: set[str] = set()
        self._published_ied_uids: set[str] = set()
        self._published_plume_uids: set[str] = set()
        self._robot_lock = threading.Lock()
        self._sensor_lock = threading.Lock()
        self._robot_sensors: dict[int, dict[str, dict]] = {}
        # robot_id -> {sensor_name -> latest reading dict}

        # Vehicle role and callsign overrides — loaded from exercise YAML
        self._vehicle_roles: dict[int, str] = {}
        self._callsigns: dict[int, str] = {}

        # Spatial fields (gas plume, etc.) loaded from exercise YAML —
        # used to render contamination contours to ATAK.
        self._spatial_env = None    # EnvironmentContext | None
        self._plume_field_name: str = "gas"
        self._plume_thresholds: list[float] = [10.0, 100.0, 500.0]
        self._plume_colors: list[str] = []   # parallel to _plume_thresholds; "" → auto
        self._plume_grid_size: int = 80
        self._plume_simplify_m: float = 0.0
        self._plume_smooth_m: float = 0.0
        self._plume_max_polygons: int = 3
        self._plume_render: str = "polygon"   # "polygon" | "ellipse"

        exercise_file = str(self.get_parameter("exercise_file").value)
        if exercise_file:
            self._load_roles_from_exercise(exercise_file)
            self._load_spatial_env(exercise_file)

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

        # World-state subscriptions (latched, from world_state_node)
        self.create_subscription(
            CivilianListMsg, "/world/civilians",
            self._civilians_callback, _LATCHED_QOS,
        )
        self.create_subscription(
            IedZoneListMsg, "/world/ied_zones",
            self._ied_zones_callback, _LATCHED_QOS,
        )

        # Publisher for inbound nav goals
        self._pub_nav_goal = self.create_publisher(NavGoal, "/cot/nav_goal", 10)

        # Outbound timers
        publish_period = 1.0 / publish_rate
        self._pose_timer = self.create_timer(publish_period, self._publish_robots)
        self._arena_timer = self.create_timer(arena_republish, self._publish_arena)
        # Civilians + IEDs republish on the same cadence as the arena
        # so iTAK keeps the markers fresh and they don't go stale.
        self._world_timer = self.create_timer(
            arena_republish, self._publish_world_entities,
        )
        # Plume contours refresh more often — concentration evolves on the
        # order of seconds, especially for analytic Gaussian plumes that
        # depend on `t` for time-of-flight from the source.
        self._plume_timer = self.create_timer(5.0, self._publish_plume_contours)
        self._exercise_start_time = time.time()

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

    def _load_spatial_env(self, exercise_file: str) -> None:
        """Load spatial fields (gas plumes, etc.) from exercise YAML.

        Used to render CBRN contamination contours on ATAK. Optional —
        when the exercise has no virtual_layers, this is a no-op.

        The plume rendering parameters can be overridden in the exercise
        YAML under `cot_bridge:`:
          cot_bridge:
            plume_field: "gas"
            plume_thresholds: [10, 100, 500]
            plume_grid_size: 60
            plume_simplify_m: 0.5
        """
        try:
            import yaml
            with open(exercise_file) as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warning(f"Cannot read exercise for plume: {e}")
            return

        try:
            self._spatial_env = load_virtual_layers(config)
        except Exception as e:
            self.get_logger().warning(f"Cannot load virtual layers: {e}")
            return

        if self._spatial_env is None:
            return

        bridge_cfg = (config.get("exercise", config) or {}).get("cot_bridge", {}) or {}
        self._plume_field_name = str(bridge_cfg.get("plume_field", "gas"))
        thresholds = bridge_cfg.get("plume_thresholds")
        if isinstance(thresholds, list) and thresholds:
            self._plume_thresholds = [float(t) for t in thresholds]
        # Optional per-threshold ARGB colours; empty string ⇒ doctrine default
        colors = bridge_cfg.get("plume_colors")
        if isinstance(colors, list):
            self._plume_colors = [str(c) for c in colors]
        self._plume_grid_size = int(bridge_cfg.get("plume_grid_size", 80))
        self._plume_simplify_m = float(bridge_cfg.get("plume_simplify_m", 0.0))
        self._plume_smooth_m = float(bridge_cfg.get("plume_smooth_m", 0.0))
        self._plume_max_polygons = int(bridge_cfg.get("plume_max_polygons", 3))
        self._plume_render = str(bridge_cfg.get("plume_render", "polygon")).lower()

        if self._plume_field_name in self._spatial_env.fields:
            self.get_logger().info(
                f"Plume field '{self._plume_field_name}' loaded; "
                f"thresholds={self._plume_thresholds}"
            )

    # --- Callbacks ---

    def _robot_status_callback(self, msg: RobotStatusListMsg) -> None:
        """Cache latest robot statuses and subscribe to new sensor topics.

        We keep wounded/destroyed/incapacitated robots in the cache so the
        casualty marker can derive the correct callsign (RED-8, not BOT-8)
        and so the operator dashboard can show the body's last position.
        Filtering by status happens at publish time instead.
        """
        current_active_ids: set[int] = set()
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
                    "health": r.health,
                }
                for r in msg.robots
                if r.status != "inactive"
            ]
            current_active_ids = {
                r["robot_id"] for r in self._robots if r["status"] == "active"
            }

        # Subscribe to sensor topics for new active robots only
        if self._publish_sensor_data:
            for rid in current_active_ids - self._subscribed_robots:
                self._subscribe_robot_sensors(rid)
            self._subscribed_robots = current_active_ids

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

    def _civilians_callback(self, msg: CivilianListMsg) -> None:
        """Cache civilian positions from world_state_node (latched topic)."""
        self._civilians = [
            {"x": float(c.x), "y": float(c.y), "label": c.label or "civilian"}
            for c in msg.civilians
        ]
        self.get_logger().info(
            f"Civilians updated: {len(self._civilians)} entities"
        )
        # Publish immediately so iTAK shows them right away rather than
        # waiting for the next world timer tick.
        self._publish_world_entities()

    def _ied_zones_callback(self, msg: IedZoneListMsg) -> None:
        """Cache IED hazards from world_state_node (latched topic)."""
        self._ied_zones = [
            {
                "x": float(z.x), "y": float(z.y),
                "detect_radius_m": float(z.detectability_radius_m),
                "label": z.label or "ied",
            }
            for z in msg.zones
        ]
        self.get_logger().info(
            f"IED zones updated: {len(self._ied_zones)} entities"
        )
        self._publish_world_entities()

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
            # Resolve the live callsign so casualty shows e.g. RED-8, not BOT-8
            _ctype, casualty_callsign = self._resolve_identity(
                int(target),
                tgt.get("team") or "",
                tgt.get("vehicle_role") or "",
            )
            try:
                xml = make_casualty_event(
                    robot_uid=f"platopod-{target}-cas",
                    callsign=casualty_callsign,
                    lat=lat, lon=lon,
                    status=status,
                    health=new_health,
                )
                self._transport.send(xml)
            except Exception as e:
                self.get_logger().warning(f"Failed to send casualty CoT: {e}")

    # --- Outbound publishing ---

    def _resolve_identity(
        self, robot_id: int, live_team: str, live_role: str,
    ) -> tuple[str, str]:
        """Return (cot_type, callsign) for a robot, preferring live fields.

        Used both by the live-unit publisher and by the casualty marker so
        a destroyed RED-8 still gets the 'RED-8' callsign and an a-h-G-X
        type instead of a generic BOT-8 / a-f-G fallback.
        """
        role = (
            live_role
            or self._vehicle_roles.get(robot_id)
            or self._default_role
        )
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
        return cot_type, callsign

    def _publish_robots(self) -> None:
        """Send CoT events for active robots with sensor data.

        Non-active robots are kept in the cache so the casualty marker
        can resolve their callsign / affiliation, but we don't redraw a
        live symbol for them — only the casualty marker remains.
        """
        with self._robot_lock:
            robots = [r for r in self._robots if r["status"] == "active"]

        for r in robots:
            robot_id = r["robot_id"]
            lat, lon = self._geo.arena_to_latlon(r["x"], r["y"])

            # Heading: arena theta (radians, 0=+x) → CoT course (degrees CW from north)
            course = math.degrees(-r["theta"]) % 360.0

            cot_type, callsign = self._resolve_identity(
                robot_id, r.get("team") or "", r.get("vehicle_role") or "",
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

    def _publish_plume_contours(self) -> None:
        """Sample the gas plume field, contour it, and emit CoT shapes.

        One nested polygon per threshold (low → wide yellow zone, high →
        narrow red core). The arena bounding box is the contour domain;
        if no boundary is known yet we skip this tick.
        """
        if self._spatial_env is None:
            self.get_logger().info(
                "Plume tick skipped: no spatial environment loaded"
                " (exercise_file param empty or virtual_layers missing)",
                throttle_duration_sec=10.0,
            )
            return
        if self._plume_field_name not in self._spatial_env.fields:
            self.get_logger().info(
                f"Plume tick skipped: field '{self._plume_field_name}' "
                f"not found (available: {list(self._spatial_env.fields)})",
                throttle_duration_sec=10.0,
            )
            return
        if not self._arena_boundary:
            self.get_logger().info(
                "Plume tick skipped: no arena boundary received from "
                "/arena/model (is arena_model_node running?)",
                throttle_duration_sec=10.0,
            )
            return
        field = self._spatial_env.fields[self._plume_field_name]

        xs = [p[0] for p in self._arena_boundary]
        ys = [p[1] for p in self._arena_boundary]
        bbox = (min(xs), min(ys), max(xs), max(ys))

        # Time argument for fields that depend on it (e.g. transient
        # dispersion). Plain Gaussian plumes ignore `t`.
        t = time.time() - self._exercise_start_time

        try:
            levels = extract_contours(
                field, bbox,
                thresholds=self._plume_thresholds,
                grid_size=self._plume_grid_size,
                time=t,
                simplify_tolerance=self._plume_simplify_m,
                smooth_amount=self._plume_smooth_m,
                max_polygons_per_level=self._plume_max_polygons,
            )
        except Exception as e:
            self.get_logger().warning(f"Contour extraction failed: {e}")
            return

        new_plume_uids: set[str] = set()
        for level_idx, level in enumerate(levels):
            override_color = (
                self._plume_colors[level_idx]
                if level_idx < len(self._plume_colors)
                and self._plume_colors[level_idx]
                else None
            )
            for i, polygon in enumerate(level.polygons):
                # Stable UID per (threshold, component) so iTAK updates
                # the existing shape rather than spawning duplicates.
                uid = (
                    f"platopod-plume-{self._plume_field_name}-"
                    f"{int(level.threshold)}-{i}"
                )
                label = f"{self._plume_field_name} ≥{level.threshold:.0f}"

                xml = self._render_plume_polygon(
                    uid, polygon, level.threshold, label, override_color,
                )
                if xml is None:
                    continue
                new_plume_uids.add(uid)
                try:
                    self._transport.send(xml)
                except Exception as e:
                    self.get_logger().warning(
                        f"Failed to send plume contour: {e}"
                    )

        # Tombstone any plume UIDs that disappeared since the last tick —
        # e.g., a contour shrinking below threshold, fragments merging,
        # or the operator switching from polygon to ellipse rendering.
        self._send_tombstones(self._published_plume_uids - new_plume_uids)
        self._published_plume_uids = new_plume_uids

        if new_plume_uids:
            self.get_logger().info(
                f"Plume tick published {len(new_plume_uids)} contour(s) "
                f"({self._plume_render} mode): {sorted(new_plume_uids)}",
                throttle_duration_sec=15.0,
            )
        else:
            self.get_logger().info(
                "Plume tick produced 0 contours — field below all thresholds "
                "or extraction failed",
                throttle_duration_sec=15.0,
            )

    def _render_plume_polygon(
        self, uid: str, polygon: list[tuple[float, float]],
        threshold: float, label: str, override_color: str | None,
    ) -> str | None:
        """Convert one polygon to a CoT event using the configured renderer.

        Polygon vertices are in arena metres; this method handles the
        geo-conversion and dispatches by self._plume_render mode.
        """
        if self._plume_render == "ellipse":
            ell = fit_ellipse(polygon)
            if ell is None:
                return None
            cx, cy, major_m, minor_m, angle_rad = ell
            try:
                clat, clon = self._geo.arena_to_latlon(cx, cy)
            except Exception as e:
                self.get_logger().warning(
                    f"Geo conversion for plume ellipse failed: {e}"
                )
                return None
            # Arena-metre axes scaled by GeoReference.scale_factor; rotate
            # by north-vs-+x convention. The geo_reference uses +x = east,
            # +y = north, so an angle measured CCW from +x equals the
            # standard math angle. CoT angle is CW from north → 90 - angle.
            scale = float(getattr(self._geo, "scale_factor", 1.0))
            angle_deg_cot = (90.0 - math.degrees(angle_rad)) % 360.0
            return make_plume_ellipse_event(
                uid=uid,
                center_lat=clat, center_lon=clon,
                major_axis_m=major_m * scale,
                minor_axis_m=minor_m * scale,
                angle_deg=angle_deg_cot,
                threshold_value=float(threshold),
                label=label,
                color=override_color,
                stale_seconds=15.0,
            )

        # Default: polygon renderer
        try:
            geo_points = [
                self._geo.arena_to_latlon(x, y) for x, y in polygon
            ]
        except Exception as e:
            self.get_logger().warning(
                f"Geo conversion for plume contour failed: {e}"
            )
            return None
        return make_plume_contour_event(
            uid=uid,
            points=geo_points,
            threshold_value=float(threshold),
            label=label,
            color=override_color,
            stale_seconds=15.0,
        )

    def _send_tombstones(self, uids: set[str]) -> None:
        """Tell ATAK/iTAK to forget each UID."""
        for uid in uids:
            try:
                self._transport.send(make_tombstone_event(uid))
            except Exception as e:
                self.get_logger().warning(
                    f"Failed to send tombstone for {uid}: {e}"
                )
        if uids:
            self.get_logger().info(
                f"Tombstoned {len(uids)} CoT UIDs: {sorted(uids)[:5]}"
                + ("…" if len(uids) > 5 else "")
            )

    def _publish_world_entities(self) -> None:
        """Periodically emit CoT markers for civilians and IED hazards.

        Each entity gets a stable UID derived from its label so iTAK
        updates the existing marker rather than spawning duplicates.
        UIDs that were published in a previous tick but no longer appear
        in the world state get a tombstone event so iTAK drops them —
        important when scenarios change at runtime.
        """
        # Civilians (neutral, a-n-G)
        new_civ_uids: set[str] = set()
        for i, c in enumerate(self._civilians):
            label = str(c.get("label") or f"civilian_{i}")
            uid = f"platopod-civ-{label}"
            new_civ_uids.add(uid)
            try:
                lat, lon = self._geo.arena_to_latlon(c["x"], c["y"])
                xml = make_civilian_marker(
                    uid=uid, lat=lat, lon=lon,
                    label=label,
                    stale_seconds=600.0,
                )
                self._transport.send(xml)
            except Exception as e:
                self.get_logger().warning(
                    f"Failed to send civilian marker: {e}"
                )
        # Tombstone civilians that disappeared since last tick
        self._send_tombstones(self._published_civilian_uids - new_civ_uids)
        self._published_civilian_uids = new_civ_uids

        # IED hazards (CBRN drawing, u-d-c-c)
        new_ied_uids: set[str] = set()
        for i, z in enumerate(self._ied_zones):
            label = str(z.get("label") or f"ied_{i}")
            uid = f"platopod-ied-{label}"
            new_ied_uids.add(uid)
            try:
                lat, lon = self._geo.arena_to_latlon(z["x"], z["y"])
                xml = make_ied_marker(
                    uid=uid, lat=lat, lon=lon,
                    confidence=0.0,
                    label=label,
                    stale_seconds=3600.0,
                )
                self._transport.send(xml)
            except Exception as e:
                self.get_logger().warning(
                    f"Failed to send IED marker: {e}"
                )
        self._send_tombstones(self._published_ied_uids - new_ied_uids)
        self._published_ied_uids = new_ied_uids

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
