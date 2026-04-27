"""OPFOR Node — autonomous virtual hostile units driven by behaviour FSM.

Loads opfor unit definitions from the exercise YAML, spawns each as a virtual
robot via the registry service, then runs `behavior.BehaviorTree.tick()` for
each unit on a fixed cadence. Resulting cmd_vel and fire intents are published
to the standard topics so the rest of the platform sees OPFOR units as
ordinary virtual robots.

This is a thin shell around `plato_pod.behavior` — all tactical decisions live
in the pure-Python module for testability.
"""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from plato_pod_msgs.msg import (
    ArenaModel as ArenaModelMsg,
    FireIntent as FireIntentMsg,
    RobotStatusList as RobotStatusListMsg,
    WeaponCatalog as WeaponCatalogMsg,
    WeatherState as WeatherStateMsg,
    WorldCover as WorldCoverMsg,
)
from plato_pod_msgs.srv import SpawnVirtual
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

_LATCHED_QOS = QoSProfile(
    depth=1, history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

from plato_pod.behavior import (
    BehaviorConfig,
    BehaviorTree,
    WorldState,
    behavior_config_from_dict,
)
from plato_pod.engagement import WeaponSpec
from plato_pod.line_of_sight import CoverPolygon
from plato_pod.robot import Robot
from plato_pod.weather import CLEAR, WeatherState
from plato_pod.world_state import load_world_state


@dataclass
class _OpforUnit:
    """Internal record for a single managed OPFOR unit."""
    robot_id: int             # set after registry spawn
    config: BehaviorConfig
    tree: BehaviorTree
    cmd_vel_pub: object       # ROS2 publisher (typed loosely to avoid stub deps)
    fire_pub: object


class OpforNode(Node):
    """ROS2 node spawning and ticking virtual hostile units."""

    def __init__(self) -> None:
        super().__init__("opfor_node")

        _dyn = ParameterDescriptor(dynamic_typing=True)
        for name, default in [
            ("exercise_file", ""),
            ("tick_rate_hz", 10.0),
            ("default_team", "red"),
        ]:
            self.declare_parameter(name, default, _dyn)

        self._exercise_file = str(self.get_parameter("exercise_file").value)
        tick_rate = float(self.get_parameter("tick_rate_hz").value)
        self._default_team = str(self.get_parameter("default_team").value)
        self._dt = 1.0 / tick_rate

        # World state caches populated by callbacks
        self._all_units: dict[int, Robot] = {}
        self._cover_polygons: list[CoverPolygon] = []
        self._weather: WeatherState = CLEAR
        self._weapons: dict[str, WeaponSpec] = {}
        self._arena_bounds: tuple[float, float, float, float] | None = None
        self._arena_ready: bool = False

        # OPFOR units indexed by their assigned robot_id
        self._units: dict[int, _OpforUnit] = {}

        # Pending unit specs awaiting spawn (need the arena boundary first)
        self._pending_specs: list[dict] = []

        # Subscribers
        self.create_subscription(
            RobotStatusListMsg, "/robots/status",
            self._robot_status_callback, 10,
        )
        self.create_subscription(
            ArenaModelMsg, "/arena/model",
            self._arena_model_callback, 10,
        )

        # Spawn service client
        self._spawn_client = self.create_client(
            SpawnVirtual, "/registry_node/spawn_virtual",
        )

        # Fire intent publisher (typed message, engagement_node consumes)
        self._fire_intent_pub = self.create_publisher(
            FireIntentMsg, "/fire_weapon", 10,
        )
        self._next_fire_seq: int = 0

        # World-state subscriptions (latched). These supply cover, weather,
        # and the weapons catalog. Override the YAML fallback as soon as
        # they fire.
        self.create_subscription(
            WorldCoverMsg, "/world/cover",
            self._cover_cb, _LATCHED_QOS,
        )
        self.create_subscription(
            WeatherStateMsg, "/world/weather",
            self._weather_cb, _LATCHED_QOS,
        )
        self.create_subscription(
            WeaponCatalogMsg, "/world/weapons",
            self._weapons_cb, _LATCHED_QOS,
        )

        self._load_exercise()

        # Tick timer
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"OPFOR node ready ({tick_rate:.1f} Hz, "
            f"{len(self._pending_specs)} units to spawn)"
        )

    # ---- exercise loading -------------------------------------------------

    def _load_exercise(self) -> None:
        """Bootstrap world data from YAML; load OPFOR unit specs.

        World data (weapons, weather, cover) is later overridden by the
        latched topics from world_state_node when those samples arrive.
        OPFOR unit specs aren't part of WorldState so they stay here.
        """
        if not self._exercise_file:
            self.get_logger().info("No exercise_file — OPFOR idle")
            return

        # Bootstrap world data from YAML
        try:
            ws = load_world_state(self._exercise_file)
            self._weapons = dict(ws.weapons)
            self._cover_polygons = list(ws.cover)
            self._weather = ws.weather
        except Exception as e:
            self.get_logger().warning(f"Cannot bootstrap world from YAML: {e}")

        # OPFOR unit specs (not part of WorldState)
        try:
            import yaml
            with open(self._exercise_file) as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Cannot load OPFOR specs: {e}")
            return

        exercise = config.get("exercise", config)
        for spec in (exercise.get("opfor", []) or []):
            self._pending_specs.append(spec)

    # ---- world-state callbacks --------------------------------------------

    def _cover_cb(self, msg: WorldCoverMsg) -> None:
        self._cover_polygons = [
            CoverPolygon(
                vertices=list(zip(p.vertices_x, p.vertices_y)),
                cover_value=float(p.cover_value),
                label=p.label,
            )
            for p in msg.polygons
        ]

    def _weather_cb(self, msg: WeatherStateMsg) -> None:
        self._weather = WeatherState(
            visibility_m=float(msg.visibility_m),
            wind_speed=float(msg.wind_speed),
            wind_direction=float(msg.wind_direction),
            fog_density=float(msg.fog_density),
            time_of_day=float(msg.time_of_day),
        )

    def _weapons_cb(self, msg: WeaponCatalogMsg) -> None:
        self._weapons = {
            w.name: WeaponSpec(
                name=w.name,
                max_range_m=float(w.max_range_m),
                base_pok_at_100m=float(w.base_pok_at_100m),
                falloff_per_100m=float(w.falloff_per_100m),
                damage=float(w.damage),
                ammo_capacity=int(w.ammo_capacity),
                suppress_radius_m=float(w.suppress_radius_m),
            )
            for w in msg.weapons
        }

    # ---- ROS2 callbacks ---------------------------------------------------

    def _arena_model_callback(self, msg: ArenaModelMsg) -> None:
        if msg.boundary_valid and msg.boundary_x:
            xs = [float(x) for x in msg.boundary_x]
            ys = [float(y) for y in msg.boundary_y]
            self._arena_bounds = (min(xs), min(ys), max(xs), max(ys))
        self._arena_ready = True
        self._maybe_spawn_pending()

    def _robot_status_callback(self, msg: RobotStatusListMsg) -> None:
        # Rebuild the unit table each tick for simplicity (registry list is small)
        new_table: dict[int, Robot] = {}
        for r in msg.robots:
            new_table[r.robot_id] = Robot(
                robot_id=r.robot_id,
                deployment=r.deployment,
                x=float(r.x), y=float(r.y), theta=float(r.theta),
                radius=float(r.radius),
                status=r.status,
                team=(r.team or None),
                vehicle_role=(r.vehicle_role or "default"),
                health=float(r.health) if r.health > 0.0 else 1.0,
                weapons=list(r.weapons),
                thermal_signature=(float(r.thermal_signature)
                                    if r.thermal_signature > 0.0 else 1.0),
            )
        self._all_units = new_table

    # ---- spawning ---------------------------------------------------------

    def _maybe_spawn_pending(self) -> None:
        """Spawn any pending OPFOR units once arena boundary is known."""
        if not self._arena_ready or not self._pending_specs:
            return
        if not self._spawn_client.service_is_ready():
            return

        for spec in self._pending_specs:
            self._spawn_one(spec)
        self._pending_specs.clear()

    def _spawn_one(self, spec: dict) -> None:
        """Issue a SpawnVirtual call and register a BehaviorTree on response."""
        try:
            position = spec.get("position", [0.0, 0.0])
            x = float(position[0])
            y = float(position[1])
            theta = float(spec.get("theta", 0.0))
            radius = float(spec.get("radius", 0.1))
            team = str(spec.get("team", self._default_team))
            vehicle_role = str(spec.get("vehicle_role", "hostile"))
            weapons = list(spec.get("weapons", []))
        except Exception as e:
            self.get_logger().warning(f"Bad opfor spec, skipping: {e}")
            return

        req = SpawnVirtual.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.radius = radius
        req.team = team
        req.vehicle_role = vehicle_role
        req.health = 1.0
        req.weapons = weapons

        cfg = behavior_config_from_dict(spec)
        # Make sure config knows what counts as enemy: every team that isn't ours
        if "enemy_teams" not in spec:
            cfg.enemy_teams = [t for t in ("blue", "red", "green") if t != team]
        if cfg.weapon is None and weapons:
            cfg.weapon = weapons[0]

        future = self._spawn_client.call_async(req)
        future.add_done_callback(
            lambda f, _cfg=cfg: self._on_spawn_response(f, _cfg)
        )

    def _on_spawn_response(self, future, cfg: BehaviorConfig) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Spawn call failed: {e}")
            return
        if not response.success:
            self.get_logger().warning(
                f"OPFOR spawn rejected: {response.message}"
            )
            return

        rid = int(response.robot_id)
        cmd_pub = self.create_publisher(Twist, f"/robot_{rid}/cmd_vel", 10)
        fire_pub = self._fire_intent_pub
        self._units[rid] = _OpforUnit(
            robot_id=rid, config=cfg,
            tree=BehaviorTree(cfg),
            cmd_vel_pub=cmd_pub, fire_pub=fire_pub,
        )
        self.get_logger().info(
            f"OPFOR unit registered: id={rid} behavior={cfg.behavior}"
        )

    # ---- main tick --------------------------------------------------------

    def _tick(self) -> None:
        if not self._units:
            return

        world = WorldState(
            all_units=list(self._all_units.values()),
            cover_polygons=self._cover_polygons,
            weather=self._weather,
            weapons=self._weapons,
            arena_bounds=self._arena_bounds,
        )

        for rid, unit in self._units.items():
            robot = self._all_units.get(rid)
            if robot is None:
                continue   # registry hasn't seen us yet
            action = unit.tree.tick(robot, world)

            twist = Twist()
            twist.linear.x = float(action.cmd_vel[0])
            twist.angular.z = float(action.cmd_vel[1])
            unit.cmd_vel_pub.publish(twist)

            if action.fire_weapon is not None:
                target_id, weapon_name = action.fire_weapon
                intent = FireIntentMsg()
                intent.actor_id = rid
                intent.target_id = int(target_id)
                intent.weapon = weapon_name
                intent.has_target_position = False
                intent.target_x = 0.0
                intent.target_y = 0.0
                self._next_fire_seq += 1
                intent.source_seq = self._next_fire_seq
                unit.fire_pub.publish(intent)


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod opfor_node."""
    rclpy.init(args=args)
    node = OpforNode()
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
