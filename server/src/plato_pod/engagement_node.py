"""Engagement Node — central evaluator of fire intents.

Subscribes to two fire-intent sources:
  * `/fire_weapon` — string topic with format "actor_id|target_id|weapon"
    used by both opfor_node and the API gateway when a player fires.

For each intent, the node:
  1. Loads the actor and target Robot snapshots from the latest /robots/status.
  2. Runs `engagement.evaluate_fire` with weapons / cover / civilians /
     weather / ROE configured from the exercise YAML.
  3. Calls /registry_node/apply_damage to update the target.
  4. Publishes engagement events on /engagement_events for the API gateway
     to broadcast to WebSocket subscribers and the CoT bridge.

Pure orchestration — all tactical logic lives in plato_pod.engagement,
plato_pod.line_of_sight, plato_pod.roe.
"""

from __future__ import annotations

from random import Random

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from plato_pod_msgs.msg import (
    CivilianList as CivilianListMsg,
    EngagementOutcome as EngagementOutcomeMsg,
    FireIntent as FireIntentMsg,
    Observation as ObservationMsg,
    RobotStatus as RobotStatusMsg,
    RobotStatusList as RobotStatusListMsg,
    RoeRules as RoeRulesMsg,
    WeaponCatalog as WeaponCatalogMsg,
    WeatherState as WeatherStateMsg,
    WorldCover as WorldCoverMsg,
)
from plato_pod_msgs.srv import ApplyDamage
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

_LATCHED_QOS = QoSProfile(
    depth=1, history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

from plato_pod.engagement import Civilian, WeaponSpec, evaluate_fire
from plato_pod.line_of_sight import CoverPolygon
from plato_pod.robot import Robot
from plato_pod.roe import ROERules, check_fire_roe, is_blocking
from plato_pod.weather import CLEAR, WeatherState
from plato_pod.world_state import load_world_state


class EngagementNode(Node):
    """Central node evaluating weapon engagements."""

    def __init__(self) -> None:
        super().__init__("engagement_node")

        _dyn = ParameterDescriptor(dynamic_typing=True)
        for name, default in [
            ("exercise_file", ""),
            ("rng_seed", 0),
        ]:
            self.declare_parameter(name, default, _dyn)

        self._exercise_file = str(self.get_parameter("exercise_file").value)
        seed = int(self.get_parameter("rng_seed").value)
        self._rng = Random(seed)

        # Tactical configuration loaded from YAML
        self._weapons: dict[str, WeaponSpec] = {}
        self._cover_polygons: list[CoverPolygon] = []
        self._civilians: list[Civilian] = []
        self._weather: WeatherState = CLEAR
        self._roe: ROERules = ROERules()
        self._cleared_targets: set[int] = set()

        # Latest robot status indexed by id
        self._units: dict[int, Robot] = {}

        # Bootstrap from YAML so the node has sensible defaults even if
        # world_state_node isn't running. World state subscriptions below
        # take precedence whenever they fire.
        self._load_exercise_fallback()

        # Subscribers — robot status + intent
        self.create_subscription(
            RobotStatusListMsg, "/robots/status",
            self._status_cb, 10,
        )
        self.create_subscription(
            FireIntentMsg, "/fire_weapon", self._fire_cb, 10,
        )
        self.create_subscription(
            ObservationMsg, "/report_observation", self._observation_cb, 10,
        )

        # Subscribers — world state (latched topics from world_state_node).
        # These override the YAML fallback as soon as a sample arrives.
        self.create_subscription(
            WorldCoverMsg, "/world/cover",
            self._cover_cb, _LATCHED_QOS,
        )
        self.create_subscription(
            CivilianListMsg, "/world/civilians",
            self._civilians_cb, _LATCHED_QOS,
        )
        self.create_subscription(
            WeatherStateMsg, "/world/weather",
            self._weather_cb, _LATCHED_QOS,
        )
        self.create_subscription(
            RoeRulesMsg, "/world/roe",
            self._roe_cb, _LATCHED_QOS,
        )
        self.create_subscription(
            WeaponCatalogMsg, "/world/weapons",
            self._weapons_cb, _LATCHED_QOS,
        )

        # Apply damage client
        self._damage_client = self.create_client(
            ApplyDamage, "/registry_node/apply_damage",
        )

        # Outbound publishers
        self._engagement_pub = self.create_publisher(
            EngagementOutcomeMsg, "/engagement_events", 10,
        )

        self.get_logger().info(
            f"Engagement node ready ({len(self._weapons)} weapons, "
            f"{len(self._cover_polygons)} cover polys, "
            f"ROE={self._roe.fire_permission})"
        )

    def _load_exercise_fallback(self) -> None:
        """Bootstrap world state from YAML if world_state_node isn't running.

        World topics override these values when they arrive.
        """
        if not self._exercise_file:
            return
        try:
            ws = load_world_state(self._exercise_file)
        except Exception as e:
            self.get_logger().error(f"Cannot bootstrap from YAML: {e}")
            return
        self._weapons = dict(ws.weapons)
        self._cover_polygons = list(ws.cover)
        self._civilians = [
            Civilian(position=c["position"], label=c.get("label", "civilian"))
            for c in ws.civilians
        ]
        self._weather = ws.weather
        self._roe = ws.roe

    # ---- world-state callbacks (override YAML fallback) -------------------

    def _cover_cb(self, msg: WorldCoverMsg) -> None:
        self._cover_polygons = [
            CoverPolygon(
                vertices=list(zip(p.vertices_x, p.vertices_y)),
                cover_value=float(p.cover_value),
                label=p.label,
            )
            for p in msg.polygons
        ]

    def _civilians_cb(self, msg: CivilianListMsg) -> None:
        self._civilians = [
            Civilian(position=(float(c.x), float(c.y)), label=c.label)
            for c in msg.civilians
        ]

    def _weather_cb(self, msg: WeatherStateMsg) -> None:
        self._weather = WeatherState(
            visibility_m=float(msg.visibility_m),
            wind_speed=float(msg.wind_speed),
            wind_direction=float(msg.wind_direction),
            fog_density=float(msg.fog_density),
            time_of_day=float(msg.time_of_day),
        )

    def _roe_cb(self, msg: RoeRulesMsg) -> None:
        self._roe = ROERules(
            fire_permission=msg.fire_permission,
            civilian_proximity_m=float(msg.civilian_proximity_m),
            require_target_id=bool(msg.require_target_id),
            friendly_fire_severity=msg.friendly_fire_severity,
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

    # ---- callbacks --------------------------------------------------------

    def _status_cb(self, msg: RobotStatusListMsg) -> None:
        new: dict[int, Robot] = {}
        for r in msg.robots:
            new[r.robot_id] = self._msg_to_robot(r)
        self._units = new

    def _observation_cb(self, msg: ObservationMsg) -> None:
        if msg.classification.strip().lower() == "hostile":
            self._cleared_targets.add(int(msg.target_id))
            self.get_logger().info(
                f"Target {msg.target_id} cleared as hostile by {msg.actor_id}"
            )

    def _fire_cb(self, msg: FireIntentMsg) -> None:
        """Handle a typed FireIntent message."""
        actor_id = int(msg.actor_id)
        target_id = int(msg.target_id)
        weapon_name = msg.weapon

        weapon = self._weapons.get(weapon_name)
        if weapon is None:
            self.get_logger().warning(
                f"Unknown weapon '{weapon_name}' — skipping engagement"
            )
            self._publish_outcome(self._make_blocked_outcome(
                msg, fired=False, rationale="unknown_weapon",
            ))
            return

        actor = self._units.get(actor_id) if actor_id >= 0 else None
        target = self._units.get(target_id) if target_id >= 0 else None
        if target is None and not msg.has_target_position:
            self.get_logger().warning(
                f"Target {target_id} not in registry and no target_position — "
                f"skipping engagement"
            )
            self._publish_outcome(self._make_blocked_outcome(
                msg, fired=False, rationale="unknown_target",
            ))
            return

        target_position = (
            (target.x, target.y) if target is not None
            else (float(msg.target_x), float(msg.target_y))
        )

        # ROE check
        violations = check_fire_roe(
            actor=actor,
            target_position=target_position,
            target=target,
            rules=self._roe,
            civilians=self._civilians,
            cleared_targets=self._cleared_targets,
        )
        if is_blocking(violations):
            self._publish_outcome(self._make_blocked_outcome(
                msg, fired=False, rationale="roe_blocked",
                roe_violations=[v.rule for v in violations],
            ))
            return

        # Indirect fire path: no actor, target may be None (position-only)
        outcome = evaluate_fire(
            actor=actor,
            target=target,
            weapon=weapon,
            cover_polygons=self._cover_polygons,
            weather=self._weather,
            civilians=self._civilians,
            other_units=list(self._units.values()),
            rng=self._rng,
        )

        if outcome.hit and outcome.damage > 0 and target is not None:
            self._apply_damage_async(target.robot_id, outcome.damage, "engagement")

        out = EngagementOutcomeMsg()
        out.actor_id = actor_id
        out.target_id = target_id
        out.weapon = weapon_name
        out.source_seq = int(msg.source_seq)
        out.fired = bool(outcome.fired)
        out.hit = bool(outcome.hit)
        out.blocked = bool(outcome.blocked)
        out.damage = float(outcome.damage)
        out.distance_m = float(outcome.distance_m)
        out.effective_pok = float(outcome.effective_pok)
        out.rationale = outcome.rationale
        out.civilian_violation = bool(outcome.civilian_violation)
        out.civilians_at_risk = list(outcome.civilians_at_risk)
        out.suppressed_targets = list(outcome.suppressed_targets)
        out.roe_violations = [v.rule for v in violations]
        self._publish_outcome(out)

    # ---- helpers ----------------------------------------------------------

    @staticmethod
    def _msg_to_robot(msg: RobotStatusMsg) -> Robot:
        return Robot(
            robot_id=msg.robot_id,
            deployment=msg.deployment,
            x=float(msg.x), y=float(msg.y), theta=float(msg.theta),
            radius=float(msg.radius),
            status=msg.status,
            team=(msg.team or None),
            vehicle_role=(msg.vehicle_role or "default"),
            health=float(msg.health) if msg.health > 0.0 else 1.0,
            weapons=list(msg.weapons),
            thermal_signature=(float(msg.thermal_signature)
                                if msg.thermal_signature > 0.0 else 1.0),
        )

    def _publish_outcome(self, outcome: EngagementOutcomeMsg) -> None:
        self._engagement_pub.publish(outcome)

    @staticmethod
    def _make_blocked_outcome(
        intent: FireIntentMsg, *,
        fired: bool, rationale: str,
        roe_violations: list[str] | None = None,
    ) -> EngagementOutcomeMsg:
        out = EngagementOutcomeMsg()
        out.actor_id = int(intent.actor_id)
        out.target_id = int(intent.target_id)
        out.weapon = intent.weapon
        out.source_seq = int(intent.source_seq)
        out.fired = fired
        out.hit = False
        out.blocked = True
        out.damage = 0.0
        out.distance_m = 0.0
        out.effective_pok = 0.0
        out.rationale = rationale
        out.civilian_violation = False
        out.civilians_at_risk = []
        out.suppressed_targets = []
        out.roe_violations = roe_violations or []
        return out

    def _apply_damage_async(self, robot_id: int, damage: float, reason: str) -> None:
        if not self._damage_client.service_is_ready():
            self.get_logger().warning(
                f"apply_damage service not ready — damage to {robot_id} dropped"
            )
            return
        req = ApplyDamage.Request()
        req.robot_id = robot_id
        req.damage = float(damage)
        req.reason = reason
        self._damage_client.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EngagementNode()
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
