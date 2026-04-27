"""World State Node — single source of truth for the tactical world.

Loads the exercise YAML once at startup, parses every section that other
nodes used to re-parse independently (cover polygons, civilians, IEDs, EW
emitters, jamming zones, dead zones, weather, ROE, weapon catalog), and
publishes them as latched topics so any node in any language can subscribe.

Runtime mutation (weather change, civilian appearance, ROE update) is
handled via the `~/inject_event` service so the world state remains the
single source consumers see.

Pure orchestration — schema parsing lives in plato_pod.world_state.
"""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from plato_pod_msgs.msg import (
    Civilian as CivilianMsg,
    CivilianList as CivilianListMsg,
    CoverPolygon as CoverPolygonMsg,
    DeadZone as DeadZoneMsg,
    DeadZoneList as DeadZoneListMsg,
    EwEmitter as EwEmitterMsg,
    EwEmitterList as EwEmitterListMsg,
    IedZone as IedZoneMsg,
    IedZoneList as IedZoneListMsg,
    JammingZone as JammingZoneMsg,
    JammingZoneList as JammingZoneListMsg,
    RoeRules as RoeRulesMsg,
    WeaponCatalog as WeaponCatalogMsg,
    WeaponSpec as WeaponSpecMsg,
    WeatherState as WeatherStateMsg,
    WorldCover as WorldCoverMsg,
)

from plato_pod.world_state import WorldState, load_world_state


# Latched-equivalent QoS: deliver the last sample to late subscribers.
LATCHED_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class WorldStateNode(Node):
    """ROS2 node owning the tactical world definition."""

    def __init__(self) -> None:
        super().__init__("world_state_node")

        _dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("exercise_file", "", _dyn)

        self._exercise_file = str(self.get_parameter("exercise_file").value)

        # Latched publishers
        self._pub_cover = self.create_publisher(
            WorldCoverMsg, "/world/cover", LATCHED_QOS,
        )
        self._pub_civilians = self.create_publisher(
            CivilianListMsg, "/world/civilians", LATCHED_QOS,
        )
        self._pub_ied = self.create_publisher(
            IedZoneListMsg, "/world/ied_zones", LATCHED_QOS,
        )
        self._pub_ew = self.create_publisher(
            EwEmitterListMsg, "/world/ew_emitters", LATCHED_QOS,
        )
        self._pub_jamming = self.create_publisher(
            JammingZoneListMsg, "/world/jamming", LATCHED_QOS,
        )
        self._pub_dead = self.create_publisher(
            DeadZoneListMsg, "/world/dead_zones", LATCHED_QOS,
        )
        self._pub_weather = self.create_publisher(
            WeatherStateMsg, "/world/weather", LATCHED_QOS,
        )
        self._pub_roe = self.create_publisher(
            RoeRulesMsg, "/world/roe", LATCHED_QOS,
        )
        self._pub_weapons = self.create_publisher(
            WeaponCatalogMsg, "/world/weapons", LATCHED_QOS,
        )

        self._world: WorldState = WorldState()
        self._load_and_publish()

        self.get_logger().info(
            f"World state node ready — "
            f"{len(self._world.cover)} cover, "
            f"{len(self._world.civilians)} civ, "
            f"{len(self._world.ied_zones)} IED, "
            f"{len(self._world.weapons)} weapons"
        )

    def _load_and_publish(self) -> None:
        if self._exercise_file:
            try:
                self._world = load_world_state(self._exercise_file)
            except Exception as e:
                self.get_logger().error(f"Cannot load world state: {e}")
                self._world = WorldState()
        self._publish_all()

    def _publish_all(self) -> None:
        # Cover
        cover_msg = WorldCoverMsg()
        cover_msg.polygons = [
            self._cover_to_msg(c) for c in self._world.cover
        ]
        self._pub_cover.publish(cover_msg)

        # Civilians
        civ_msg = CivilianListMsg()
        civ_msg.civilians = [self._civilian_to_msg(c) for c in self._world.civilians]
        self._pub_civilians.publish(civ_msg)

        # IED
        ied_msg = IedZoneListMsg()
        ied_msg.zones = [self._ied_to_msg(z) for z in self._world.ied_zones]
        self._pub_ied.publish(ied_msg)

        # EW emitters
        ew_msg = EwEmitterListMsg()
        ew_msg.emitters = [self._ew_to_msg(e) for e in self._world.ew_emitters]
        self._pub_ew.publish(ew_msg)

        # Jamming
        jam_msg = JammingZoneListMsg()
        jam_msg.zones = [self._jamming_to_msg(z) for z in self._world.jamming_zones]
        self._pub_jamming.publish(jam_msg)

        # Dead zones
        dz_msg = DeadZoneListMsg()
        dz_msg.zones = [self._dead_to_msg(z) for z in self._world.dead_zones]
        self._pub_dead.publish(dz_msg)

        # Weather
        self._pub_weather.publish(self._weather_to_msg(self._world.weather))

        # ROE
        self._pub_roe.publish(self._roe_to_msg(self._world.roe))

        # Weapons
        wcat = WeaponCatalogMsg()
        wcat.weapons = [self._weapon_to_msg(w) for w in self._world.weapons.values()]
        self._pub_weapons.publish(wcat)

    # ---- conversion helpers ----------------------------------------------

    @staticmethod
    def _cover_to_msg(cover) -> CoverPolygonMsg:
        msg = CoverPolygonMsg()
        msg.vertices_x = [float(v[0]) for v in cover.vertices]
        msg.vertices_y = [float(v[1]) for v in cover.vertices]
        msg.cover_value = float(cover.cover_value)
        msg.label = cover.label
        return msg

    @staticmethod
    def _civilian_to_msg(c) -> CivilianMsg:
        msg = CivilianMsg()
        msg.x = float(c["position"][0])
        msg.y = float(c["position"][1])
        msg.movement = c.get("movement", "stationary")
        msg.count = int(c.get("count", 1))
        msg.radius_m = float(c.get("radius_m", 0.0))
        msg.label = c.get("label", "civilian")
        return msg

    @staticmethod
    def _ied_to_msg(z) -> IedZoneMsg:
        msg = IedZoneMsg()
        msg.x = float(z["position"][0])
        msg.y = float(z["position"][1])
        msg.detectability_radius_m = float(z.get("detectability_radius_m", 5.0))
        msg.label = z.get("label", "ied")
        return msg

    @staticmethod
    def _ew_to_msg(e) -> EwEmitterMsg:
        msg = EwEmitterMsg()
        msg.x = float(e["position"][0])
        msg.y = float(e["position"][1])
        msg.frequency_mhz = float(e.get("frequency_mhz", 0.0))
        msg.signal_strength = float(e.get("signal_strength", 1.0))
        msg.label = e.get("label", "emitter")
        return msg

    @staticmethod
    def _jamming_to_msg(z) -> JammingZoneMsg:
        msg = JammingZoneMsg()
        msg.x = float(z.position[0])
        msg.y = float(z.position[1])
        msg.radius_m = float(z.radius_m)
        msg.strength = float(z.strength)
        msg.label = z.label
        return msg

    @staticmethod
    def _dead_to_msg(z) -> DeadZoneMsg:
        msg = DeadZoneMsg()
        msg.vertices_x = [float(v[0]) for v in z.vertices]
        msg.vertices_y = [float(v[1]) for v in z.vertices]
        msg.label = z.label
        return msg

    @staticmethod
    def _weather_to_msg(w) -> WeatherStateMsg:
        msg = WeatherStateMsg()
        msg.visibility_m = float(w.visibility_m)
        msg.wind_speed = float(w.wind_speed)
        msg.wind_direction = float(w.wind_direction)
        msg.fog_density = float(w.fog_density)
        msg.time_of_day = float(w.time_of_day)
        return msg

    @staticmethod
    def _roe_to_msg(r) -> RoeRulesMsg:
        msg = RoeRulesMsg()
        msg.fire_permission = r.fire_permission
        msg.civilian_proximity_m = float(r.civilian_proximity_m)
        msg.require_target_id = bool(r.require_target_id)
        msg.friendly_fire_severity = r.friendly_fire_severity
        return msg

    @staticmethod
    def _weapon_to_msg(w) -> WeaponSpecMsg:
        msg = WeaponSpecMsg()
        msg.name = w.name
        msg.max_range_m = float(w.max_range_m)
        msg.base_pok_at_100m = float(w.base_pok_at_100m)
        msg.falloff_per_100m = float(w.falloff_per_100m)
        msg.damage = float(w.damage)
        msg.ammo_capacity = int(w.ammo_capacity)
        msg.suppress_radius_m = float(w.suppress_radius_m)
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldStateNode()
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
