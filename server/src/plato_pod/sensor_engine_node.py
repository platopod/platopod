"""Sensor Engine Node — computes sensor data for all robots.

Subscribes to /robots/status and /arena/model, runs configured sensors
per robot, and publishes sensor data as JSON on per-robot topics.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from plato_pod_msgs.msg import (
    ArenaModel as ArenaModelMsg,
    RobotStatusList as RobotStatusListMsg,
    SensorReading as SensorReadingMsg,
)

from plato_pod.robot import Robot
from plato_pod.sensor_engine import SensorEngine
from plato_pod.sensor_plugins.base import ArenaState, EnvironmentContext, SensorConfig
from plato_pod.sensor_plugins.gazebo_bridge import GazeboImuBridge, GazeboLidarBridge
from plato_pod.sensor_plugins.gaden_bridge import GadenGasBridge
from plato_pod.virtual_layer_loader import load_virtual_layers


# Per-sensor schema versions — bump when the payload shape changes
SENSOR_SCHEMA_VERSIONS = {
    "gas": "1.0",
    "gps": "1.0",
    "lidar_2d": "1.0",
    "sonar": "1.0",
    "fof": "1.0",
    "rangefinder": "1.0",
    "thermal": "1.0",
    "ied_detector": "1.0",
    "df_receiver": "1.0",
    "uav_camera": "1.0",
    "imu": "1.0",
}


def _make_sensor_reading_msg(
    *, robot_id: int, sensor_name: str,
    timestamp: float, payload: dict,
) -> SensorReadingMsg:
    """Build a typed SensorReading envelope from a Python sensor reading dict."""
    msg = SensorReadingMsg()
    msg.robot_id = int(robot_id)
    msg.sensor_name = sensor_name
    msg.timestamp = float(timestamp)
    msg.schema_version = SENSOR_SCHEMA_VERSIONS.get(sensor_name, "1.0")
    msg.payload_format = "json"
    msg.payload = json.dumps(payload, default=_json_default)
    msg.typed_published = False
    return msg


def _json_default(obj):
    """Best-effort JSON encoder for tuples and other simple types."""
    if isinstance(obj, tuple):
        return list(obj)
    if hasattr(obj, "__dict__"):
        return obj.__dict__
    return str(obj)


class SensorEngineNode(Node):
    """ROS2 node computing and publishing sensor data."""

    def __init__(self) -> None:
        super().__init__("sensor_engine_node")

        from rcl_interfaces.msg import ParameterDescriptor
        _dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("tick_rate_hz", 10.0, _dyn)
        self.declare_parameter("default_preset", "minimal", _dyn)
        self.declare_parameter("exercise_file", "", _dyn)
        self.declare_parameter("sensor_source", "python", _dyn)

        tick_rate = float(self.get_parameter("tick_rate_hz").value)
        self._default_preset = self.get_parameter("default_preset").value

        # Sensor engine
        self._engine = SensorEngine()

        # Load virtual layers from exercise YAML if provided
        exercise_file = self.get_parameter("exercise_file").value
        if exercise_file:
            try:
                import yaml
                with open(exercise_file) as f:
                    exercise_config = yaml.safe_load(f)
                env = load_virtual_layers(exercise_config)
                if env is not None:
                    self._engine.set_environment(env)
                    self.get_logger().info(
                        f"Loaded {len(env.fields)} virtual layer(s) from {exercise_file}"
                    )
            except Exception as e:
                self.get_logger().warning(f"Failed to load virtual layers: {e}")

        # Gazebo sensor bridge setup
        self._sensor_source = self.get_parameter("sensor_source").value
        self._gazebo_lidar_bridges: dict[int, GazeboLidarBridge] = {}
        self._gazebo_imu_bridges: dict[int, GazeboImuBridge] = {}
        self._gaden_bridge = GadenGasBridge()
        self._gaden_subscribed = False

        if self._sensor_source == "gazebo":
            self.get_logger().info("Sensor source: Gazebo (bridged from gz-sim)")
            self._subscribe_gaden()
        else:
            self.get_logger().info("Sensor source: Python plugins")

        # Cached state
        self._robots: dict[int, dict] = {}  # robot_id -> status dict
        self._arena = ArenaState(boundary=(), obstacles=[])
        self._known_robot_ids: set[int] = set()

        # Per-robot sensor data publishers
        self._sensor_pubs: dict[int, dict[str, object]] = {}
        # robot_id -> {sensor_name -> publisher}

        # Subscribers
        self.create_subscription(
            RobotStatusListMsg, "/robots/status",
            self._robot_status_callback, 10,
        )
        self.create_subscription(
            ArenaModelMsg, "/arena/model",
            self._arena_model_callback, 10,
        )

        # Tick timer
        dt = 1.0 / tick_rate
        self._timer = self.create_timer(dt, self._tick)

        self.get_logger().info(
            f"Sensor engine node ready ({tick_rate} Hz, "
            f"default preset: {self._default_preset})"
        )

    def _robot_status_callback(self, msg: RobotStatusListMsg) -> None:
        """Track robots and auto-configure sensors for new ones."""
        current_ids = set()
        for robot in msg.robots:
            if robot.status == "active":
                current_ids.add(robot.robot_id)
                self._robots[robot.robot_id] = {
                    "robot_id": robot.robot_id,
                    "x": robot.x, "y": robot.y, "theta": robot.theta,
                    "radius": robot.radius, "type": robot.deployment,
                    "team": None,
                }

                # Auto-apply default preset for new robots
                if robot.robot_id not in self._known_robot_ids:
                    self._engine.apply_preset(robot.robot_id, self._default_preset)
                    self.get_logger().info(
                        f"Robot {robot.robot_id}: applied '{self._default_preset}' preset"
                    )
                    # Subscribe to Gazebo sensor topics for this robot
                    if self._sensor_source == "gazebo":
                        self._subscribe_gazebo_sensors(robot.robot_id)
                        self._subscribe_gaden_for_robot(robot.robot_id)

        # Remove departed robots
        for rid in self._known_robot_ids - current_ids:
            self._engine.remove_robot(rid)
            self._robots.pop(rid, None)
            # Clean up publishers
            for pub in self._sensor_pubs.pop(rid, {}).values():
                self.destroy_publisher(pub)

        self._known_robot_ids = current_ids

    def _subscribe_gazebo_sensors(self, robot_id: int) -> None:
        """Subscribe to Gazebo-bridged sensor topics for a robot."""
        # Lidar
        lidar_bridge = GazeboLidarBridge()
        self._gazebo_lidar_bridges[robot_id] = lidar_bridge

        # The ros_gz_bridge publishes lidar to this topic pattern
        lidar_topic = (
            f"/world/arena/model/robot_{robot_id}"
            f"/link/base_link/sensor/lidar/scan"
        )

        try:
            from sensor_msgs.msg import LaserScan
            def lidar_cb(msg: LaserScan, bridge=lidar_bridge) -> None:
                bridge.update_from_scan(
                    ranges=list(msg.ranges),
                    angle_min=msg.angle_min,
                    angle_max=msg.angle_max,
                    angle_increment=msg.angle_increment,
                    range_min=msg.range_min,
                    range_max=msg.range_max,
                )
            self.create_subscription(LaserScan, lidar_topic, lidar_cb, 10)
            self.get_logger().info(f"Subscribed to Gazebo lidar: {lidar_topic}")
        except ImportError:
            self.get_logger().warning("sensor_msgs not available for lidar bridge")

        # IMU
        imu_bridge = GazeboImuBridge()
        self._gazebo_imu_bridges[robot_id] = imu_bridge

        imu_topic = (
            f"/world/arena/model/robot_{robot_id}"
            f"/link/base_link/sensor/imu/imu"
        )

        try:
            from sensor_msgs.msg import Imu
            def imu_cb(msg: Imu, bridge=imu_bridge) -> None:
                bridge.update_from_imu(
                    linear_acceleration=(
                        msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z,
                    ),
                    angular_velocity=(
                        msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z,
                    ),
                    orientation=(
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w,
                    ),
                )
            self.create_subscription(Imu, imu_topic, imu_cb, 10)
            self.get_logger().info(f"Subscribed to Gazebo IMU: {imu_topic}")
        except ImportError:
            self.get_logger().warning("sensor_msgs not available for IMU bridge")

    def _subscribe_gaden(self) -> None:
        """Subscribe to GADEN gas concentration topics if available.

        GADEN's simulated_sensor publishes Float32 on per-robot topics.
        We subscribe to a pattern topic and extract robot_id from it.
        """
        if self._gaden_subscribed:
            return
        try:
            from std_msgs.msg import Float32

            # GADEN publishes to /Sensor_{id}/gas_concentration
            # We create subscriptions dynamically when robots appear
            self._gaden_subscribed = True
            self.get_logger().info("GADEN gas bridge enabled")
        except ImportError:
            self.get_logger().warning("std_msgs not available for GADEN bridge")

    def _subscribe_gaden_for_robot(self, robot_id: int) -> None:
        """Subscribe to GADEN concentration topic for one robot."""
        if not self._gaden_subscribed:
            return
        try:
            from std_msgs.msg import Float32

            topic = f"/Sensor_{robot_id}/gas_concentration"

            def gaden_cb(msg: Float32, rid=robot_id) -> None:
                self._gaden_bridge.update_concentration(rid, msg.data)

            self.create_subscription(Float32, topic, gaden_cb, 10)
            self.get_logger().info(f"Subscribed to GADEN: {topic}")
        except ImportError:
            pass

    def _arena_model_callback(self, msg: ArenaModelMsg) -> None:
        """Cache arena state for sensor computation."""
        if msg.boundary_valid:
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
            self._arena = ArenaState(boundary=boundary, obstacles=obstacles)

    def _tick(self) -> None:
        """Compute and publish sensor data for all robots."""
        all_robot_states = []
        for rd in self._robots.values():
            all_robot_states.append(Robot(
                robot_id=rd["robot_id"],
                deployment=rd.get("deployment", rd.get("type", "virtual")),
                x=rd["x"], y=rd["y"], theta=rd["theta"],
                radius=rd["radius"], team=rd.get("team"),
            ))

        for robot_state in all_robot_states:
            other_robots = [
                r for r in all_robot_states
                if r.robot_id != robot_state.robot_id
            ]

            readings = self._engine.compute_sensors(
                robot_state, self._arena, other_robots
            )

            for reading in readings:
                sensor_data = reading.data

                # If Gazebo bridge has data for this sensor, use it instead
                if self._sensor_source == "gazebo":
                    if reading.sensor_name == "lidar_2d":
                        bridge = self._gazebo_lidar_bridges.get(reading.robot_id)
                        if bridge and bridge.has_data():
                            sensor_data = bridge.compute(
                                robot_state, self._arena, other_robots, SensorConfig(),
                            )
                    elif reading.sensor_name == "gas":
                        if self._gaden_bridge.has_data(reading.robot_id):
                            sensor_data = self._gaden_bridge.compute(
                                robot_state, self._arena, other_robots,
                                SensorConfig(params=reading.data if isinstance(reading.data, dict) else {}),
                            )

                pub = self._get_or_create_pub(
                    reading.robot_id, reading.sensor_name
                )
                pub.publish(_make_sensor_reading_msg(
                    robot_id=reading.robot_id,
                    sensor_name=reading.sensor_name,
                    timestamp=reading.timestamp,
                    payload=sensor_data,
                ))

            # Publish Gazebo-only sensors (IMU) that have no Python equivalent
            if self._sensor_source == "gazebo":
                rid = robot_state.robot_id
                imu_bridge = self._gazebo_imu_bridges.get(rid)
                if imu_bridge and imu_bridge.has_data():
                    import time as _time
                    pub = self._get_or_create_pub(rid, "imu")
                    pub.publish(_make_sensor_reading_msg(
                        robot_id=rid,
                        sensor_name="imu",
                        timestamp=_time.time(),
                        payload=imu_bridge.compute(
                            robot_state, self._arena, [], SensorConfig(),
                        ),
                    ))

    def _get_or_create_pub(self, robot_id: int, sensor_name: str):
        """Get or create a typed SensorReading publisher for a robot's sensor."""
        if robot_id not in self._sensor_pubs:
            self._sensor_pubs[robot_id] = {}
        if sensor_name not in self._sensor_pubs[robot_id]:
            topic = f"/robot_{robot_id}/sensors/{sensor_name}"
            self._sensor_pubs[robot_id][sensor_name] = self.create_publisher(
                SensorReadingMsg, topic, 10
            )
        return self._sensor_pubs[robot_id][sensor_name]

    # Public methods for the gateway to call
    def configure_sensor(
        self, robot_id: int, sensor_name: str, config_dict: dict
    ) -> tuple[bool, str]:
        """Configure a sensor (called by gateway)."""
        from plato_pod.sensor_plugins.base import SensorConfig
        config = SensorConfig(
            noise_stddev=config_dict.get("noise_stddev", 0.0),
            rate_hz=config_dict.get("rate_hz", 10.0),
            dropout_rate=config_dict.get("dropout_rate", 0.0),
            params={k: v for k, v in config_dict.items()
                    if k not in ("noise_stddev", "rate_hz", "dropout_rate")},
        )
        return self._engine.configure_sensor(robot_id, sensor_name, config)

    def apply_preset(self, robot_id: int, preset_name: str) -> tuple[bool, str]:
        """Apply a sensor preset (called by gateway)."""
        return self._engine.apply_preset(robot_id, preset_name)

    def get_robot_sensors(self, robot_id: int) -> list[str]:
        """Get configured sensors for a robot."""
        return self._engine.get_robot_sensors(robot_id)

    @property
    def engine(self) -> SensorEngine:
        """Direct access to the sensor engine for environment updates."""
        return self._engine


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod sensor_engine_node."""
    rclpy.init(args=args)
    node = SensorEngineNode()
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
