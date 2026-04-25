"""Arena Model Node — dynamic boundary, obstacles, and zones.

Loads arena boundary from exercise YAML (virtual-first) or from localization
provider detections. Publishes the authoritative arena model for all other
nodes to consume.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node
from std_msgs.msg import Header

from plato_pod_msgs.msg import (
    ArenaModel as ArenaModelMsg,
    Obstacle as ObstacleMsg,
    TagDetections,
    Zone as ZoneMsg,
)

from plato_pod.arena_model import (
    ArenaModel,
    Obstacle,
    Zone,
    build_arena_model,
    load_exercise_arena,
)



# Boundary position change threshold (metres) — avoid unnecessary recomputation
_POSITION_EPSILON = 0.001


class ArenaModelNode(Node):
    """ROS2 node managing the arena model."""

    def __init__(self) -> None:
        super().__init__("arena_model_node")

        # Declare parameters
        self.declare_parameter("exercise_file", "")
        self.declare_parameter("boundary_staleness_timeout", 10.0)
        self.declare_parameter("circle_vertices", 16)
        self.declare_parameter("origin_tag", 101)

        # Read parameters
        exercise_file = self.get_parameter("exercise_file").value
        self._staleness_timeout = self.get_parameter("boundary_staleness_timeout").value
        self._circle_vertices = self.get_parameter("circle_vertices").value
        self._origin_tag = self.get_parameter("origin_tag").value

        # Load boundary, obstacles, and zones from exercise config
        self._exercise_boundary: list[tuple[float, float]] | None = None
        self._obstacles: list[Obstacle] = []
        self._zones: list[Zone] = []
        if exercise_file:
            self._load_exercise(exercise_file)

        # Boundary tag cache: tag_id → (x, y, last_seen_time)
        self._boundary_cache: dict[int, tuple[float, float, float]] = {}
        self._last_boundary_positions: dict[int, tuple[float, float]] = {}
        self._current_model: ArenaModel | None = None

        # Publishers
        self._pub_model = self.create_publisher(ArenaModelMsg, "/arena/model", 10)
        self._pub_boundary = self.create_publisher(
            PolygonStamped, "/arena/boundary", 10
        )

        # Subscriber
        self._sub_detections = self.create_subscription(
            TagDetections, "/tags/detections", self._detections_callback, 10
        )

        # If exercise defines a boundary, build the model immediately
        if self._exercise_boundary is not None:
            self._current_model = build_arena_model(
                self._exercise_boundary,
                obstacles=self._obstacles,
                zones=self._zones,
                origin_tag=self._origin_tag,
                timestamp=0.0,
            )
            self.get_logger().info(
                f"Arena boundary from exercise config: "
                f"{len(self._exercise_boundary)} vertices"
            )

        # Periodic republish timer (1 Hz) — ensures late subscribers get the model
        self._republish_timer = self.create_timer(1.0, self._republish)

        self.get_logger().info("Arena model node ready")

    def _republish(self) -> None:
        """Periodically republish the current model for late subscribers."""
        if self._current_model is not None:
            self._publish_model(self._current_model)
            self._publish_boundary(self._current_model)

    def _load_exercise(self, path: str) -> None:
        """Load boundary, obstacles, and zones from an exercise YAML file."""
        try:
            boundary, self._obstacles, self._zones = load_exercise_arena(
                path, circle_vertices=self._circle_vertices
            )
            if boundary is not None:
                self._exercise_boundary = boundary
            self.get_logger().info(
                f"Exercise loaded: boundary={'yes' if boundary else 'no'}, "
                f"{len(self._obstacles)} obstacles, "
                f"{len(self._zones)} zones from {path}"
            )
        except (FileNotFoundError, KeyError) as e:
            self.get_logger().warning(f"Could not load exercise '{path}': {e}")

    def _detections_callback(self, msg: TagDetections) -> None:
        """Process tag detections and update the arena model.

        If the exercise defines an explicit boundary, tag detections do NOT
        redefine the boundary — the YAML boundary is authoritative. Tags are
        still tracked for physical robot pose estimation and camera calibration.
        """
        # If exercise defines the boundary, no need to recompute from tags
        if self._exercise_boundary is not None:
            return

        now = time.time()

        # Update boundary tag cache from the detection message
        for tag_pose in msg.boundary_tags:
            self._boundary_cache[tag_pose.tag_id] = (
                tag_pose.x, tag_pose.y, now,
            )

        # Check for stale tags
        for tag_id, (x, y, last_seen) in self._boundary_cache.items():
            age = now - last_seen
            if age > self._staleness_timeout:
                self.get_logger().warning(
                    f"Boundary tag {tag_id} stale ({age:.1f}s > "
                    f"{self._staleness_timeout}s) — using cached position"
                )

        # Check if boundary has changed (position moved > epsilon)
        boundary_changed = False
        current_positions = {
            tag_id: (x, y) for tag_id, (x, y, _) in self._boundary_cache.items()
        }

        if current_positions != self._last_boundary_positions:
            for tag_id, (x, y) in current_positions.items():
                prev = self._last_boundary_positions.get(tag_id)
                if prev is None:
                    boundary_changed = True
                    break
                dx = x - prev[0]
                dy = y - prev[1]
                if (dx * dx + dy * dy) > _POSITION_EPSILON * _POSITION_EPSILON:
                    boundary_changed = True
                    break

            if not boundary_changed and len(current_positions) != len(
                self._last_boundary_positions
            ):
                boundary_changed = True

        if not boundary_changed and self._current_model is not None:
            return

        # Recompute the arena model from tags — sort by tag ID
        self._last_boundary_positions = dict(current_positions)
        sorted_tags = sorted(current_positions.items())
        boundary_points = [pos for _, pos in sorted_tags]

        timestamp = msg.boundary_tags[0].timestamp if msg.boundary_tags else now

        model = build_arena_model(
            boundary_points,
            obstacles=self._obstacles,
            zones=self._zones,
            origin_tag=self._origin_tag,
            timestamp=timestamp,
        )
        self._current_model = model

        # Publish
        self._publish_model(model)
        self._publish_boundary(model)

    def _publish_model(self, model: ArenaModel) -> None:
        """Publish the full ArenaModel message."""
        msg = ArenaModelMsg()
        msg.boundary_x = [float(p[0]) for p in model.boundary]
        msg.boundary_y = [float(p[1]) for p in model.boundary]
        msg.origin_tag = model.origin_tag
        msg.timestamp = float(model.timestamp)
        msg.boundary_valid = model.boundary_valid

        msg.obstacles = []
        for obs in model.obstacles:
            o = ObstacleMsg()
            o.label = obs.label
            o.vertices_x = [float(v[0]) for v in obs.vertices]
            o.vertices_y = [float(v[1]) for v in obs.vertices]
            o.original_type = obs.original_type
            msg.obstacles.append(o)

        msg.zones = []
        for zone in model.zones:
            z = ZoneMsg()
            z.name = zone.name
            z.team = zone.team or ""
            z.vertices_x = [float(v[0]) for v in zone.vertices]
            z.vertices_y = [float(v[1]) for v in zone.vertices]
            z.hold_time_seconds = float(zone.hold_time_seconds)
            z.original_type = zone.original_type
            msg.zones.append(z)

        self._pub_model.publish(msg)

    def _publish_boundary(self, model: ArenaModel) -> None:
        """Publish the boundary as geometry_msgs/PolygonStamped."""
        msg = PolygonStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "arena"

        for x, y in model.boundary:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            msg.polygon.points.append(pt)

        self._pub_boundary.publish(msg)

    def destroy_node(self) -> None:
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod arena_model_node."""
    rclpy.init(args=args)
    node = ArenaModelNode()
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
