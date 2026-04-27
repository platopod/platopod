"""Registry Node — manages the robot registry via ROS2 services.

Subscribes to localization provider detections for physical robot pose
updates and /arena/model for spawn validation boundary. Exposes ROS2
services for registration, spawning, removal, reset, and listing.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node

from plato_pod_msgs.msg import (
    ArenaModel as ArenaModelMsg,
    RobotStatus as RobotStatusMsg,
    RobotStatusList as RobotStatusListMsg,
    TagDetections,
)
from plato_pod_msgs.srv import (
    ApplyDamage,
    ListRobots,
    RegisterPhysical,
    RemoveRobot,
    ResetRobot,
    SpawnVirtual,
)

from plato_pod.pose import PoseSource
from plato_pod.robot import DEFAULT_RADIUS, Robot
from plato_pod.robot_registry import Registry


class RegistryNode(Node):
    """ROS2 node managing the robot registry."""

    def __init__(self) -> None:
        super().__init__("registry_node")

        # Parameters
        self.declare_parameter("default_radius", DEFAULT_RADIUS)
        self.declare_parameter("publish_rate_hz", 10.0)

        self._default_radius = self.get_parameter("default_radius").value
        publish_rate = self.get_parameter("publish_rate_hz").value

        # Registry
        self._registry = Registry()

        # Cached state
        self._arena_boundary: tuple[tuple[float, float], ...] = ()

        # Virtual robot pose subscriptions (dynamic, created on spawn)
        self._virtual_pose_subs: dict[int, object] = {}  # robot_id -> subscription

        # Services
        self.create_service(
            RegisterPhysical, "~/register_physical", self._handle_register_physical
        )
        self.create_service(
            SpawnVirtual, "~/spawn_virtual", self._handle_spawn_virtual
        )
        self.create_service(
            RemoveRobot, "~/remove_robot", self._handle_remove_robot
        )
        self.create_service(
            ResetRobot, "~/reset_robot", self._handle_reset_robot
        )
        self.create_service(
            ListRobots, "~/list_robots", self._handle_list_robots
        )
        self.create_service(
            ApplyDamage, "~/apply_damage", self._handle_apply_damage
        )

        # Subscribers
        self.create_subscription(
            TagDetections, "/tags/detections", self._detections_callback, 10
        )
        self.create_subscription(
            ArenaModelMsg, "/arena/model", self._arena_model_callback, 10
        )

        # Publisher
        self._pub_status = self.create_publisher(
            RobotStatusListMsg, "/robots/status", 10
        )

        # Status publish timer
        timer_period = 1.0 / publish_rate
        self._timer = self.create_timer(timer_period, self._publish_status)

        self.get_logger().info("Registry node ready")

    def _detections_callback(self, msg: TagDetections) -> None:
        """Update physical robot poses from localization provider detections."""
        # Update poses of registered physical robots
        for tag_pose in msg.robot_tags:
            entry = self._registry.find_by_localization_id(str(tag_pose.tag_id))
            if entry is not None and entry.status == "active":
                self._registry.update_pose(
                    entry.robot_id, tag_pose.x, tag_pose.y, tag_pose.theta
                )

    def _arena_model_callback(self, msg: ArenaModelMsg) -> None:
        """Cache the arena boundary for spawn validation."""
        if msg.boundary_valid:
            self._arena_boundary = tuple(
                (float(x), float(y))
                for x, y in zip(msg.boundary_x, msg.boundary_y)
            )

    def _handle_register_physical(
        self, request: RegisterPhysical.Request, response: RegisterPhysical.Response
    ) -> RegisterPhysical.Response:
        """Handle physical robot registration from any localization backend."""
        # Map localization_source string to PoseSource enum
        try:
            loc_source = PoseSource(request.localization_source)
        except ValueError:
            loc_source = PoseSource.UNKNOWN

        ok, robot_id, message = self._registry.register_physical(
            localization_id=request.localization_id,
            localization_source=loc_source,
            radius_m=request.radius_mm / 1000.0,
            x=request.x,
            y=request.y,
            theta=request.theta,
        )
        response.success = ok
        response.robot_id = robot_id
        response.message = message
        if ok:
            self.get_logger().info(
                f"Physical robot registered: id={robot_id}, "
                f"loc={request.localization_id} ({request.localization_source})"
            )
        return response

    def _handle_spawn_virtual(
        self, request: SpawnVirtual.Request, response: SpawnVirtual.Response
    ) -> SpawnVirtual.Response:
        """Handle virtual robot spawn request."""
        radius = request.radius if request.radius > 0 else self._default_radius
        # Optional tactical fields with safe defaults
        team = request.team if request.team else None
        vehicle_role = request.vehicle_role if request.vehicle_role else "default"
        health = request.health if request.health > 0.0 else 1.0
        weapons = list(request.weapons) if request.weapons else []
        ok, robot_id, message = self._registry.spawn_virtual(
            x=request.x,
            y=request.y,
            theta=request.theta,
            radius=radius,
            boundary=self._arena_boundary,
            team=team,
            vehicle_role=vehicle_role,
            health=health,
            weapons=weapons,
        )
        response.success = ok
        response.robot_id = robot_id
        response.message = message
        if ok:
            self.get_logger().info(
                f"Virtual robot spawned: id={robot_id} at ({request.x:.3f}, {request.y:.3f})"
            )
            self._subscribe_virtual_pose(robot_id)
        else:
            self.get_logger().warning(f"Spawn rejected: {message}")
        return response

    def _handle_remove_robot(
        self, request: RemoveRobot.Request, response: RemoveRobot.Response
    ) -> RemoveRobot.Response:
        """Handle robot removal request."""
        ok, message = self._registry.remove_robot(request.robot_id)
        response.success = ok
        response.message = message
        if ok:
            self._unsubscribe_virtual_pose(request.robot_id)
        return response

    def _handle_reset_robot(
        self, request: ResetRobot.Request, response: ResetRobot.Response
    ) -> ResetRobot.Response:
        """Handle robot reset request."""
        ok, message = self._registry.reset_robot(request.robot_id)
        response.success = ok
        response.message = message
        return response

    def _handle_list_robots(
        self, request: ListRobots.Request, response: ListRobots.Response
    ) -> ListRobots.Response:
        """Handle robot list query."""
        robots = self._registry.list_robots()
        response.robots = [self._entry_to_msg(r) for r in robots]
        return response

    def _handle_apply_damage(
        self, request: ApplyDamage.Request, response: ApplyDamage.Response
    ) -> ApplyDamage.Response:
        """Subtract damage from a robot's health and update its status."""
        ok, new_health, new_status, message = self._registry.apply_damage(
            request.robot_id, float(request.damage),
        )
        response.success = ok
        response.new_health = float(new_health)
        response.new_status = new_status
        response.message = message
        if ok:
            self.get_logger().info(
                f"Damage applied: id={request.robot_id} "
                f"reason={request.reason} health→{new_health:.2f} "
                f"status={new_status}"
            )
        return response

    def _subscribe_virtual_pose(self, robot_id: int) -> None:
        """Create a pose subscription for a virtual robot."""
        topic = f"/robot_{robot_id}/pose"

        def callback(msg: Pose2D, rid=robot_id) -> None:
            self._registry.update_pose(rid, msg.x, msg.y, msg.theta)

        self._virtual_pose_subs[robot_id] = self.create_subscription(
            Pose2D, topic, callback, 10,
        )

    def _unsubscribe_virtual_pose(self, robot_id: int) -> None:
        """Remove a virtual robot's pose subscription."""
        sub = self._virtual_pose_subs.pop(robot_id, None)
        if sub is not None:
            self.destroy_subscription(sub)

    def _publish_status(self) -> None:
        """Publish the current robot status list."""
        robots = self._registry.list_robots()
        msg = RobotStatusListMsg()
        msg.robots = [self._entry_to_msg(r) for r in robots]
        msg.timestamp = float(time.time())
        self._pub_status.publish(msg)

    @staticmethod
    def _entry_to_msg(entry: Robot) -> RobotStatusMsg:
        """Convert a Robot to a RobotStatus ROS2 message."""
        msg = RobotStatusMsg()
        msg.robot_id = entry.robot_id
        msg.deployment = entry.deployment
        msg.x = float(entry.x)
        msg.y = float(entry.y)
        msg.theta = float(entry.theta)
        msg.radius = float(entry.radius)
        msg.status = entry.status
        msg.localization_id = entry.localization_id
        msg.localization_source = entry.localization_source.value
        msg.team = entry.team or ""
        msg.vehicle_role = entry.vehicle_role
        msg.health = float(entry.health)
        msg.weapons = list(entry.weapons)
        msg.thermal_signature = float(entry.thermal_signature)
        return msg


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod registry_node."""
    rclpy.init(args=args)
    node = RegistryNode()
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
