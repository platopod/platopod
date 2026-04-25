"""Virtual Simulation Node — kinematics engine for virtual robots.

Subscribes to cmd_vel for each virtual robot, computes differential-drive
motion at 50 Hz, and publishes updated poses. Virtual robots become
indistinguishable from physical robots to all other components.
"""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node

from plato_pod_msgs.msg import (
    ArenaModel as ArenaModelMsg,
    RobotStatusList as RobotStatusListMsg,
)

from plato_pod.kinematics import (
    add_pose_drift,
    add_velocity_noise,
    apply_acceleration_limit,
    clamp_to_boundary,
)
from plato_pod.kinematics_model import DifferentialDrive, KinematicsModel, get_kinematics_model


@dataclass
class VirtualRobotState:
    """Internal state for a single virtual robot."""
    robot_id: int
    x: float
    y: float
    theta: float
    radius: float
    kinematics: KinematicsModel = None  # type: ignore[assignment]
    target_linear: float = 0.0
    target_angular: float = 0.0
    current_linear: float = 0.0
    current_angular: float = 0.0


class VirtualSimNode(Node):
    """ROS2 node simulating virtual robot kinematics."""

    def __init__(self) -> None:
        super().__init__("virtual_sim_node")

        # Parameters
        self.declare_parameter("tick_rate_hz", 50)
        self.declare_parameter("max_linear_speed", 0.2)
        self.declare_parameter("max_angular_speed", 2.0)
        self.declare_parameter("acceleration_limit", 0.0)
        self.declare_parameter("velocity_noise_stddev", 0.0)
        self.declare_parameter("pose_drift_stddev", 0.0)

        self._tick_rate = self.get_parameter("tick_rate_hz").value
        self._max_linear = self.get_parameter("max_linear_speed").value
        self._max_angular = self.get_parameter("max_angular_speed").value
        self._accel_limit = self.get_parameter("acceleration_limit").value
        self._vel_noise = self.get_parameter("velocity_noise_stddev").value
        self._pose_drift = self.get_parameter("pose_drift_stddev").value

        self._dt = 1.0 / self._tick_rate

        # Virtual robot states
        self._robots: dict[int, VirtualRobotState] = {}
        self._cmd_vel_subs: dict[int, object] = {}
        self._pose_pubs: dict[int, object] = {}

        # Tracked virtual robot IDs from registry
        self._known_virtual_ids: set[int] = set()

        # Arena boundary for safety-net clamping
        self._boundary: tuple[tuple[float, float], ...] = ()

        # Subscribe to robot status to discover virtual robots
        self.create_subscription(
            RobotStatusListMsg, "/robots/status",
            self._robot_status_callback, 10,
        )

        # Subscribe to arena model for boundary clamping
        self.create_subscription(
            ArenaModelMsg, "/arena/model",
            self._arena_model_callback, 10,
        )

        # Simulation tick timer
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"Virtual sim node ready ({self._tick_rate} Hz, "
            f"accel={self._accel_limit}, noise={self._vel_noise}, "
            f"drift={self._pose_drift})"
        )

    def _robot_status_callback(self, msg: RobotStatusListMsg) -> None:
        """Track virtual robot lifecycle from the registry."""
        current_virtual_ids = set()

        for robot in msg.robots:
            if robot.deployment == "virtual" and robot.status == "active":
                current_virtual_ids.add(robot.robot_id)

                if robot.robot_id not in self._robots:
                    # New virtual robot — create state and subscriptions
                    self._add_robot(
                        robot.robot_id, robot.x, robot.y, robot.theta, robot.radius
                    )

        # Remove robots that are no longer in the registry
        removed = self._known_virtual_ids - current_virtual_ids
        for robot_id in removed:
            self._remove_robot(robot_id)

        self._known_virtual_ids = current_virtual_ids

    def _arena_model_callback(self, msg: ArenaModelMsg) -> None:
        """Cache arena boundary for safety-net clamping."""
        if msg.boundary_valid:
            self._boundary = tuple(
                (float(x), float(y))
                for x, y in zip(msg.boundary_x, msg.boundary_y)
            )

    def _add_robot(
        self, robot_id: int, x: float, y: float, theta: float, radius: float
    ) -> None:
        """Create state and ROS2 resources for a new virtual robot."""
        try:
            kin = get_kinematics_model("differential_drive")
        except KeyError:
            kin = DifferentialDrive()
        self._robots[robot_id] = VirtualRobotState(
            robot_id=robot_id, x=x, y=y, theta=theta, radius=radius,
            kinematics=kin,
        )

        # Create cmd_vel subscriber
        topic = f"/robot_{robot_id}/cmd_vel"

        def callback(msg: Twist, rid=robot_id) -> None:
            self._cmd_vel_callback(rid, msg)

        self._cmd_vel_subs[robot_id] = self.create_subscription(
            Twist, topic, callback, 10,
        )

        # Create pose publisher
        self._pose_pubs[robot_id] = self.create_publisher(
            Pose2D, f"/robot_{robot_id}/pose", 10,
        )

        self.get_logger().info(
            f"Virtual robot {robot_id} added at ({x:.3f}, {y:.3f}, {theta:.2f})"
        )

    def _remove_robot(self, robot_id: int) -> None:
        """Remove state and ROS2 resources for a deleted virtual robot."""
        self._robots.pop(robot_id, None)

        sub = self._cmd_vel_subs.pop(robot_id, None)
        if sub is not None:
            self.destroy_subscription(sub)

        pub = self._pose_pubs.pop(robot_id, None)
        if pub is not None:
            self.destroy_publisher(pub)

        self.get_logger().info(f"Virtual robot {robot_id} removed")

    def _cmd_vel_callback(self, robot_id: int, msg: Twist) -> None:
        """Update target velocity for a virtual robot."""
        state = self._robots.get(robot_id)
        if state is None:
            return

        lin = max(-self._max_linear, min(self._max_linear, msg.linear.x))
        ang = max(-self._max_angular, min(self._max_angular, msg.angular.z))
        state.target_linear = lin
        state.target_angular = ang

    def _tick(self) -> None:
        """Simulation tick — update all virtual robot poses."""
        for robot_id, state in self._robots.items():
            # Apply acceleration limit
            state.current_linear = apply_acceleration_limit(
                state.current_linear, state.target_linear,
                self._accel_limit, self._dt,
            )
            state.current_angular = apply_acceleration_limit(
                state.current_angular, state.target_angular,
                self._accel_limit, self._dt,
            )

            # Apply velocity noise
            lin, ang = add_velocity_noise(
                state.current_linear, state.current_angular, self._vel_noise,
            )

            # Compute kinematic update using the robot's kinematics model
            new_x, new_y, new_theta = state.kinematics.predict(
                state.x, state.y, state.theta, lin, ang, self._dt,
            )

            # Apply pose drift
            new_x, new_y, new_theta = add_pose_drift(
                new_x, new_y, new_theta, self._pose_drift,
            )

            # Safety-net boundary clamping
            new_x, new_y = clamp_to_boundary(
                new_x, new_y, state.radius, self._boundary,
            )

            state.x = new_x
            state.y = new_y
            state.theta = new_theta

            # Publish pose
            pub = self._pose_pubs.get(robot_id)
            if pub is not None:
                pose_msg = Pose2D()
                pose_msg.x = float(state.x)
                pose_msg.y = float(state.y)
                pose_msg.theta = float(state.theta)
                pub.publish(pose_msg)


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod virtual_sim_node."""
    rclpy.init(args=args)
    node = VirtualSimNode()
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
