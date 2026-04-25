"""Robot Bridge Node — UDP communication with physical ESP32 robots.

Handles UDP registration (REG), translates ROS2 cmd_vel to UDP motor
commands, and monitors robot connectivity via heartbeat pings.
"""

from __future__ import annotations

import logging
import queue
import socket
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from plato_pod_msgs.msg import TagDetections
from plato_pod_msgs.srv import RegisterPhysical

from plato_pod.pose import PoseSource
from plato_pod.udp_protocol import (
    clamp_velocity,
    format_heartbeat,
    format_move,
    format_reg_response_deferred,
    format_reg_response_err_duplicate,
    format_reg_response_ok,
    parse_incoming,
)

logger = logging.getLogger(__name__)


@dataclass
class RegistrationRequest:
    """Pending registration from the UDP thread."""
    tag_id: int
    radius_mm: int
    x: float
    y: float
    theta: float
    udp_addr: tuple[str, int]
    response_event: threading.Event
    result: tuple[bool, int, str] | None = None  # (success, robot_id, message)


class RobotBridgeNode(Node):
    """ROS2 node bridging UDP robots to the ROS2 graph."""

    def __init__(self) -> None:
        super().__init__("robot_bridge_node")

        # Parameters
        self.declare_parameter("udp_port", 9999)
        self.declare_parameter("heartbeat_interval_sec", 2.0)
        self.declare_parameter("heartbeat_timeout_sec", 6.0)
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("max_angular_speed", 2.0)

        self._udp_port = self.get_parameter("udp_port").value
        self._heartbeat_interval = self.get_parameter("heartbeat_interval_sec").value
        self._heartbeat_timeout = self.get_parameter("heartbeat_timeout_sec").value
        self._max_linear = self.get_parameter("max_linear_speed").value
        self._max_angular = self.get_parameter("max_angular_speed").value

        # Robot tracking
        self._robot_addresses: dict[int, tuple[str, int]] = {}  # robot_id -> (ip, port)
        self._tag_to_robot: dict[int, int] = {}  # tag_id -> robot_id
        self._last_heartbeat_response: dict[int, float] = {}  # robot_id -> timestamp
        self._cmd_vel_subs: dict[int, object] = {}  # robot_id -> subscription
        self._addr_lock = threading.Lock()

        # Visible robot tags cache (from /tags/detections)
        self._visible_tags: dict[int, tuple[float, float, float]] = {}
        # tag_id -> (x, y, theta)
        self._visible_lock = threading.Lock()

        # Registration queue (UDP thread → ROS2 timer)
        self._reg_queue: queue.Queue[RegistrationRequest] = queue.Queue()

        # Registry service client
        self._registry_client = self.create_client(
            RegisterPhysical, "/registry_node/register_physical"
        )

        # Subscribe to tag detections for visible tag cache
        self.create_subscription(
            TagDetections, "/tags/detections", self._detections_callback, 10
        )

        # UDP socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", self._udp_port))
        self._sock.settimeout(0.1)  # 100ms timeout for clean shutdown
        self._running = True

        # UDP receive daemon thread
        self._udp_thread = threading.Thread(
            target=self._udp_receive_loop, daemon=True
        )
        self._udp_thread.start()

        # Registration processing timer (50 Hz — drains the queue)
        self._reg_timer = self.create_timer(0.02, self._process_registration_queue)

        # Heartbeat timer
        self._hb_timer = self.create_timer(
            self._heartbeat_interval, self._heartbeat_tick
        )

        self.get_logger().info(
            f"Robot bridge ready on UDP port {self._udp_port}"
        )

    def _detections_callback(self, msg: TagDetections) -> None:
        """Cache visible robot tags for registration matching."""
        with self._visible_lock:
            self._visible_tags.clear()
            for tag_pose in msg.robot_tags:
                self._visible_tags[tag_pose.tag_id] = (
                    tag_pose.x, tag_pose.y, tag_pose.theta,
                )

    def _udp_receive_loop(self) -> None:
        """Background thread: receive and handle UDP messages from robots."""
        while self._running:
            try:
                data, addr = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            parsed = parse_incoming(data)
            if parsed is None:
                continue

            cmd, params = parsed

            if cmd == "REG":
                self._handle_reg(params["tag_id"], params["radius_mm"], addr)
            elif cmd == "OK":
                # Heartbeat response from a robot
                self._handle_heartbeat_response(addr)
            elif cmd == "PONG":
                self._handle_heartbeat_response(addr)

    def _handle_reg(
        self, tag_id: int, radius_mm: int, addr: tuple[str, int]
    ) -> None:
        """Handle a REG message from the UDP thread."""
        # Check if tag is visible
        with self._visible_lock:
            tag_data = self._visible_tags.get(tag_id)

        if tag_data is None:
            self._sock.sendto(format_reg_response_deferred(), addr)
            return

        # Check if already registered with this tag_id
        with self._addr_lock:
            if tag_id in self._tag_to_robot:
                robot_id = self._tag_to_robot[tag_id]
                # Check if it's inactive (reconnection)
                # We'll let the registry service handle this
                pass

        x, y, theta = tag_data

        # Enqueue for the ROS2 timer to process
        req = RegistrationRequest(
            tag_id=tag_id,
            radius_mm=radius_mm,
            x=x, y=y, theta=theta,
            udp_addr=addr,
            response_event=threading.Event(),
        )
        self._reg_queue.put(req)

        # Wait for response (with timeout)
        if req.response_event.wait(timeout=3.0):
            if req.result is not None:
                success, robot_id, message = req.result
                if success:
                    self._sock.sendto(format_reg_response_ok(robot_id), addr)
                    with self._addr_lock:
                        self._robot_addresses[robot_id] = addr
                        self._tag_to_robot[tag_id] = robot_id
                        self._last_heartbeat_response[robot_id] = time.time()
                    # Create cmd_vel subscription on the main thread
                    # (will be picked up by the registration timer)
                elif "DUPLICATE" in message:
                    self._sock.sendto(format_reg_response_err_duplicate(), addr)
                else:
                    self._sock.sendto(format_reg_response_deferred(), addr)
            else:
                self._sock.sendto(format_reg_response_deferred(), addr)
        else:
            # Timeout — registry service not responding
            self._sock.sendto(format_reg_response_deferred(), addr)

    def _process_registration_queue(self) -> None:
        """ROS2 timer callback: drain the registration queue and call the service."""
        while not self._reg_queue.empty():
            try:
                req = self._reg_queue.get_nowait()
            except queue.Empty:
                break

            if not self._registry_client.wait_for_service(timeout_sec=0.1):
                req.result = (False, -1, "DEFERRED")
                req.response_event.set()
                continue

            srv_req = RegisterPhysical.Request()
            srv_req.localization_id = str(req.tag_id)
            srv_req.localization_source = PoseSource.CAMERA_ARTAG.value
            srv_req.radius_mm = req.radius_mm
            srv_req.udp_address = req.udp_addr[0]
            srv_req.udp_port = req.udp_addr[1]
            srv_req.x = req.x
            srv_req.y = req.y
            srv_req.theta = req.theta

            future = self._registry_client.call_async(srv_req)
            future.add_done_callback(
                lambda f, r=req: self._on_registration_response(f, r)
            )

    def _on_registration_response(self, future, req: RegistrationRequest) -> None:
        """Callback when the registry service responds."""
        try:
            result = future.result()
            req.result = (result.success, result.robot_id, result.message)
            if result.success:
                # Create cmd_vel subscription for this robot
                self._create_cmd_vel_sub(result.robot_id)
        except Exception as e:
            self.get_logger().error(f"Registration service call failed: {e}")
            req.result = (False, -1, "DEFERRED")
        finally:
            req.response_event.set()

    def _create_cmd_vel_sub(self, robot_id: int) -> None:
        """Create a cmd_vel subscription for a newly registered robot."""
        if robot_id in self._cmd_vel_subs:
            return

        topic = f"/robot_{robot_id}/cmd_vel"

        def callback(msg: Twist, rid=robot_id) -> None:
            self._handle_cmd_vel(rid, msg)

        sub = self.create_subscription(Twist, topic, callback, 10)
        self._cmd_vel_subs[robot_id] = sub
        self.get_logger().info(f"Subscribed to {topic}")

    def _handle_cmd_vel(self, robot_id: int, msg: Twist) -> None:
        """Translate a cmd_vel message to UDP motor command."""
        with self._addr_lock:
            addr = self._robot_addresses.get(robot_id)

        if addr is None:
            return

        linear = msg.linear.x
        angular = msg.angular.z
        linear, angular = clamp_velocity(
            linear, angular, self._max_linear, self._max_angular
        )

        try:
            self._sock.sendto(format_move(linear, angular), addr)
        except OSError as e:
            self.get_logger().error(f"UDP send to robot {robot_id} failed: {e}")

    def _handle_heartbeat_response(self, addr: tuple[str, int]) -> None:
        """Record a heartbeat response from a robot."""
        with self._addr_lock:
            for robot_id, robot_addr in self._robot_addresses.items():
                if robot_addr == addr:
                    self._last_heartbeat_response[robot_id] = time.time()
                    break

    def _heartbeat_tick(self) -> None:
        """Send heartbeat pings and check for timeouts."""
        now = time.time()

        with self._addr_lock:
            addresses = dict(self._robot_addresses)
            last_responses = dict(self._last_heartbeat_response)

        for robot_id, addr in addresses.items():
            # Send heartbeat
            try:
                self._sock.sendto(format_heartbeat(), addr)
            except OSError:
                pass

            # Check timeout
            last = last_responses.get(robot_id, now)
            if now - last > self._heartbeat_timeout:
                self.get_logger().warning(
                    f"Robot {robot_id} heartbeat timeout — marking inactive"
                )
                # Remove from tracking
                with self._addr_lock:
                    self._robot_addresses.pop(robot_id, None)
                    self._last_heartbeat_response.pop(robot_id, None)
                    # Find and remove tag mapping
                    tag_id = None
                    for tid, rid in self._tag_to_robot.items():
                        if rid == robot_id:
                            tag_id = tid
                            break
                    if tag_id is not None:
                        del self._tag_to_robot[tag_id]

                # Destroy cmd_vel subscription
                sub = self._cmd_vel_subs.pop(robot_id, None)
                if sub is not None:
                    self.destroy_subscription(sub)

                # Notify registry (fire and forget)
                if self._registry_client.wait_for_service(timeout_sec=0.1):
                    from plato_pod_msgs.srv import ResetRobot
                    reset_client = self.create_client(
                        ResetRobot, "/registry_node/reset_robot"
                    )
                    req = ResetRobot.Request()
                    req.robot_id = robot_id
                    reset_client.call_async(req)

    def send_to_robot(self, robot_id: int, data: bytes) -> bool:
        """Send arbitrary UDP data to a registered robot.

        Args:
            robot_id: Target robot ID.
            data: Raw bytes to send.

        Returns:
            True if sent successfully.
        """
        with self._addr_lock:
            addr = self._robot_addresses.get(robot_id)
        if addr is None:
            return False
        try:
            self._sock.sendto(data, addr)
            return True
        except OSError:
            return False

    def destroy_node(self) -> None:
        """Clean shutdown."""
        self._running = False
        self._sock.close()
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod robot_bridge_node."""
    rclpy.init(args=args)
    node = RobotBridgeNode()
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
