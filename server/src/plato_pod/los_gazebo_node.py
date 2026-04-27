"""LoS Gazebo Node — serves /evaluate_los via a Gazebo lidar query.

Issues a one-shot lidar ray against the live Gazebo world for each request,
then compares the returned hit distance to the requested ray length.

Implements the same `EvaluateLos.srv` contract as `los_python_node`, so
consumers (engagement_node, sensor plugins, behavior trees) are agnostic
to the backend.

This node requires Gazebo + ros_gz_bridge. When the Gazebo lidar topic is
not available, the service responds with `visible=true, rationale="gazebo_unavailable"`
so callers can fall back to the Python backend.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node

try:
    from sensor_msgs.msg import LaserScan
    _HAS_LASER_SCAN = True
except ImportError:
    LaserScan = None  # type: ignore[assignment, misc]
    _HAS_LASER_SCAN = False

from plato_pod_msgs.srv import EvaluateLos


class LosGazeboNode(Node):
    """LoS service backed by a Gazebo lidar ray cast.

    Subscribes to a configurable LaserScan topic that is expected to be
    bridged from a `ray` (or `gpu_ray`) sensor in the Gazebo world. On
    each request, the node consults the most recent scan and computes
    visibility from the bearing of the requested ray.

    For the v1 implementation we sample the most recent scan; a v2 could
    spawn a transient sensor entity per query for arbitrary (from, to)
    pairs that don't share an origin with a known sensor.
    """

    def __init__(self) -> None:
        super().__init__("los_gazebo_node")

        self.declare_parameter(
            "laser_scan_topic", "/world/los_probe/scan",
        )
        topic = str(self.get_parameter("laser_scan_topic").value)

        self._latest_scan = None
        self._scan_lock = threading.Lock()
        self._scan_age_limit_s = 1.0

        if _HAS_LASER_SCAN:
            self.create_subscription(
                LaserScan, topic, self._scan_cb, 10,
            )
        else:
            self.get_logger().warning(
                "sensor_msgs.LaserScan unavailable — LoS service will "
                "always return visible=true with rationale=gazebo_unavailable"
            )

        self.create_service(
            EvaluateLos, "~/evaluate_los", self._handle_evaluate,
        )
        self.get_logger().info(
            f"LoS Gazebo node ready (laser scan topic: {topic})"
        )

    def _scan_cb(self, msg) -> None:
        with self._scan_lock:
            self._latest_scan = (time.time(), msg)

    def _handle_evaluate(
        self, request: EvaluateLos.Request, response: EvaluateLos.Response,
    ) -> EvaluateLos.Response:
        # No scan yet: respond optimistically so the caller can fall back.
        with self._scan_lock:
            sample = self._latest_scan

        if sample is None or not _HAS_LASER_SCAN:
            response.visible = True
            response.attenuation = 1.0
            response.rationale = "gazebo_unavailable"
            response.obstruction_distance_m = -1.0
            return response

        scan_time, scan = sample
        if time.time() - scan_time > self._scan_age_limit_s:
            response.visible = True
            response.attenuation = 1.0
            response.rationale = "scan_stale"
            response.obstruction_distance_m = -1.0
            return response

        # Compute bearing of the requested ray relative to scan frame
        # (assumed to share the world frame for v1). The scan represents
        # rays from the sensor origin; we treat the sensor origin as the
        # ray start and ignore any offset between (from_x, from_y) and
        # the actual sensor pose. v2 should spawn a probe at request.from.
        dx = request.to_x - request.from_x
        dy = request.to_y - request.from_y
        ray_length = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)

        # Find the scan beam closest to this bearing
        idx = int(round((bearing - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            response.visible = True
            response.attenuation = 1.0
            response.rationale = "out_of_scan_fov"
            response.obstruction_distance_m = -1.0
            return response

        hit_distance = float(scan.ranges[idx])
        if math.isinf(hit_distance) or hit_distance >= ray_length:
            response.visible = True
            response.attenuation = 1.0
            response.rationale = "clear"
            response.obstruction_distance_m = -1.0
        else:
            response.visible = False
            response.attenuation = 0.0
            response.rationale = "blocked_by_terrain"
            response.obstruction_distance_m = hit_distance

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LosGazeboNode()
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
