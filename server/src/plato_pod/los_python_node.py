"""LoS Python Node — serves /evaluate_los via the 2D-plus-height-sample model.

Subscribes to `/world/cover` (latched, from world_state_node) for cover
polygons. Uses `plato_pod.line_of_sight.has_line_of_sight` for the math.

This is the lightweight backend — fine for desk arenas without Gazebo.
For terrain-aware LoS, run `los_gazebo_node` instead; same service contract,
consumers are agnostic.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from plato_pod_msgs.msg import WorldCover as WorldCoverMsg
from plato_pod_msgs.srv import EvaluateLos

from plato_pod.line_of_sight import CoverPolygon, has_line_of_sight

_LATCHED_QOS = QoSProfile(
    depth=1, history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class LosPythonNode(Node):
    """LoS service backed by the Python 2D ray model."""

    def __init__(self) -> None:
        super().__init__("los_python_node")

        self._cover: list[CoverPolygon] = []

        self.create_subscription(
            WorldCoverMsg, "/world/cover",
            self._cover_cb, _LATCHED_QOS,
        )
        self.create_service(
            EvaluateLos, "~/evaluate_los", self._handle_evaluate,
        )

        self.get_logger().info(
            "LoS Python node ready (backend: 2D + height samples)"
        )

    def _cover_cb(self, msg: WorldCoverMsg) -> None:
        self._cover = [
            CoverPolygon(
                vertices=list(zip(p.vertices_x, p.vertices_y)),
                cover_value=float(p.cover_value),
                label=p.label,
            )
            for p in msg.polygons
        ]
        self.get_logger().info(
            f"Cover updated: {len(self._cover)} polygons"
        )

    def _handle_evaluate(
        self, request: EvaluateLos.Request, response: EvaluateLos.Response,
    ) -> EvaluateLos.Response:
        result = has_line_of_sight(
            (float(request.from_x), float(request.from_y), float(request.from_z)),
            (float(request.to_x), float(request.to_y), float(request.to_z)),
            cover_polygons=self._cover,
        )
        response.visible = bool(result.visible)
        response.attenuation = float(result.attenuation)
        response.rationale = result.rationale
        response.obstruction_distance_m = -1.0
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LosPythonNode()
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
