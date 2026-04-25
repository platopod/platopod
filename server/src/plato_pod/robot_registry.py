"""Robot registry — manages physical and virtual robots.

Thread-safe registry with spawn validation, registration, and pose updates.
All logic is pure (no ROS2 dependency). The registry_node.py wraps this
in a ROS2 node.
"""

from __future__ import annotations

import logging
import math
import threading
from dataclasses import replace

from plato_pod.geometry import circle_to_polygon, polygon_contains_polygon
from plato_pod.pose import PoseSource
from plato_pod.robot import DEFAULT_RADIUS, Robot

logger = logging.getLogger(__name__)


def validate_spawn_position(
    x: float,
    y: float,
    radius: float,
    boundary: tuple[tuple[float, float], ...],
    existing_robots: list[Robot],
) -> tuple[bool, str]:
    """Validate a spawn position against boundary and existing robots.

    Args:
        x: Spawn X position.
        y: Spawn Y position.
        radius: Robot radius in metres.
        boundary: Arena boundary polygon vertices.
        existing_robots: Currently registered active robots.

    Returns:
        Tuple of (valid, error_message). error_message is empty if valid.
    """
    if len(boundary) < 3:
        return False, "out_of_bounds: arena boundary not yet established"

    # Check robot circle fits inside boundary
    robot_polygon = circle_to_polygon(x, y, radius, n_vertices=16)
    all_inside, outside_indices = polygon_contains_polygon(
        list(boundary), robot_polygon
    )
    if not all_inside:
        return False, "out_of_bounds: robot extends outside arena boundary"

    # Check collision with existing robots
    for robot in existing_robots:
        if robot.status != "active":
            continue
        dx = x - robot.x
        dy = y - robot.y
        dist = math.sqrt(dx * dx + dy * dy)
        min_dist = radius + robot.radius
        if dist < min_dist:
            return False, f"collision: too close to robot {robot.robot_id}"

    return True, ""


class Registry:
    """Thread-safe robot registry."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._robots: dict[int, Robot] = {}
        self._next_id: int = 1

    def register_physical(
        self,
        localization_id: str,
        localization_source: PoseSource,
        radius_m: float,
        x: float,
        y: float,
        theta: float,
    ) -> tuple[bool, int, str]:
        """Register a physical robot.

        If the localization_id was previously registered but is inactive,
        restores the same robot_id.

        Args:
            localization_id: Provider-specific ID (e.g. tag "3", "rover-1").
            localization_source: Which localization backend produced the pose.
            radius_m: Robot radius in metres.
            x: Current X position from detection.
            y: Current Y position from detection.
            theta: Current heading from detection.

        Returns:
            Tuple of (success, robot_id, message).
        """
        with self._lock:
            # Check for duplicate active localization_id
            for entry in self._robots.values():
                if entry.localization_id == localization_id and entry.status == "active":
                    return False, -1, "ERR DUPLICATE"

            # Check for inactive entry with same localization_id (reconnection)
            for entry in self._robots.values():
                if entry.localization_id == localization_id and entry.status == "inactive":
                    restored = replace(
                        entry, x=x, y=y, theta=theta, radius=radius_m,
                        status="active",
                    )
                    self._robots[entry.robot_id] = restored
                    logger.info(
                        "Physical robot restored: id=%d, loc_id=%s",
                        entry.robot_id, localization_id,
                    )
                    return True, entry.robot_id, "OK"

            # New registration
            robot_id = self._next_id
            self._next_id += 1
            entry = Robot(
                robot_id=robot_id,
                deployment="physical",
                x=x, y=y, theta=theta,
                radius=radius_m,
                status="active",
                localization_id=localization_id,
                localization_source=localization_source,
            )
            self._robots[robot_id] = entry
            logger.info(
                "Physical robot registered: id=%d, loc_id=%s",
                robot_id, localization_id,
            )
            return True, robot_id, "OK"

    def spawn_virtual(
        self,
        x: float,
        y: float,
        theta: float,
        radius: float,
        boundary: tuple[tuple[float, float], ...],
    ) -> tuple[bool, int, str]:
        """Spawn a virtual robot with boundary and collision validation.

        Args:
            x: Spawn X position.
            y: Spawn Y position.
            theta: Spawn heading.
            radius: Robot radius (0 for default).
            boundary: Arena boundary polygon.

        Returns:
            Tuple of (success, robot_id, message).
        """
        if radius <= 0:
            radius = DEFAULT_RADIUS

        with self._lock:
            active_robots = [
                r for r in self._robots.values() if r.status == "active"
            ]

        valid, error = validate_spawn_position(
            x, y, radius, boundary, active_robots
        )
        if not valid:
            return False, -1, error

        with self._lock:
            robot_id = self._next_id
            self._next_id += 1
            entry = Robot(
                robot_id=robot_id,
                deployment="virtual",
                x=x, y=y, theta=theta,
                radius=radius,
                status="active",
            )
            self._robots[robot_id] = entry
            logger.info("Virtual robot spawned: id=%d at (%.3f, %.3f)", robot_id, x, y)
            return True, robot_id, "OK"

    def remove_robot(self, robot_id: int) -> tuple[bool, str]:
        """Remove a virtual robot. Physical robots cannot be removed via API.

        Returns:
            Tuple of (success, message).
        """
        with self._lock:
            entry = self._robots.get(robot_id)
            if entry is None:
                return False, f"Robot {robot_id} not found"
            if entry.deployment == "physical":
                return False, f"Cannot remove physical robot {robot_id}"
            del self._robots[robot_id]
            logger.info("Virtual robot removed: id=%d", robot_id)
            return True, "OK"

    def reset_robot(self, robot_id: int) -> tuple[bool, str]:
        """Force-reset a physical robot (mark inactive, release for re-matching).

        Returns:
            Tuple of (success, message).
        """
        with self._lock:
            entry = self._robots.get(robot_id)
            if entry is None:
                return False, f"Robot {robot_id} not found"
            if entry.deployment == "virtual":
                return False, f"Cannot reset virtual robot {robot_id} — use remove"
            self._robots[robot_id] = replace(entry, status="inactive")
            logger.info("Physical robot reset: id=%d", robot_id)
            return True, "OK"

    def list_robots(self) -> list[Robot]:
        """Return a snapshot of all registered robots."""
        with self._lock:
            return list(self._robots.values())

    def get(self, robot_id: int) -> Robot | None:
        """Get a single robot entry."""
        with self._lock:
            return self._robots.get(robot_id)

    def find_by_localization_id(self, localization_id: str) -> Robot | None:
        """Find a robot by its localization ID."""
        with self._lock:
            for entry in self._robots.values():
                if entry.localization_id == localization_id:
                    return entry
            return None

    def update_pose(
        self, robot_id: int, x: float, y: float, theta: float
    ) -> bool:
        """Update the pose of a registered robot.

        Returns True if the robot was found and updated.
        """
        with self._lock:
            entry = self._robots.get(robot_id)
            if entry is None:
                return False
            self._robots[robot_id] = replace(entry, x=x, y=y, theta=theta)
            return True

    def mark_inactive(self, robot_id: int) -> bool:
        """Mark a robot as inactive (e.g. heartbeat timeout).

        Returns True if the robot was found.
        """
        with self._lock:
            entry = self._robots.get(robot_id)
            if entry is None:
                return False
            self._robots[robot_id] = replace(entry, status="inactive")
            logger.info("Robot %d marked inactive", robot_id)
            return True
