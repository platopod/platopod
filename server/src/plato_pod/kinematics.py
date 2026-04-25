"""Differential-drive kinematics for virtual robot simulation.

Pure functions for kinematic updates, angle normalization, optional noise
injection, acceleration ramping, and boundary clamping. No ROS2 dependency.
"""

from __future__ import annotations

import math
import random

from plato_pod.geometry import point_in_polygon


def normalize_angle(theta: float) -> float:
    """Wrap angle to [-pi, pi].

    Args:
        theta: Angle in radians.

    Returns:
        Normalized angle in [-pi, pi].
    """
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def apply_acceleration_limit(
    current_vel: float,
    target_vel: float,
    accel_limit: float,
    dt: float,
) -> float:
    """Ramp velocity toward target respecting acceleration limit.

    Args:
        current_vel: Current velocity.
        target_vel: Desired velocity.
        accel_limit: Maximum acceleration (units/s²). 0 means instant.
        dt: Time step (seconds).

    Returns:
        New velocity after applying the acceleration limit.
    """
    if accel_limit <= 0:
        return target_vel

    diff = target_vel - current_vel
    max_change = accel_limit * dt

    if abs(diff) <= max_change:
        return target_vel
    elif diff > 0:
        return current_vel + max_change
    else:
        return current_vel - max_change


def add_velocity_noise(
    linear_x: float,
    angular_z: float,
    noise_stddev: float,
) -> tuple[float, float]:
    """Add Gaussian noise to velocity commands.

    Args:
        linear_x: Linear velocity (m/s).
        angular_z: Angular velocity (rad/s).
        noise_stddev: Standard deviation of noise. 0 means no noise.

    Returns:
        Noisy (linear_x', angular_z').
    """
    if noise_stddev <= 0:
        return linear_x, angular_z

    return (
        linear_x + random.gauss(0, noise_stddev),
        angular_z + random.gauss(0, noise_stddev),
    )


def add_pose_drift(
    x: float, y: float, theta: float,
    drift_stddev: float,
) -> tuple[float, float, float]:
    """Add Gaussian noise to the pose (simulating odometry drift).

    Args:
        x: Current X position.
        y: Current Y position.
        theta: Current heading.
        drift_stddev: Standard deviation of drift. 0 means no drift.

    Returns:
        Drifted (x', y', theta').
    """
    if drift_stddev <= 0:
        return x, y, theta

    return (
        x + random.gauss(0, drift_stddev),
        y + random.gauss(0, drift_stddev),
        normalize_angle(theta + random.gauss(0, drift_stddev)),
    )


def clamp_to_boundary(
    x: float, y: float, radius: float,
    boundary: tuple[tuple[float, float], ...],
) -> tuple[float, float]:
    """Project a robot position back inside the arena boundary if needed.

    Simple fallback: if the center is outside, find the nearest boundary
    edge center and move toward it. This is a safety net — the upstream
    command pipeline should prevent boundary crossing.

    Args:
        x: Robot center X.
        y: Robot center Y.
        radius: Robot radius (metres).
        boundary: Arena boundary polygon vertices.

    Returns:
        Clamped (x', y'). Unchanged if already inside.
    """
    if len(boundary) < 3:
        return x, y

    if point_in_polygon((x, y), list(boundary)):
        return x, y

    # Find nearest point on boundary edges
    best_x, best_y = x, y
    best_dist_sq = float('inf')

    n = len(boundary)
    for i in range(n):
        ax, ay = boundary[i]
        bx, by = boundary[(i + 1) % n]

        # Project point onto line segment
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-12:
            continue

        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len_sq))
        px = ax + t * dx
        py = ay + t * dy

        dist_sq = (x - px) ** 2 + (y - py) ** 2
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            # Move slightly inside (by radius) toward the polygon interior
            # Simple approach: move toward the centroid
            best_x, best_y = px, py

    # Move the result slightly inward by the robot radius
    cx = sum(v[0] for v in boundary) / n
    cy = sum(v[1] for v in boundary) / n
    dx = cx - best_x
    dy = cy - best_y
    dist = math.sqrt(dx * dx + dy * dy)
    if dist > 1e-6:
        best_x += dx / dist * radius
        best_y += dy / dist * radius

    return best_x, best_y
