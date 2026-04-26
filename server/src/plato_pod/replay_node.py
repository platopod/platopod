"""Replay Node — drives virtual robots along recorded GPS tracks.

Loads GPS tracks from GPX or YAML files, converts to arena coordinates,
and publishes virtual robot poses at the configured playback speed.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node

from plato_pod.geo_reference import GeoReference
from plato_pod.replay import (
    ReplayTrack,
    interpolate_arena_position,
    load_gpx,
    load_replay_yaml,
)

import yaml
from pathlib import Path


class ReplayNode(Node):
    """ROS2 node replaying recorded GPS tracks as virtual robot positions."""

    def __init__(self) -> None:
        super().__init__("replay_node")

        self.declare_parameter("exercise_file", "")
        self.declare_parameter("playback_speed", 1)
        self.declare_parameter("scale_factor", 1)
        self.declare_parameter("tick_rate_hz", 10)

        exercise_file = self.get_parameter("exercise_file").value
        self._playback_speed = float(self.get_parameter("playback_speed").value) or 1.0
        scale_factor = float(self.get_parameter("scale_factor").value) or 1.0
        tick_rate = float(self.get_parameter("tick_rate_hz").value) or 10.0

        # Geo reference for coordinate conversion
        self._geo = GeoReference(
            origin_lat=-35.2975,
            origin_lon=149.1012,
            scale_factor=scale_factor,
        )

        # Load tracks
        self._tracks: list[ReplayTrack] = []
        self._pose_pubs: dict[int, object] = {}
        self._start_time = time.monotonic()

        if exercise_file:
            self._load_exercise(exercise_file)

        # Tick timer
        dt = 1.0 / tick_rate
        self._timer = self.create_timer(dt, self._tick)

        self.get_logger().info(
            f"Replay node ready: {len(self._tracks)} tracks, "
            f"speed={self._playback_speed}x, scale={scale_factor}"
        )

    def _load_exercise(self, exercise_file: str) -> None:
        """Load replay tracks from exercise YAML."""
        try:
            with open(exercise_file) as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Cannot load exercise: {e}")
            return

        exercise = config.get("exercise", config)

        # Read geo reference from exercise
        geo_cfg = exercise.get("geo", {})
        if geo_cfg:
            self._geo = GeoReference(
                origin_lat=geo_cfg.get("origin_lat", self._geo.origin_lat),
                origin_lon=geo_cfg.get("origin_lon", self._geo.origin_lon),
                origin_alt=geo_cfg.get("origin_alt", self._geo.origin_alt),
                rotation_deg=geo_cfg.get("rotation_deg", self._geo.rotation_deg),
                scale_factor=geo_cfg.get("scale_factor", self._geo.scale_factor),
            )

        # Load replay tracks
        replay_cfg = exercise.get("replay", {})
        tracks_file = replay_cfg.get("tracks_file", "")
        self._playback_speed = replay_cfg.get(
            "playback_speed", self._playback_speed
        )

        if tracks_file:
            tracks_path = Path(tracks_file)
            if not tracks_path.is_absolute():
                tracks_path = Path(exercise_file).parent / tracks_path

            if tracks_path.suffix == ".gpx":
                self._tracks = load_gpx(tracks_path)
            elif tracks_path.suffix in (".yaml", ".yml"):
                self._tracks = load_replay_yaml(tracks_path)
            else:
                self.get_logger().error(f"Unknown track format: {tracks_path.suffix}")
                return

            self.get_logger().info(f"Loaded {len(self._tracks)} tracks from {tracks_path}")

        # Assign robot IDs from exercise config
        robots = exercise.get("robots", {})
        physical = robots.get("physical", [])
        for robot_cfg in physical:
            track_id = robot_cfg.get("replay_track", "")
            if track_id:
                for track in self._tracks:
                    if track.track_id == track_id:
                        track.robot_id = robot_cfg.get(
                            "tag_id", robot_cfg.get("id", -1)
                        )
                        track.team = robot_cfg.get("team", "")
                        track.vehicle_role = robot_cfg.get("vehicle_role", "recon")
                        break

        # Auto-assign IDs to unmatched tracks
        next_id = 100
        for track in self._tracks:
            if track.robot_id < 0:
                track.robot_id = next_id
                next_id += 1

        # Create pose publishers
        for track in self._tracks:
            topic = f"/robot_{track.robot_id}/pose"
            self._pose_pubs[track.robot_id] = self.create_publisher(
                Pose2D, topic, 10
            )

    def _tick(self) -> None:
        """Publish interpolated positions for all tracks."""
        elapsed = (time.monotonic() - self._start_time) * self._playback_speed

        for track in self._tracks:
            result = interpolate_arena_position(track, elapsed, self._geo)
            if result is None:
                continue

            x, y, theta = result
            pub = self._pose_pubs.get(track.robot_id)
            if pub is not None:
                msg = Pose2D()
                msg.x = x
                msg.y = y
                msg.theta = theta
                pub.publish(msg)


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod replay_node."""
    rclpy.init(args=args)
    node = ReplayNode()
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
