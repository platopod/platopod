"""Gazebo Bridge Node — glue between Gazebo and the Plato Pod platform.

Manages the Gazebo simulation lifecycle: generates SDF world from exercise
YAML, launches gz-sim as a subprocess, launches ros_gz_bridge to bridge
poses, commands, and sensor data between Gazebo and ROS2 topics.
"""

from __future__ import annotations

import os
import subprocess
import tempfile

import rclpy
from geometry_msgs.msg import Pose, Pose2D
from rclpy.node import Node

from plato_pod.gazebo_world_builder import build_world_sdf
from plato_pod.geo_reference import GeoReference
from plato_pod.ghost_model_manager import GhostModelManager
from plato_pod.providers.gazebo_provider import GazeboProvider
from plato_pod.terrain_pipeline import TerrainConfig, generate_terrain, terrain_config_from_yaml

import yaml


def _detect_gpu() -> bool:
    """Check if a GPU is available for Gazebo rendering."""
    try:
        result = subprocess.run(
            ["which", "nvidia-smi"],
            capture_output=True, timeout=5,
        )
        if result.returncode == 0:
            return True
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    # Check for any GPU via /dev/dri
    return os.path.exists("/dev/dri/renderD128")


class GazeboBridgeNode(Node):
    """ROS2 node bridging Gazebo simulation to the Plato Pod platform."""

    def __init__(self) -> None:
        super().__init__("gazebo_bridge_node")

        # Parameters
        self.declare_parameter("exercise_file", "")
        self.declare_parameter("headless", False)
        self.declare_parameter("physics_step_size", 0.001)
        self.declare_parameter("real_time_factor", 1.0)
        self.declare_parameter("pose_publish_rate_hz", 50.0)
        self.declare_parameter("model_path", "models")
        self.declare_parameter("wall_height", 0.3)
        self.declare_parameter("bridge_sensors", True)
        self.declare_parameter("scale_factor", 1.0)
        self.declare_parameter("ghost_update_rate_hz", 10.0)

        exercise_file = self.get_parameter("exercise_file").value
        headless = self.get_parameter("headless").value
        physics_step = self.get_parameter("physics_step_size").value
        rtf = self.get_parameter("real_time_factor").value
        pose_rate = self.get_parameter("pose_publish_rate_hz").value
        model_path = self.get_parameter("model_path").value
        wall_height = self.get_parameter("wall_height").value
        self._bridge_sensors = self.get_parameter("bridge_sensors").value
        scale_factor = self.get_parameter("scale_factor").value
        ghost_rate = self.get_parameter("ghost_update_rate_hz").value

        # GPU detection
        self._gpu_available = _detect_gpu()
        self.get_logger().info(
            f"GPU {'detected' if self._gpu_available else 'not found'} — "
            f"using {'gpu_lidar' if self._gpu_available else 'cpu ray sensor'}"
        )

        # Provider
        self._provider = GazeboProvider()

        # Load exercise and generate world
        self._robots: list[dict] = []
        self._world_file: str | None = None
        self._gz_process: subprocess.Popen | None = None
        self._bridge_process: subprocess.Popen | None = None

        if exercise_file:
            self._generate_and_launch(
                exercise_file, model_path, physics_step, rtf, wall_height, headless
            )

        # Per-robot pose publishers and cmd_vel subscribers
        self._pose_pubs: dict[int, object] = {}
        self._cmd_vel_subs: dict[int, object] = {}

        for r in self._robots:
            rid = r.get("id", r.get("robot_id", 0))
            self._setup_robot_topics(rid)

        # Pose publish timer
        pose_period = 1.0 / pose_rate
        self._pose_timer = self.create_timer(pose_period, self._publish_poses)

        # Ghost model manager for physical robots at classroom scale
        self._scale_factor = scale_factor
        self._ghost_manager: GhostModelManager | None = None
        self._physical_poses: dict[int, tuple[float, float, float]] = {}

        if scale_factor > 1.0:
            geo = GeoReference(
                origin_lat=0.0, origin_lon=0.0,
                scale_factor=scale_factor,
            )
            self._ghost_manager = GhostModelManager(
                geo_ref=geo, model_path=model_path,
            )
            # Subscribe to robot status for physical robot detection
            from plato_pod_msgs.msg import RobotStatusList as RobotStatusListMsg
            self.create_subscription(
                RobotStatusListMsg, "/robots/status",
                self._robot_status_for_ghosts, 10,
            )
            # Ghost pose update timer
            ghost_period = 1.0 / ghost_rate
            self._ghost_timer = self.create_timer(ghost_period, self._update_ghosts)
            self.get_logger().info(
                f"Ghost model manager active: scale={scale_factor}, "
                f"update rate={ghost_rate} Hz"
            )

        self.get_logger().info(
            f"Gazebo bridge ready: {len(self._robots)} robots, "
            f"headless={headless}, sensors_bridged={self._bridge_sensors}"
        )

    def _generate_and_launch(
        self, exercise_file: str, model_path: str,
        physics_step: float, rtf: float, wall_height: float,
        headless: bool,
    ) -> None:
        """Generate SDF world from exercise YAML and launch gz-sim."""
        try:
            with open(exercise_file) as f:
                config = yaml.safe_load(f)
        except (FileNotFoundError, yaml.YAMLError) as e:
            self.get_logger().error(f"Cannot load exercise: {e}")
            return

        exercise = config.get("exercise", config)
        arena = exercise.get("arena", {})
        scoring = exercise.get("scoring", {})

        boundary = arena.get("boundary", [])
        obstacles = arena.get("obstacles", [])
        zones = scoring.get("zones", [])
        raw_robots = exercise.get("robots", [])
        if isinstance(raw_robots, dict):
            raw_robots = raw_robots.get("physical", []) + raw_robots.get("virtual", [])

        # Normalize robot dicts
        self._robots = []
        for i, r in enumerate(raw_robots):
            robot = dict(r)
            robot["id"] = r.get("id", r.get("robot_id", r.get("tag_id", i + 1)))
            if "x" not in robot and boundary:
                t = (i + 0.5) / max(len(raw_robots), 1)
                idx = int(t * len(boundary))
                bx, by = boundary[idx % len(boundary)]
                robot.setdefault("x", bx * 0.5 + 0.1)
                robot.setdefault("y", by * 0.5 + 0.1)
            robot.setdefault("x", 0.1 * (i + 1))
            robot.setdefault("y", 0.1 * (i + 1))
            robot.setdefault("theta", 0.0)
            robot.setdefault("vehicle_role", "default")
            self._robots.append(robot)

        boundary_tuples = [(p[0], p[1]) for p in boundary] if boundary else []

        # Terrain pipeline: generate heightmap from DEM if configured
        terrain_dict = None
        terrain_cfg = terrain_config_from_yaml(config)
        if terrain_cfg and terrain_cfg.source != "flat":
            # Write temp files to a directory so heightmap PNG is alongside SDF
            import tempfile as _tf
            terrain_dir = _tf.mkdtemp(prefix="platopod_terrain_")
            result = generate_terrain(terrain_cfg, terrain_dir)
            if result is not None:
                terrain_dict = {
                    "heightmap_path": str(result.heightmap_path),
                    "size_x": result.size_x,
                    "size_y": result.size_y,
                    "size_z": result.size_z,
                }
                self.get_logger().info(
                    f"Terrain heightmap generated: {result.heightmap_path} "
                    f"({result.size_x:.0f}x{result.size_y:.0f}m, "
                    f"elev {result.min_elevation:.1f}–{result.max_elevation:.1f}m)"
                )

        # Generate SDF
        world_sdf = build_world_sdf(
            arena_boundary=boundary_tuples,
            obstacles=obstacles,
            zones=zones,
            robots=self._robots,
            terrain=terrain_dict,
            physics_step_size=physics_step,
            real_time_factor=rtf,
            wall_height=wall_height,
            model_path=os.path.abspath(model_path),
        )

        # Write to temp file
        fd, self._world_file = tempfile.mkstemp(suffix=".sdf", prefix="platopod_world_")
        os.close(fd)
        with open(self._world_file, "w") as f:
            f.write(world_sdf)

        self.get_logger().info(f"Generated world SDF: {self._world_file}")

        # Launch gz-sim
        self._launch_gazebo(model_path, headless)

        # Launch ros_gz_bridge for pose/cmd_vel and optionally sensors
        if self._gz_process is not None:
            self._launch_bridge()

    def _launch_gazebo(self, model_path: str, headless: bool) -> None:
        """Launch gz-sim subprocess."""
        cmd = ["gz", "sim", self._world_file]
        if headless:
            cmd.append("-s")

        env = os.environ.copy()
        models_dir = os.path.abspath(model_path)
        existing = env.get("GZ_SIM_RESOURCE_PATH", "")
        env["GZ_SIM_RESOURCE_PATH"] = f"{models_dir}:{existing}" if existing else models_dir

        try:
            self._gz_process = subprocess.Popen(
                cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            )
            self.get_logger().info(f"Gazebo launched (PID {self._gz_process.pid})")

            import time
            time.sleep(2.0)
            if self._gz_process.poll() is not None:
                _, stderr = self._gz_process.communicate(timeout=1)
                self.get_logger().error(
                    f"Gazebo exited with code {self._gz_process.returncode}: "
                    f"{stderr.decode('utf-8', errors='replace')[:500]}"
                )
                self._gz_process = None
            else:
                # Unpause the simulation
                subprocess.run(
                    ["gz", "service",
                     "-s", "/world/arena/control",
                     "--reqtype", "gz.msgs.WorldControl",
                     "--reptype", "gz.msgs.Boolean",
                     "--req", "pause: false",
                     "--timeout", "5000"],
                    capture_output=True, timeout=10,
                )
                self.get_logger().info("Gazebo simulation unpaused")
        except FileNotFoundError:
            self.get_logger().error(
                "gz command not found — install Gazebo: "
                "sudo apt install ros-jazzy-ros-gz"
            )

    def _launch_bridge(self) -> None:
        """Launch ros_gz_bridge for poses, commands, and sensors."""
        bridge_args = []

        for r in self._robots:
            rid = r.get("id", 0)
            model_name = f"robot_{rid}"

            # Pose: Gazebo → ROS2
            bridge_args.append(
                f"/model/{model_name}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose"
            )
            # cmd_vel: ROS2 → Gazebo
            bridge_args.append(
                f"/model/{model_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
            )

            if self._bridge_sensors:
                # Lidar: Gazebo → ROS2
                bridge_args.append(
                    f"/world/arena/model/{model_name}/link/base_link/sensor/lidar/scan"
                    f"@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
                )
                # IMU: Gazebo → ROS2
                bridge_args.append(
                    f"/world/arena/model/{model_name}/link/base_link/sensor/imu/imu"
                    f"@sensor_msgs/msg/Imu[gz.msgs.IMU"
                )

        if not bridge_args:
            return

        cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"] + bridge_args

        try:
            self._bridge_process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            )
            self.get_logger().info(
                f"ros_gz_bridge launched (PID {self._bridge_process.pid}), "
                f"{len(bridge_args)} topics bridged"
            )
        except FileNotFoundError:
            self.get_logger().error(
                "ros_gz_bridge not found — install: "
                "sudo apt install ros-jazzy-ros-gz-bridge"
            )

    def _setup_robot_topics(self, robot_id: int) -> None:
        """Create pose publisher and cmd_vel forwarding for one robot."""
        self._pose_pubs[robot_id] = self.create_publisher(
            Pose2D, f"/robot_{robot_id}/pose", 10
        )

        def pose_callback(msg: Pose, rid=robot_id) -> None:
            pos = msg.position
            ori = msg.orientation
            self._provider.update_pose(
                rid, pos.x, pos.y, pos.z,
                ori.x, ori.y, ori.z, ori.w,
                localization_id=str(rid),
            )

        # Subscribe to bridged pose topic
        # ros_gz_bridge publishes to /model/robot_{id}/pose
        self.create_subscription(
            Pose, f"/model/robot_{robot_id}/pose", pose_callback, 10
        )

    def _publish_poses(self) -> None:
        """Publish Gazebo robot poses to platform topics."""
        for pose in self._provider.get_poses():
            pub = self._pose_pubs.get(pose.robot_id)
            if pub is not None:
                msg = Pose2D()
                msg.x = pose.x
                msg.y = pose.y
                msg.theta = pose.theta
                pub.publish(msg)

    def _robot_status_for_ghosts(self, msg) -> None:
        """Track physical robot poses for ghost model updates."""
        for r in msg.robots:
            if r.deployment == "physical" and r.status == "active":
                self._physical_poses[r.robot_id] = (r.x, r.y, r.theta)
                # Spawn ghost if not yet created
                if self._ghost_manager and not self._ghost_manager.has_ghost(r.robot_id):
                    self._ghost_manager.spawn_ghost(
                        r.robot_id, "recon",  # default role, could come from config
                        r.x, r.y, r.theta,
                    )

    def _update_ghosts(self) -> None:
        """Update ghost model positions from physical robot poses."""
        if self._ghost_manager is None:
            return
        for rid, (x, y, theta) in self._physical_poses.items():
            self._ghost_manager.update_pose(rid, x, y, theta)

    @property
    def gpu_available(self) -> bool:
        """Whether a GPU was detected at startup."""
        return self._gpu_available

    def destroy_node(self) -> None:
        """Clean shutdown: terminate Gazebo and bridge."""
        if self._bridge_process is not None:
            self._bridge_process.terminate()
            try:
                self._bridge_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._bridge_process.kill()
            self.get_logger().info("ros_gz_bridge terminated")

        if self._gz_process is not None:
            self._gz_process.terminate()
            try:
                self._gz_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._gz_process.kill()
            self.get_logger().info("Gazebo terminated")

        if self._world_file and os.path.exists(self._world_file):
            os.unlink(self._world_file)

        super().destroy_node()


def main(args=None) -> None:
    """Entry point for ros2 run plato_pod gazebo_bridge_node."""
    rclpy.init(args=args)
    node = GazeboBridgeNode()
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
