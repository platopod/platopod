"""Ghost model manager — tracks physical robots in the Gazebo world.

When physical robots operate on a desktop arena at classroom scale
(e.g., 0.84m), their positions are scaled up and projected into the
Gazebo world (e.g., 840m). A "ghost" model is spawned in Gazebo at
each robot's scaled position. The ghost carries Gazebo sensors (lidar,
IMU, camera) that interact with the real terrain and obstacles.

The ghost model manager handles spawning, pose updates, and removal.
Pose updates use `gz service` calls at a configurable rate.

No ROS2 dependency — uses only subprocess and the gz CLI.
"""

from __future__ import annotations

import logging
import subprocess
import threading
from dataclasses import dataclass

from plato_pod.geo_reference import GeoReference

logger = logging.getLogger(__name__)

# Vehicle role → model name (same mapping as world builder)
ROLE_TO_MODEL: dict[str, str] = {
    "default": "platopod",
    "recon": "recon",
    "cbrn_recon": "recon",
    "tank": "tank",
    "apc": "apc",
    "artillery": "tank",
    "sensor": "platopod",
}


@dataclass
class GhostState:
    """Tracked state for a ghost model."""
    robot_id: int
    model_name: str         # Gazebo model name (e.g., "ghost_1")
    vehicle_role: str
    world_x: float = 0.0
    world_y: float = 0.0
    world_z: float = 0.0
    theta: float = 0.0
    spawned: bool = False


class GhostModelManager:
    """Manages ghost models in Gazebo for physical desktop robots.

    Each physical robot gets a ghost model spawned in the Gazebo world
    at its scaled position. The ghost carries the sensors that interact
    with the Gazebo terrain.
    """

    def __init__(
        self,
        geo_ref: GeoReference,
        model_path: str = "models",
        world_name: str = "arena",
    ) -> None:
        self._geo = geo_ref
        self._model_path = model_path
        self._world_name = world_name
        self._lock = threading.Lock()
        self._ghosts: dict[int, GhostState] = {}

    def spawn_ghost(
        self,
        robot_id: int,
        vehicle_role: str,
        arena_x: float,
        arena_y: float,
        theta: float,
    ) -> bool:
        """Spawn a ghost model in Gazebo at the scaled position.

        Args:
            robot_id: Physical robot's ID.
            vehicle_role: Vehicle type for model selection.
            arena_x: Robot X in arena frame (desktop scale).
            arena_y: Robot Y in arena frame (desktop scale).
            theta: Robot heading in radians.

        Returns:
            True if spawn succeeded.
        """
        world_x, world_y = self._geo.arena_to_world(arena_x, arena_y)
        model_type = ROLE_TO_MODEL.get(vehicle_role, "platopod")
        model_name = f"ghost_{robot_id}"

        state = GhostState(
            robot_id=robot_id,
            model_name=model_name,
            vehicle_role=vehicle_role,
            world_x=world_x,
            world_y=world_y,
            world_z=0.1,
            theta=theta,
        )

        # Use gz service to spawn the model
        sdf_uri = f"model://{model_type}"
        req = (
            f'sdf: "<include><uri>{sdf_uri}</uri>'
            f'<name>{model_name}</name></include>" '
            f'name: "{model_name}" '
            f'pose: {{position: {{x: {world_x}, y: {world_y}, z: 0.1}}, '
            f'orientation: {{x: 0, y: 0, z: {theta}, w: 1}}}}'
        )

        try:
            result = subprocess.run(
                ["gz", "service",
                 "-s", f"/world/{self._world_name}/create",
                 "--reqtype", "gz.msgs.EntityFactory",
                 "--reptype", "gz.msgs.Boolean",
                 "--req", req,
                 "--timeout", "5000"],
                capture_output=True, timeout=10,
            )
            if result.returncode == 0:
                state.spawned = True
                with self._lock:
                    self._ghosts[robot_id] = state
                logger.info(
                    f"Ghost {model_name} spawned at ({world_x:.1f}, {world_y:.1f}) "
                    f"model={model_type}"
                )
                return True
            else:
                stderr = result.stderr.decode("utf-8", errors="replace")[:200]
                logger.error(f"Failed to spawn ghost {model_name}: {stderr}")
                return False
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            logger.error(f"gz command failed: {e}")
            return False

    def update_pose(
        self,
        robot_id: int,
        arena_x: float,
        arena_y: float,
        theta: float,
    ) -> bool:
        """Move a ghost model to match the physical robot's scaled position.

        Args:
            robot_id: Physical robot's ID.
            arena_x: Robot X in arena frame (desktop scale).
            arena_y: Robot Y in arena frame (desktop scale).
            theta: Robot heading in radians.

        Returns:
            True if the update was sent.
        """
        with self._lock:
            state = self._ghosts.get(robot_id)
            if state is None or not state.spawned:
                return False

        world_x, world_y = self._geo.arena_to_world(arena_x, arena_y)
        state.world_x = world_x
        state.world_y = world_y
        state.theta = theta

        req = (
            f'name: "{state.model_name}" '
            f'position: {{x: {world_x}, y: {world_y}, z: {state.world_z}}} '
            f'orientation: {{x: 0, y: 0, z: {theta}, w: 1}}'
        )

        try:
            subprocess.run(
                ["gz", "service",
                 "-s", f"/world/{self._world_name}/set_pose",
                 "--reqtype", "gz.msgs.Pose",
                 "--reptype", "gz.msgs.Boolean",
                 "--req", req,
                 "--timeout", "1000"],
                capture_output=True, timeout=3,
            )
            return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def remove_ghost(self, robot_id: int) -> bool:
        """Remove a ghost model from Gazebo.

        Returns:
            True if the model was removed.
        """
        with self._lock:
            state = self._ghosts.pop(robot_id, None)
            if state is None:
                return False

        req = f'name: "{state.model_name}" type: MODEL'

        try:
            subprocess.run(
                ["gz", "service",
                 "-s", f"/world/{self._world_name}/remove",
                 "--reqtype", "gz.msgs.Entity",
                 "--reptype", "gz.msgs.Boolean",
                 "--req", req,
                 "--timeout", "5000"],
                capture_output=True, timeout=10,
            )
            logger.info(f"Ghost {state.model_name} removed")
            return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def get_ghost_ids(self) -> list[int]:
        """Return IDs of all active ghost models."""
        with self._lock:
            return [rid for rid, s in self._ghosts.items() if s.spawned]

    def get_ghost_state(self, robot_id: int) -> GhostState | None:
        """Return the state of a ghost model, or None if not tracked."""
        with self._lock:
            return self._ghosts.get(robot_id)

    def has_ghost(self, robot_id: int) -> bool:
        """Check if a ghost model exists for a robot."""
        with self._lock:
            state = self._ghosts.get(robot_id)
            return state is not None and state.spawned
