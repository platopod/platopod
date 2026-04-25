"""Control manager — tracks robot ownership, velocity watchdog, and control timeout.

Thread-safe. Manages which client controls which robot, fires watchdog events
when velocity commands stop, and releases control after prolonged inactivity.
No ROS2 dependency.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass


@dataclass
class ControlEntry:
    """Tracks a client's control of a robot."""
    client_id: str
    robot_id: int
    last_cmd_time: float        # monotonic time of last cmd_vel
    last_linear_x: float
    last_angular_z: float
    watchdog_fired: bool        # True after watchdog zeroed velocity


@dataclass(frozen=True, slots=True)
class WatchdogEvent:
    """Fired when velocity watchdog triggers (velocity → zero)."""
    robot_id: int
    client_id: str


@dataclass(frozen=True, slots=True)
class ControlReleasedEvent:
    """Fired when control timeout releases a robot."""
    robot_id: int
    client_id: str


class ControlManager:
    """Thread-safe control ownership and timeout manager."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._controls: dict[int, ControlEntry] = {}  # robot_id -> entry

    def try_acquire(self, client_id: str, robot_id: int) -> tuple[bool, str]:
        """Try to acquire control of a robot.

        First cmd_vel acquires control. Same client can re-acquire.
        Another client gets robot_busy.

        Returns:
            (success, error_code). error_code is "robot_busy" on failure.
        """
        with self._lock:
            entry = self._controls.get(robot_id)
            if entry is None or entry.client_id == client_id:
                return True, ""
            return False, "robot_busy"

    def record_command(
        self, client_id: str, robot_id: int,
        linear_x: float, angular_z: float, now: float,
    ) -> None:
        """Record a cmd_vel command. Creates or updates the control entry."""
        with self._lock:
            self._controls[robot_id] = ControlEntry(
                client_id=client_id,
                robot_id=robot_id,
                last_cmd_time=now,
                last_linear_x=linear_x,
                last_angular_z=angular_z,
                watchdog_fired=False,
            )

    def release(self, client_id: str, robot_id: int) -> bool:
        """Release control of a specific robot. Only the controlling client can release.

        Returns True if released.
        """
        with self._lock:
            entry = self._controls.get(robot_id)
            if entry is not None and entry.client_id == client_id:
                del self._controls[robot_id]
                return True
            return False

    def release_all(self, client_id: str) -> list[int]:
        """Release all robots controlled by a client (e.g. on disconnect).

        Returns list of released robot_ids.
        """
        with self._lock:
            released = []
            for robot_id, entry in list(self._controls.items()):
                if entry.client_id == client_id:
                    del self._controls[robot_id]
                    released.append(robot_id)
            return released

    def get_controller(self, robot_id: int) -> str | None:
        """Get the client_id controlling this robot, or None."""
        with self._lock:
            entry = self._controls.get(robot_id)
            return entry.client_id if entry else None

    def controlled_robots(self, client_id: str) -> list[int]:
        """Get all robot_ids controlled by a client."""
        with self._lock:
            return [
                entry.robot_id for entry in self._controls.values()
                if entry.client_id == client_id
            ]

    def tick(
        self, now: float,
        watchdog_timeout: float = 0.5,
        control_timeout: float = 30.0,
    ) -> tuple[list[WatchdogEvent], list[ControlReleasedEvent]]:
        """Check all controlled robots for watchdog and control timeouts.

        Args:
            now: Current monotonic time.
            watchdog_timeout: Seconds before zeroing velocity (default 500ms).
            control_timeout: Seconds before releasing control (default 30s).

        Returns:
            (watchdog_events, control_released_events).
        """
        watchdog_events: list[WatchdogEvent] = []
        release_events: list[ControlReleasedEvent] = []

        with self._lock:
            to_release: list[int] = []

            for robot_id, entry in self._controls.items():
                elapsed = now - entry.last_cmd_time

                # Control timeout — release control
                if elapsed > control_timeout:
                    release_events.append(ControlReleasedEvent(
                        robot_id=robot_id,
                        client_id=entry.client_id,
                    ))
                    to_release.append(robot_id)
                    continue

                # Velocity watchdog — zero velocity but keep control
                if elapsed > watchdog_timeout and not entry.watchdog_fired:
                    had_velocity = (
                        abs(entry.last_linear_x) > 1e-6
                        or abs(entry.last_angular_z) > 1e-6
                    )
                    if had_velocity:
                        watchdog_events.append(WatchdogEvent(
                            robot_id=robot_id,
                            client_id=entry.client_id,
                        ))
                    entry.watchdog_fired = True

            for robot_id in to_release:
                del self._controls[robot_id]

        return watchdog_events, release_events
