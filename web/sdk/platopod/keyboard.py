"""Keyboard controller — drive a robot with arrow keys.

Uses curses for terminal key capture. Sends cmd_vel at 10 Hz.
Arrow keys: Up=forward, Down=backward, Left=turn left, Right=turn right.
Press 'q' to quit.
"""

from __future__ import annotations

import curses
import time

from platopod.arena import Arena


class KeyboardController:
    """Drive a robot using arrow keys."""

    def __init__(
        self,
        arena: Arena,
        robot_id: int,
        speed: float = 0.15,
        turn_speed: float = 1.5,
    ) -> None:
        """Create a keyboard controller.

        Args:
            arena: Connected Arena instance.
            robot_id: Robot to control.
            speed: Linear speed when arrow keys are pressed (m/s).
            turn_speed: Angular speed when arrow keys are pressed (rad/s).
        """
        self._arena = arena
        self._robot_id = robot_id
        self._speed = speed
        self._turn_speed = turn_speed

    def run(self) -> None:
        """Block and listen for keyboard input. Press 'q' to stop."""
        curses.wrapper(self._run_curses)

    def _run_curses(self, stdscr) -> None:
        """Curses main loop."""
        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0, f"Controlling robot {self._robot_id}")
        stdscr.addstr(1, 0, "Arrow keys: move | q: quit")
        stdscr.addstr(2, 0, f"Speed: {self._speed} m/s | Turn: {self._turn_speed} rad/s")
        stdscr.refresh()

        forward = False
        backward = False
        left = False
        right = False

        tick_interval = 0.1  # 10 Hz

        while True:
            # Read all pending keys
            while True:
                key = stdscr.getch()
                if key == -1:
                    break

                if key == ord('q'):
                    self._arena.cmd_vel(self._robot_id, 0.0, 0.0)
                    return

                if key == curses.KEY_UP:
                    forward = True
                    backward = False
                elif key == curses.KEY_DOWN:
                    backward = True
                    forward = False
                elif key == curses.KEY_LEFT:
                    left = True
                    right = False
                elif key == curses.KEY_RIGHT:
                    right = True
                    left = False

            # Compute velocity from key state
            linear_x = 0.0
            angular_z = 0.0

            if forward:
                linear_x = self._speed
            elif backward:
                linear_x = -self._speed

            if left:
                angular_z = self._turn_speed
            elif right:
                angular_z = -self._turn_speed

            self._arena.cmd_vel(self._robot_id, linear_x, angular_z)

            # Display state
            stdscr.addstr(4, 0, f"linear: {linear_x:+.2f}  angular: {angular_z:+.2f}  ")
            stdscr.refresh()

            # Reset key state each tick (impulse-based: keys must be held)
            forward = backward = left = right = False

            time.sleep(tick_interval)
