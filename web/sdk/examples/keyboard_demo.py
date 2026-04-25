#!/usr/bin/env python3
"""Keyboard control demo — drive a robot with arrow keys.

Usage:
    python3 keyboard_demo.py [--url ws://192.168.4.1:8080/api/control] [--robot-id 1]
"""

import argparse
import sys

# Allow running from the examples/ directory
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent.parent))

from platopod import Arena, KeyboardController


def main() -> None:
    parser = argparse.ArgumentParser(description="Plato Pod Keyboard Control")
    parser.add_argument(
        "--url", default="ws://localhost:8080/api/control",
        help="WebSocket URL (default: ws://localhost:8080/api/control)",
    )
    parser.add_argument(
        "--robot-id", type=int, default=None,
        help="Robot ID to control (default: prompt)",
    )
    parser.add_argument(
        "--speed", type=float, default=0.15,
        help="Linear speed (m/s, default: 0.15)",
    )
    parser.add_argument(
        "--turn-speed", type=float, default=1.5,
        help="Angular speed (rad/s, default: 1.5)",
    )
    args = parser.parse_args()

    print(f"Connecting to {args.url}...")
    arena = Arena(args.url)

    robot_id = args.robot_id
    if robot_id is None:
        robots = arena.list_robots()
        if robots:
            print("\nAvailable robots:")
            for r in robots:
                print(f"  ID {r['robot_id']}: {r['type']} ({r['status']})")
            print()
        robot_id = int(input("Enter robot ID: "))

    print(f"\nControlling robot {robot_id}. Arrow keys to move, 'q' to quit.\n")

    controller = KeyboardController(arena, robot_id, args.speed, args.turn_speed)
    try:
        controller.run()
    finally:
        arena.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
