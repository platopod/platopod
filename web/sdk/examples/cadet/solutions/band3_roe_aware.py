"""Band 3 reference solution — gradient ascent with ROE awareness.

Expected: ≤80 ticks AND zero civilian violations on the `civilians`
scenario. Distinction-level solution.

Demonstrates: combining sensor data with positional reasoning,
multi-constraint decision making, defensive backing-off, and the
search-then-track pattern from Band 2.
"""

from __future__ import annotations


DETECTABLE = 5.0


def decide(reading: dict, pose: dict) -> tuple[float, float]:
    # Rule 1 — slow down near civilians. Moving fast (>0.05 m/s) inside
    # civilian-proximity distance is an ROE violation in itself.
    # Apply this clamp BEFORE the red-zone retreat so a panicked dash
    # away from gas doesn't trample bystanders.
    base_speed = 0.05 if reading["civilian_proximity"] else 0.10

    # Rule 2 — survive. If the red-zone flag is set, retreat — but use
    # the civilian-aware speed limit when applicable.
    if reading["in_red_zone"]:
        return -base_speed, 0.0

    history = reading["history"]
    current = reading["concentration"]

    # Search phase — no signal yet. Drive forward at base speed.
    if current < DETECTABLE:
        return base_speed, 0.0

    # Tracking phase — gradient ascent.
    if len(history) < 2:
        return base_speed, 0.0
    delta = history[-1] - history[-2]
    if delta >= 0.0:
        return base_speed, 0.0

    # Falling — turn while sampling. Tighter turn near civilians so
    # we don't sweep through their proximity zone.
    angular = 1.5 if reading["civilian_proximity"] else 1.0
    return base_speed, angular
