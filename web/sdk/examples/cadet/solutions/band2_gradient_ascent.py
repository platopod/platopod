"""Band 2 reference solution — gradient ascent with search fallback.

Expected: finds the source in 60-130 ticks on the calm scenario.
Demonstrates: state via reading["history"], comparing values, handling
the "no signal yet" phase. ~15 lines of logic.
"""

from __future__ import annotations


# Threshold below which we consider the gas signal too weak to follow.
DETECTABLE = 5.0


def decide(reading: dict, pose: dict) -> tuple[float, float]:
    history = reading["history"]
    current = reading["concentration"]

    # Search phase — concentration too low to give a useful gradient.
    if current < DETECTABLE:
        return 0.10, 0.0   # drive forward; we're still hunting for the plume

    # Tracking phase — we have a signal, follow the gradient.
    if len(history) < 2:
        return 0.10, 0.0
    delta = history[-1] - history[-2]
    if delta >= 0:
        # Concentration is rising or steady — keep heading
        return 0.10, 0.0
    # Falling — turn while moving slowly so we sample new directions
    return 0.05, 1.0
