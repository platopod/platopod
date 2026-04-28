"""Band 1 reference solution — random walk.

Expected outcome: finds the source eventually (typically 150-250 ticks).
Will sometimes fail when the random walk strays into a corner.
Demonstrates: the loop, randomness, basic actions. ~5 lines of logic.
"""

from __future__ import annotations

import random


def decide(reading: dict, pose: dict) -> tuple[float, float]:
    if random.random() < 0.5:
        return 0.10, 0.0                       # forward
    return 0.0, random.uniform(-1.0, 1.0)       # turn
