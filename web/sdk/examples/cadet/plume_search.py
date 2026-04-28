"""Cadet Exercise 1 — Gas Plume Search.

A virtual chemical agent has been released somewhere on Dowsett Field.
Your robot has a gas sensor; your job is to write the algorithm that
makes the robot find the source.

You only need to edit the `decide()` function below. Everything else
(running the robot, drawing the plume, scoring you) is handled by the
runtime.

To test your code:

    python plume_runtime.py                     # local simulation
    python plume_runtime.py --scenario windy    # try a different wind
    python plume_runtime.py --score             # show scores
    python plume_runtime.py --live --robot 5    # connect to a real robot

Submission: a working plume_search.py with your decide() implementation.
Pass marks: completes the search in under 200 ticks (Band 1 or above).
Distinction: ≤80 ticks AND zero ROE violations (Band 3).

──────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

import math
import random


def decide(reading: dict, pose: dict) -> tuple[float, float]:
    """Return (linear_speed, angular_speed) for the next 100 ms.

    Called 10 times per second. The robot will move at the speeds you
    return until the next call.

    Args:
        reading: latest gas-sensor data, e.g.
            {
                "concentration": 230.5,           # current ppm-equivalent
                "history": [12, 45, 230, 230.5],  # last few readings
                "civilian_proximity": False,      # True if civilian within
                                                  # 15 m of robot
                "in_red_zone": False,             # True if concentration
                                                  # exceeds the lethal
                                                  # threshold
            }

        pose: robot state, e.g.
            {
                "x": 0.30,            # arena metres east of origin
                "y": 0.42,            # arena metres north of origin
                "heading": 1.57,      # radians, 0=east, π/2=north
                "team": "blue",
            }

    Returns:
        (linear, angular) tuple:
          linear  — m/s, positive moves the robot forward (max 0.2)
          angular — rad/s, positive turns the robot left (max 1.5)

    Examples:
          return  0.10,  0.0    # drive forward at 10 cm/s
          return  0.0,   0.5    # spin left at 0.5 rad/s
          return  0.05,  0.3    # drive forward and curve left
          return  0.0,   0.0    # stop
    """

    # ════════════════════════════════════════════════════════════════
    # BAND 1 — Random walk  (everyone should reach this)
    #
    # Make the robot wander. Half the time it goes forward, half the
    # time it picks a new random heading. This will eventually find
    # the source by chance — but slowly.
    #
    # Hint: use random.random() to get a number in [0, 1) and decide
    # which action to take.
    # ════════════════════════════════════════════════════════════════

    if random.random() < 0.5:
        return 0.10, 0.0                          # drive forward
    else:
        return 0.0, random.uniform(-1.0, 1.0)     # turn

    # ════════════════════════════════════════════════════════════════
    # BAND 2 — Gradient ascent  (delete the `return` above to enable)
    #
    # Compare the current concentration to the previous one. If it's
    # going UP, keep going. If it's going DOWN, you're moving away
    # from the source — turn around.
    #
    # The list `reading["history"]` has the last few readings; use the
    # difference between current and previous to decide.
    # ════════════════════════════════════════════════════════════════

    # history = reading["history"]
    # if len(history) >= 2:
    #     delta = history[-1] - history[-2]
    #     if delta > 0:
    #         return 0.10, 0.0    # closing on source — keep going
    #     else:
    #         return 0.05, 1.0    # turn while moving slowly
    # return 0.10, 0.0

    # ════════════════════════════════════════════════════════════════
    # BAND 3 — ROE-aware  (top marks)
    #
    # Same as Band 2, BUT also avoid the red lethal zone and don't
    # endanger civilians. If reading["in_red_zone"] is True, retreat
    # immediately (drive backward). If reading["civilian_proximity"]
    # is True, slow down — moving fast near civilians is a violation.
    #
    # A solid Band 3 solution will find the source in ≤80 ticks
    # without ever being flagged for a civilian or red-zone violation.
    # ════════════════════════════════════════════════════════════════

    # if reading["in_red_zone"]:
    #     return -0.10, 0.0       # back away from lethal exposure
    #
    # speed = 0.05 if reading["civilian_proximity"] else 0.10
    # history = reading["history"]
    # if len(history) >= 2 and history[-1] < history[-2]:
    #     return speed, 1.5       # turn back toward the gradient
    # return speed, 0.0


# Don't edit below this line — the runtime imports `decide` from here.
if __name__ == "__main__":
    print(
        "This is the function file. Run `python plume_runtime.py` to "
        "test your algorithm."
    )
