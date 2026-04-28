# Cadet Exercise 1 — Gas Plume Search

**Course:** Introduction to Programming (1st-year cadets)
**Time budget:** 2 × 1-hour tutorials + lab session
**Submission:** a working `plume_search.py` with your `decide()` function

## The mission

A virtual chemical agent has been released somewhere on Dowsett Field. Your robot is downwind of the source, somewhere in the plume tail. **Write the algorithm that drives the robot to the source.** You'll be scored on three things:

1. How fast you reach the source (ticks)
2. Whether you trample any civilians on the way (ROE violations)
3. Whether you walk into the lethal red zone (red-zone exposure)

You only edit **one function** — `decide()` in `plume_search.py`. Everything else (running the robot, drawing the plume, scoring you) is handled by the runtime.

## What the platform gives you

```python
def decide(reading: dict, pose: dict) -> tuple[float, float]:
    # reading = {
    #   "concentration": float,             # current sensor reading
    #   "history": [float, ...],            # last 6 readings
    #   "civilian_proximity": bool,         # True if a civilian is within 15m
    #   "in_red_zone": bool,                # True if exposure is lethal
    # }
    # pose = {"x": float, "y": float, "heading": float, "team": str}
    # Return (linear_m_per_s, angular_rad_per_s)
    ...
```

## Difficulty bands

The skeleton has three TODO sections. Implement each one in turn.

| Band | What you write | Expected result | Grade |
|---|---|---|---|
| **1** | Random walk: pick a random direction, move | Eventually finds the source. Sometimes ~250 ticks, sometimes 60. | PASS |
| **2** | Gradient ascent: compare current reading to previous; turn if it dropped | Finds source in 30-60 ticks on calm scenarios, 50-100 on windy | PASS |
| **3** | ROE-aware: same as Band 2, plus avoid civilians and lethal zones | 35-80 ticks AND zero civilian violations | **DISTINCTION** |

The grade is awarded by the runtime — see your scoresheet at the end.

## Running it

### Develop on your laptop (no platform / no robot needed)

```bash
cd web/sdk/examples/cadet/
python3 plume_runtime.py                    # default: calm wind scenario
python3 plume_runtime.py --scenario windy   # try the windier one
python3 plume_runtime.py --scenario civilians
python3 plume_runtime.py --headless --score # no animation, just print the score
```

A matplotlib window opens showing the plume contours (yellow / orange / red), your robot (blue dot), the source (red ✕), civilians (cyan ●), and your trail. Edit `decide()`, save, re-run. Tight feedback loop.

### Lab day — drive a real robot

After your code passes Band 2 in simulation, lab time runs each cadet's solution against the desk arena while the instructor watches the live picture on iTAK:

```bash
python3 plume_runtime.py --live --robot 5
```

Same algorithm; same `decide()`. The platform handles tag-based localisation, motor control, gas-sensor plumbing, and the operator picture.

## Scoring

The runtime prints something like:

```
══════════════ Score ══════════════
  Ticks to source     : 47
  Total ticks run     : 47
  Civilian violations : 0
  Red-zone ticks      : 0
  Final distance      : 0.142 arena-m
  Grade               : DISTINCTION — Band 3 (fast + clean)
═══════════════════════════════════
```

| Grade | Criterion |
|---|---|
| FAIL | Never reached the source |
| PASS — Band 1 | Reached source in any number of ticks |
| PASS — Band 2 | ≤150 ticks |
| **DISTINCTION — Band 3** | ≤80 ticks AND zero civilian violations |

## Hints (if you're stuck)

- **Stuck on Band 1?** Use `random.random()` to choose between "go forward" and "turn". Don't overthink it — you just need motion.
- **Stuck on Band 2?** Use the *delta* between `reading["history"][-1]` and `reading["history"][-2]`. Positive = closing on source.
- **Stuck on Band 3?** What does `reading["civilian_proximity"]` mean for your speed? What about `reading["in_red_zone"]`?
- **Always going in circles?** Your `linear` is 0. Increase it.
- **Robot stuck in a corner?** Reduce `linear` or add a turn.
- **Robot oscillates near source?** That's normal — Gaussian plumes have low concentration *exactly* at the source. The runtime still scores you "found it" if you get within 15 m.

## Files in this folder

```
plume_search.py              ← edit this
plume_runtime.py             ← run this; don't edit (read it if curious)
solutions/
   band1_random_walk.py      ← reference: ~5 lines
   band2_gradient_ascent.py  ← reference: ~15 lines
   band3_roe_aware.py        ← reference: ~25 lines
README.md                    ← this file
```

## For instructors

The runtime takes a `--save-video file.mp4` flag for capturing student attempts. The `--headless --score` mode is suitable for batch grading: feed each student's `plume_search.py` to a CI-style script that runs all three scenarios and tallies the results.

The simulated runtime uses the same `GaussianPlumeField` that the real platform uses (imported from `server/src/plato_pod/spatial_field.py`), so a solution that works in simulation should behave identically on real hardware — modulo wheel slip, sensor noise, and the AprilTag localisation noise.

For real-robot lab time:

```bash
# Bring up the platform (instructor's terminal)
/ros2_ws/tools/run_demo.sh /ros2_ws/config/exercises/gas-plume-search.yaml \
    <iTAK_iPhone_IP>

# Spawn one virtual robot per cadet, or use AprilTag-tracked desk robots.
# Then each cadet's machine connects:
python3 plume_runtime.py --live --robot <id> \
    --ws ws://<instructor_machine>:8080/api/control
```

The instructor's iTAK shows the plume contours and every cadet's robot moving simultaneously — pedagogically the strongest moment in the course.
