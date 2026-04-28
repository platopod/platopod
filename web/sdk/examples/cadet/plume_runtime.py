"""Runtime for Cadet Exercise 1 — runs your `decide()` function.

Two modes:

    Default (simulated): physics + plume model run locally on your
    laptop, no platform / no Docker / no robot needed. Live matplotlib
    animation shows your robot navigating the plume.

    Live (--live): connects to a running Plato Pod platform via the
    Python SDK and drives an actual robot (physical or virtual). Use
    in lab time when the platform is up.

In both modes your decide() function is called 10 times per second
and the resulting (linear, angular) command is applied to the robot.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

# Make the platform's own modules importable so we can reuse the
# Gaussian plume model — same physics the real platform uses.
HERE = Path(__file__).resolve().parent
SERVER_SRC = HERE.parents[3] / "server" / "src"
if SERVER_SRC.exists():
    sys.path.insert(0, str(SERVER_SRC))

try:
    from plato_pod.spatial_field import GaussianPlumeField
except ImportError:
    GaussianPlumeField = None    # type: ignore[assignment, misc]


# ─── Scenario presets ─────────────────────────────────────────────────────

@dataclass
class Scenario:
    name: str
    bbox: tuple[float, float, float, float]   # arena (xmin, ymin, xmax, ymax)
    source: tuple[float, float]
    wind_speed: float                          # m/s
    wind_direction: float                      # radians, 0 = blowing east
    release_rate: float
    diffusion: float
    robot_start: tuple[float, float, float]    # (x, y, heading)
    civilians: list[tuple[float, float]] = field(default_factory=list)
    red_zone_threshold: float = 5000.0        # triggers only inside the source halo;
                                               # ROE-aware Band 3 uses this to flag the
                                               # "stop here, you've found the lethal core"
                                               # boundary, NOT to abort the search early.
    civilian_proximity_m: float = 0.10        # arena metres
    source_radius_m: float = 0.15             # ~22 m at scale 150 — generous "found it" tolerance.
                                               # Gaussian plumes have a singularity at the source
                                               # (concentration = 0 exactly there) so naive
                                               # gradient ascent gets close but rarely lands on it.


SCENARIOS = {
    # Each scenario places the robot DOWNWIND of the source — i.e., in
    # the plume tail — so gradient ascent has a signal to follow. Real
    # CBRN reconnaissance often works this way (the cadet patrol enters
    # downwind, walks up the gradient toward the release point).
    "calm": Scenario(
        name="calm",
        bbox=(0.0, 0.0, 0.852, 0.723),
        source=(0.20, 0.40),
        wind_speed=2.0,
        wind_direction=0.0,                       # blowing east
        release_rate=500.0,
        diffusion=0.08,
        robot_start=(0.70, 0.40, math.pi),         # facing west toward source
    ),
    "windy": Scenario(
        name="windy",
        bbox=(0.0, 0.0, 0.852, 0.723),
        source=(0.20, 0.30),
        wind_speed=3.0,
        wind_direction=math.pi / 4,               # blowing NE
        release_rate=800.0,
        diffusion=0.10,                            # broader plume — wind disperses it widely
        robot_start=(0.70, 0.65, -3 * math.pi / 4),  # SW-bound, into the plume
    ),
    "civilians": Scenario(
        name="civilians",
        bbox=(0.0, 0.0, 0.852, 0.723),
        source=(0.25, 0.45),
        wind_speed=2.5,
        wind_direction=0.0,                       # blowing east
        release_rate=500.0,
        diffusion=0.08,
        robot_start=(0.75, 0.45, math.pi),         # west-bound up the plume
        civilians=[(0.45, 0.42), (0.50, 0.50), (0.55, 0.38)],
    ),
}


# ─── Simulated robot ──────────────────────────────────────────────────────

@dataclass
class SimRobot:
    x: float
    y: float
    heading: float
    radius: float = 0.025
    max_linear: float = 0.20
    max_angular: float = 1.5

    def step(self, linear: float, angular: float, dt: float,
             bbox: tuple[float, float, float, float]) -> None:
        """Forward one tick using differential-drive kinematics."""
        # Clamp commands to the platform's envelope
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))

        # Update heading first (mid-step kinematics for stability)
        self.heading = (self.heading + angular * dt + math.pi) % (2 * math.pi) - math.pi
        # Move along new heading
        self.x += linear * math.cos(self.heading) * dt
        self.y += linear * math.sin(self.heading) * dt
        # Stay inside the arena
        xmin, ymin, xmax, ymax = bbox
        self.x = max(xmin + self.radius, min(xmax - self.radius, self.x))
        self.y = max(ymin + self.radius, min(ymax - self.radius, self.y))


# ─── Score model ──────────────────────────────────────────────────────────

@dataclass
class Score:
    ticks_to_source: int | None = None
    civilian_violations: int = 0
    red_zone_ticks: int = 0
    total_ticks: int = 0
    final_distance_m: float = 0.0

    def grade(self) -> str:
        if self.ticks_to_source is None:
            return "FAIL — never reached the source"
        if self.ticks_to_source <= 80 and self.civilian_violations == 0:
            return "DISTINCTION — Band 3 (fast + clean)"
        if self.ticks_to_source <= 150:
            return "PASS — Band 2 (gradient ascent)"
        return "PASS — Band 1 (random walk)"

    def report(self) -> str:
        lines = [
            "══════════════ Score ══════════════",
            f"  Ticks to source     : {self.ticks_to_source if self.ticks_to_source is not None else 'NEVER'}",
            f"  Total ticks run     : {self.total_ticks}",
            f"  Civilian violations : {self.civilian_violations}",
            f"  Red-zone ticks      : {self.red_zone_ticks}",
            f"  Final distance      : {self.final_distance_m:.3f} arena-m",
            f"  Grade               : {self.grade()}",
            "═══════════════════════════════════",
        ]
        return "\n".join(lines)


# ─── Sensor model ─────────────────────────────────────────────────────────

class GasSensor:
    """Wraps the platform's GaussianPlumeField with a small history buffer."""

    def __init__(self, field, history_len: int = 6) -> None:
        self.field = field
        self.history: list[float] = []
        self.history_len = history_len

    def read(self, x: float, y: float, t: float) -> float:
        c = float(self.field.evaluate(x, y, t))
        self.history.append(c)
        if len(self.history) > self.history_len:
            self.history.pop(0)
        return c


# ─── Simulated runtime ────────────────────────────────────────────────────

def run_simulated(decide_fn, scenario: Scenario, max_ticks: int = 300,
                   show: bool = True, save_video: str | None = None) -> Score:
    """Run the cadet's decide() against a Python-only simulation.

    Animates the plume contours and the robot trajectory. Returns a Score.
    """
    if GaussianPlumeField is None:
        print("ERROR: cannot import GaussianPlumeField. Run from the "
              "platopod repo so server/src/ is on sys.path.")
        sys.exit(2)

    field = GaussianPlumeField(
        source_x=scenario.source[0], source_y=scenario.source[1],
        release_rate=scenario.release_rate,
        wind_speed=scenario.wind_speed,
        wind_direction=scenario.wind_direction,
        diffusion_coeff=scenario.diffusion,
    )
    sensor = GasSensor(field)
    robot = SimRobot(
        x=scenario.robot_start[0],
        y=scenario.robot_start[1],
        heading=scenario.robot_start[2],
    )
    score = Score()
    trail: list[tuple[float, float]] = []

    dt = 0.1   # 10 Hz

    # Optional matplotlib animation
    if show:
        try:
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
            import numpy as np
        except ImportError:
            print("matplotlib not installed; running headless.")
            show = False

    if show:
        # Plume sample grid
        nx, ny = 80, 80
        xs = np.linspace(scenario.bbox[0], scenario.bbox[2], nx)
        ys = np.linspace(scenario.bbox[1], scenario.bbox[3], ny)
        Z = np.array([[field.evaluate(x, y, 0.0) for x in xs] for y in ys])

        fig, ax = plt.subplots(figsize=(8, 7))
        ax.set_xlim(scenario.bbox[0], scenario.bbox[2])
        ax.set_ylim(scenario.bbox[1], scenario.bbox[3])
        ax.set_aspect("equal")
        ax.set_title(f"Scenario: {scenario.name}  —  ✕ = source, ● = robot")

        # Filled contours for the plume
        levels = [10, 50, 200, 500, 1500, 5000]
        colours = ["#2ecc40", "#ffdc00", "#ff851b", "#ff4136",
                    "#85144b", "#001f3f"]
        ax.contourf(xs, ys, Z, levels=levels, colors=colours, alpha=0.55,
                     extend="max")

        # Source marker
        ax.plot(*scenario.source, marker="X", markersize=14,
                markerfacecolor="red", markeredgecolor="black",
                linestyle="None", label="source")
        # Civilians
        for cx, cy in scenario.civilians:
            ax.plot(cx, cy, marker="o", markersize=12,
                    markerfacecolor="#39CCCC",
                    markeredgecolor="black", linestyle="None")

        (trail_line,) = ax.plot([], [], color="black", linewidth=1, alpha=0.6)
        (robot_dot,) = ax.plot([], [], marker="o", markersize=10,
                                markerfacecolor="#0074D9",
                                markeredgecolor="black", linestyle="None")
        info = ax.text(0.02, 0.97, "", transform=ax.transAxes,
                        fontsize=10, va="top",
                        bbox=dict(boxstyle="round", facecolor="white",
                                   edgecolor="grey", alpha=0.85))

        def update(frame: int):
            tick = frame
            # Step
            t = tick * dt
            concentration = sensor.read(robot.x, robot.y, t)

            # Civilian proximity check
            civ_close = any(
                math.hypot(robot.x - cx, robot.y - cy)
                < scenario.civilian_proximity_m
                for cx, cy in scenario.civilians
            )
            in_red = concentration >= scenario.red_zone_threshold

            reading = {
                "concentration": concentration,
                "history": list(sensor.history),
                "civilian_proximity": civ_close,
                "in_red_zone": in_red,
            }
            pose = {
                "x": robot.x, "y": robot.y, "heading": robot.heading,
                "team": "blue",
            }

            try:
                lin, ang = decide_fn(reading, pose)
            except Exception as e:
                info.set_text(f"decide() raised: {e}")
                return trail_line, robot_dot, info

            # Penalise speed near civilians (>0.05 m/s = violation)
            if civ_close and abs(lin) > 0.05:
                score.civilian_violations += 1
            if in_red:
                score.red_zone_ticks += 1

            robot.step(lin, ang, dt, scenario.bbox)
            trail.append((robot.x, robot.y))
            score.total_ticks = tick + 1

            # Update plot
            xs_t = [p[0] for p in trail]
            ys_t = [p[1] for p in trail]
            trail_line.set_data(xs_t, ys_t)
            robot_dot.set_data([robot.x], [robot.y])

            distance_to_source = math.hypot(
                robot.x - scenario.source[0],
                robot.y - scenario.source[1],
            )
            score.final_distance_m = distance_to_source

            info.set_text(
                f"tick {tick}\n"
                f"conc {concentration:.0f}\n"
                f"dist to src {distance_to_source:.3f} m"
                + ("\n⚠ civilian close" if civ_close else "")
                + ("\n☣ red zone"        if in_red    else "")
            )

            if (score.ticks_to_source is None
                    and distance_to_source < scenario.source_radius_m):
                score.ticks_to_source = tick + 1
                info.set_text(info.get_text() + "\n✓ FOUND SOURCE")
                ani.event_source.stop()

            return trail_line, robot_dot, info

        ani = FuncAnimation(  # noqa: F841
            fig, update, frames=max_ticks,
            interval=dt * 1000, blit=False, repeat=False,
        )

        if save_video:
            ani.save(save_video, fps=10)
        else:
            plt.show()
    else:
        # Headless: same loop without rendering
        for tick in range(max_ticks):
            t = tick * dt
            concentration = sensor.read(robot.x, robot.y, t)
            civ_close = any(
                math.hypot(robot.x - cx, robot.y - cy)
                < scenario.civilian_proximity_m
                for cx, cy in scenario.civilians
            )
            in_red = concentration >= scenario.red_zone_threshold

            reading = {
                "concentration": concentration,
                "history": list(sensor.history),
                "civilian_proximity": civ_close,
                "in_red_zone": in_red,
            }
            pose = {"x": robot.x, "y": robot.y,
                     "heading": robot.heading, "team": "blue"}

            try:
                lin, ang = decide_fn(reading, pose)
            except Exception as e:
                print(f"decide() raised at tick {tick}: {e}")
                break

            if civ_close and abs(lin) > 0.05:
                score.civilian_violations += 1
            if in_red:
                score.red_zone_ticks += 1

            robot.step(lin, ang, dt, scenario.bbox)
            score.total_ticks = tick + 1
            distance = math.hypot(robot.x - scenario.source[0],
                                    robot.y - scenario.source[1])
            score.final_distance_m = distance
            if (score.ticks_to_source is None
                    and distance < scenario.source_radius_m):
                score.ticks_to_source = tick + 1
                break

    return score


# ─── Live runtime (real platform) ─────────────────────────────────────────

def run_live(decide_fn, robot_id: int, scenario_name: str,
             max_ticks: int = 600,
             ws_url: str = "ws://localhost:8080/api/control") -> Score:
    """Drive a real (or virtual) robot on a running Plato Pod platform.

    Requires `pip install -e web/sdk` and a running stack. Use this in
    lab time once you've validated your decide() in simulation.
    """
    try:
        from platopod import Arena    # web/sdk/platopod/arena.py
    except ImportError:
        print("Cannot import platopod SDK. Run `pip install -e web/sdk` "
              "from the repo root.")
        sys.exit(2)

    scenario = SCENARIOS.get(scenario_name, SCENARIOS["calm"])
    score = Score()

    arena = Arena(ws_url)
    arena.subscribe_sensors(robot_id=robot_id, sensors=["gas", "gps"],
                              rate_hz=10)

    history: list[float] = []
    dt = 0.1
    print(f"Live mode: driving robot {robot_id} on {ws_url}")

    # Cache civilians once at start. The platform publishes them via
    # latched topics so they don't change unless an inject_event fires.
    civilians = arena.get_civilians()
    if civilians:
        print(f"Live world state: {len(civilians)} civilian(s) loaded")

    try:
        for tick in range(max_ticks):
            gas = arena.get_sensor(robot_id, "gas") or {}
            concentration = float(gas.get("concentration", 0.0))
            history.append(concentration)
            if len(history) > 6:
                history.pop(0)

            # Pose from list_robots (best-effort)
            robots = arena.list_robots()
            me = next((r for r in robots if r.get("robot_id") == robot_id), {})
            pose = {
                "x": float(me.get("x", 0.0)),
                "y": float(me.get("y", 0.0)),
                "heading": float(me.get("theta", 0.0)),
                "team": me.get("team") or "blue",
            }

            # Compute civilian-proximity flag from the cached world state
            # so the live `reading` shape matches the simulated runtime's.
            civ_close = any(
                math.hypot(pose["x"] - float(c["x"]),
                            pose["y"] - float(c["y"]))
                < scenario.civilian_proximity_m
                for c in civilians
            )

            in_red = concentration >= scenario.red_zone_threshold
            reading = {
                "concentration": concentration,
                "history": list(history),
                "civilian_proximity": civ_close,
                "in_red_zone": in_red,
            }

            try:
                lin, ang = decide_fn(reading, pose)
            except Exception as e:
                print(f"decide() raised at tick {tick}: {e}")
                break

            arena.cmd_vel(robot_id, lin, ang)
            score.total_ticks = tick + 1
            arena.sleep(dt)
    finally:
        arena.cmd_vel(robot_id, 0.0, 0.0)
        arena.close()

    return score


# ─── CLI ──────────────────────────────────────────────────────────────────

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scenario", default="calm",
                    choices=list(SCENARIOS.keys()),
                    help="which preset scenario to run")
    p.add_argument("--ticks", type=int, default=300,
                    help="max ticks (default 300 = 30 s)")
    p.add_argument("--headless", action="store_true",
                    help="no animation; useful for scoring")
    p.add_argument("--score", action="store_true",
                    help="print the score even when running with animation")
    p.add_argument("--save-video", type=str, default=None,
                    help="save animation as MP4 (matplotlib FFMpegWriter)")
    p.add_argument("--live", action="store_true",
                    help="connect to real platform via WebSocket SDK")
    p.add_argument("--robot", type=int, default=1,
                    help="robot_id to drive in --live mode")
    p.add_argument("--ws", default="ws://localhost:8080/api/control",
                    help="WebSocket URL of the platform gateway")
    args = p.parse_args()

    # Import the cadet's decide function from the file next to this one
    from plume_search import decide

    if args.live:
        score = run_live(decide, args.robot, args.scenario,
                          max_ticks=args.ticks, ws_url=args.ws)
    else:
        scenario = SCENARIOS[args.scenario]
        score = run_simulated(
            decide, scenario,
            max_ticks=args.ticks,
            show=not args.headless,
            save_video=args.save_video,
        )

    if args.headless or args.score or args.live:
        print()
        print(score.report())

    return 0


if __name__ == "__main__":
    sys.exit(main())
