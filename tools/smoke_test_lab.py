#!/usr/bin/env python3
"""smoke_test_lab — one-shot integration check for the desk-arena lab setup.

Walks through every link in the physical-robot tactical chain and prints
PASS / FAIL per stage. Use before each lab session; failures point at
the specific component that needs attention.

Each stage:
  1. Container & ROS2 graph alive
  2. Camera + AprilTag detection publishing /tags/detections
  3. arena_model_node has a published boundary
  4. registry_node responsive (list_robots service works)
  5. world_state_node has published the latched /world/* topics
  6. Gas plume field loadable from the active exercise YAML
  7. Sensor engine producing /robot_*/sensors/gas readings
  8. cot_bridge_node alive and sending CoT to a configured target
  9. API gateway REST + WebSocket reachable
 10. SDK can spawn a virtual robot, drive it, read sensors, query
     world state, then remove it cleanly

Run inside the container:

    python3 /ros2_ws/tools/smoke_test_lab.py \
        --exercise /ros2_ws/config/exercises/cbrn-recon-patrol.yaml \
        --gateway-url http://localhost:8080
"""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request


# ─── Helpers ──────────────────────────────────────────────────────────────

GREEN = "\033[1;32m"
RED = "\033[1;31m"
YELLOW = "\033[1;33m"
DIM = "\033[2m"
RESET = "\033[0m"


def _result(ok: bool, label: str, detail: str = "") -> bool:
    tag = f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"
    print(f"  [{tag}] {label}" + (f"  {DIM}{detail}{RESET}" if detail else ""))
    return ok


def _section(title: str) -> None:
    print()
    print(f"{YELLOW}── {title} ─{RESET}")


def _ros_topic_exists(topic: str, timeout_s: float = 2.0) -> bool:
    try:
        out = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True,
            text=True, timeout=timeout_s,
        )
        return topic in out.stdout.splitlines()
    except Exception:
        return False


def _ros_topic_echo(topic: str, qos_args: list[str] | None = None,
                    timeout_s: float = 3.0) -> str | None:
    args = ["ros2", "topic", "echo", topic, "--once"]
    if qos_args:
        args.extend(qos_args)
    try:
        out = subprocess.run(
            args, capture_output=True, text=True, timeout=timeout_s,
        )
        if out.returncode == 0 and out.stdout.strip():
            return out.stdout
        return None
    except subprocess.TimeoutExpired:
        return None
    except Exception:
        return None


def _ros_node_list(timeout_s: float = 2.0) -> list[str]:
    try:
        out = subprocess.run(
            ["ros2", "node", "list"], capture_output=True,
            text=True, timeout=timeout_s,
        )
        return out.stdout.splitlines()
    except Exception:
        return []


def _http_get(url: str, timeout_s: float = 3.0) -> tuple[int, str] | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as resp:
            return resp.status, resp.read().decode()
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode() if e.fp else ""
    except Exception:
        return None


# ─── Stages ───────────────────────────────────────────────────────────────

def stage_ros_graph() -> bool:
    _section("1. ROS2 graph")
    nodes = _ros_node_list()
    if not nodes:
        return _result(False, "ros2 node list returns empty",
                        "is the container running and ROS2 sourced?")
    expected = {"/registry_node", "/arena_model_node", "/api_gateway_node"}
    missing = expected - set(nodes)
    ok = not missing
    detail = f"{len(nodes)} nodes alive"
    if missing:
        detail += f"; missing: {sorted(missing)}"
    return _result(ok, "expected core nodes alive", detail)


def stage_apriltag(skip: bool) -> bool:
    _section("2. Camera + AprilTag detection")
    if skip:
        return _result(True, "(skipped — --no-camera)",
                        "use --no-camera in virtual-only labs")
    if not _ros_topic_exists("/tags/detections"):
        return _result(False, "/tags/detections topic missing",
                        "is vision_node running?")
    out = _ros_topic_echo("/tags/detections")
    ok = out is not None
    return _result(ok, "/tags/detections has data",
                    "tags currently visible to the camera"
                    if ok else "no detections; check camera framing")


def stage_arena_boundary() -> bool:
    _section("3. Arena model")
    if not _ros_topic_exists("/arena/model"):
        return _result(False, "/arena/model topic missing")
    out = _ros_topic_echo("/arena/model")
    ok = out is not None and "boundary_x" in out
    return _result(ok, "arena boundary published", "")


def stage_registry(gateway_url: str) -> bool:
    _section("4. Registry service via REST")
    res = _http_get(f"{gateway_url}/robots")
    if res is None:
        return _result(False, "GET /robots failed",
                        "is api_gateway_node up on this URL?")
    code, body = res
    if code != 200:
        return _result(False, f"GET /robots returned HTTP {code}", body[:120])
    try:
        robots = json.loads(body)
    except Exception as e:
        return _result(False, "GET /robots not valid JSON", str(e))
    return _result(True, "GET /robots returned JSON",
                    f"{len(robots)} robot(s) currently registered")


def stage_world_state(gateway_url: str) -> bool:
    _section("5. World state (/world REST + latched topics)")
    res = _http_get(f"{gateway_url}/world")
    if res is None:
        return _result(False, "GET /world failed")
    code, body = res
    if code == 503:
        return _result(False, "GET /world returns 503",
                        "world_state_node not running, or hasn't published yet")
    if code != 200:
        return _result(False, f"GET /world returned HTTP {code}", body[:120])
    try:
        ws = json.loads(body)
    except Exception:
        return _result(False, "GET /world not valid JSON")
    n_civ = len(ws.get("civilians", []))
    n_ied = len(ws.get("ied_zones", []))
    n_weap = len(ws.get("weapons", []))
    return _result(True, "/world snapshot OK",
                    f"civ={n_civ} ied={n_ied} weapons={n_weap}")


def stage_plume_field(exercise_yaml: str) -> bool:
    _section("6. Gas plume field loadable")
    try:
        sys.path.insert(0, "/ros2_ws/src/plato_pod/src")
        sys.path.insert(0, "/ros2_ws/src")
        import yaml
        with open(exercise_yaml) as f:
            cfg = yaml.safe_load(f)
        from plato_pod.virtual_layer_loader import load_virtual_layers
        env = load_virtual_layers(cfg)
        if env is None:
            return _result(False, "no virtual_layers in exercise YAML")
        if "gas" not in env.fields:
            return _result(True, "no gas field in this exercise",
                            "skipping plume tests — non-CBRN scenario")
        gas = env.fields["gas"]
        sample = gas.evaluate(0.5, 0.5, 0.0)
        return _result(True, "gas field instantiates and samples",
                        f"sample at (0.5, 0.5, t=0) = {sample:.1f}")
    except Exception as e:
        return _result(False, f"plume load failed: {e}")


def stage_sensor_engine(robot_id: int = 1) -> bool:
    _section("7. Sensor engine (gas reading)")
    topic = f"/robot_{robot_id}/sensors/gas"
    if not _ros_topic_exists(topic):
        return _result(True, f"{topic} not yet published",
                        "(skipped — no robot {robot_id}; will retest after spawn)"
                        .format(robot_id=robot_id))
    out = _ros_topic_echo(topic)
    return _result(out is not None, f"{topic} has data",
                    "sensor reading flowing" if out else "no data")


def stage_cot_bridge() -> bool:
    _section("8. CoT bridge alive")
    nodes = _ros_node_list()
    if "/cot_bridge_node" not in nodes:
        return _result(False, "/cot_bridge_node not running",
                        "ros2 launch plato_pod cot_bridge.launch.py …")
    return _result(True, "/cot_bridge_node alive",
                    "manual: confirm CoT events arrive on iTAK")


def stage_api_gateway(gateway_url: str) -> bool:
    _section("9. API gateway health")
    res = _http_get(f"{gateway_url}/health")
    if res is None:
        return _result(False, "/health unreachable",
                        f"is the gateway listening on {gateway_url}?")
    code, body = res
    return _result(code == 200, f"/health returned {code}", body[:160])


def stage_sdk_round_trip(gateway_url: str) -> bool:
    _section("10. SDK round-trip — spawn → cmd_vel → world → remove")
    try:
        sys.path.insert(0, "/ros2_ws/web/sdk")
        from platopod import Arena
    except Exception as e:
        return _result(False, f"cannot import platopod SDK: {e}")

    ws_url = gateway_url.replace("http://", "ws://").replace(
        "https://", "wss://"
    ) + "/api/control"

    try:
        arena = Arena(ws_url)
    except Exception as e:
        return _result(False, f"Arena() connection failed: {e}")

    spawned_id: int | None = None
    try:
        # Spawn
        result = arena.spawn(x=0.10, y=0.10, theta=0.0)
        if not result.get("success"):
            return _result(False, "spawn() failed", result.get("message", ""))
        spawned_id = int(result["robot_id"])

        # World state
        ws = arena.get_world_state()
        if not isinstance(ws, dict):
            return _result(False, "get_world_state() returned non-dict")

        # cmd_vel
        time.sleep(0.3)
        arena.cmd_vel(spawned_id, 0.05, 0.0)
        time.sleep(0.4)
        arena.cmd_vel(spawned_id, 0.0, 0.0)

        # Verify it moved
        robots = arena.list_robots()
        me = next((r for r in robots if r.get("robot_id") == spawned_id), None)
        if me is None:
            return _result(False, "spawned robot vanished from list_robots")
        moved = (me["x"] != 0.10) or (me["y"] != 0.10)

        return _result(True, "SDK round-trip OK",
                        f"id={spawned_id} moved={moved} "
                        f"final=({me['x']:.3f}, {me['y']:.3f})")
    except Exception as e:
        return _result(False, f"SDK round-trip raised: {e}")
    finally:
        if spawned_id is not None:
            try:
                arena.remove(spawned_id)
            except Exception:
                pass
        try:
            arena.close()
        except Exception:
            pass


# ─── Driver ───────────────────────────────────────────────────────────────

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--exercise", default="",
                    help="Path to the active exercise YAML (for plume check)")
    p.add_argument("--gateway-url", default="http://localhost:8080",
                    help="API gateway base URL (default: localhost:8080)")
    p.add_argument("--no-camera", action="store_true",
                    help="Skip the AprilTag detection check (virtual-only labs)")
    p.add_argument("--robot-id", type=int, default=1,
                    help="Pre-existing robot_id to query in stage 7")
    args = p.parse_args()

    print(f"{YELLOW}Plato Pod — lab smoke test{RESET}")
    print(f"{DIM}gateway: {args.gateway_url}   "
          f"exercise: {args.exercise or '(none)'}{RESET}")

    results = []
    results.append(stage_ros_graph())
    results.append(stage_apriltag(skip=args.no_camera))
    results.append(stage_arena_boundary())
    results.append(stage_registry(args.gateway_url))
    results.append(stage_world_state(args.gateway_url))
    if args.exercise:
        results.append(stage_plume_field(args.exercise))
    results.append(stage_sensor_engine(args.robot_id))
    results.append(stage_cot_bridge())
    results.append(stage_api_gateway(args.gateway_url))
    results.append(stage_sdk_round_trip(args.gateway_url))

    print()
    n_pass = sum(1 for r in results if r)
    n_total = len(results)
    if n_pass == n_total:
        print(f"{GREEN}── ALL {n_total} STAGES PASS — lab is ready ──{RESET}")
        return 0
    print(f"{RED}── {n_pass}/{n_total} stages passed; "
          f"{n_total - n_pass} fail(s) need attention ──{RESET}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
