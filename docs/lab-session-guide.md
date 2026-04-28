# Lab Session Guide — Cadet Exercise 1: Plume Search

Step-by-step procedure for running the plume-search exercise as a real lab session with the desk arena, an overhead AprilTag camera, an iPhone running iTAK, and 1–N cadets connecting from their own laptops.

This is the **classroom integration test plan** the user asked for. It also doubles as the demo runbook.

## Pre-lab — instructor's checklist

- [ ] Desk arena is set up, AprilTags 100/101/102/103 visible at the four corners
- [ ] One C922 webcam mounted overhead, focused, exposure-locked (Settings → Camera in `config/apriltag_settings.yaml`)
- [ ] At least one ESP32 robot powered on, paired to the lab WiFi, OLED showing its tag id
- [ ] iPhone (or other ATAK device) on the same WiFi as the instructor's machine, iTAK installed
- [ ] Instructor knows the iPhone's IP and the lab WiFi's network for cadet laptops to connect

## Stage 1 — instructor brings up the platform (5 min)

In the Docker container:

```bash
cd /ros2_ws
EX=/ros2_ws/config/exercises/cbrn-recon-patrol.yaml
IPHONE_IP=192.168.1.233       # ← substitute the iTAK device's IP

# Bring up the full stack — arena_model, registry, world_state, virtual_sim,
# los_python, engagement, cot_bridge — and start CoT streaming to iTAK.
GEO_LAT=-35.293935 GEO_LON=149.166421 SCALE=150 \
    /ros2_ws/tools/run_demo.sh "$EX" "$IPHONE_IP"
```

Wait ~10 s. iTAK should show the arena boundary and three plume contours over the gas source.

## Stage 2 — instructor runs the smoke test (1 min)

```bash
python3 /ros2_ws/tools/smoke_test_lab.py \
    --exercise "$EX" \
    --gateway-url http://localhost:8080
```

**Every stage should print PASS.** If any stage fails:

| Failing stage | Most likely cause | Fix |
|---|---|---|
| 1. ROS2 graph | Container ROS2 not sourced | `source /opt/ros/jazzy/setup.bash; source install/setup.bash` |
| 2. AprilTag | Camera not detected | `v4l2-ctl --list-devices` → update `camera_device` |
| 3. Arena boundary | arena_model_node not running | restart `run_demo.sh` |
| 4. Registry REST | Gateway port wrong / 8080 in use | `--gateway-url http://localhost:8090` |
| 5. World state | world_state_node down | restart `run_demo.sh` |
| 6. Plume field | YAML schema typo | check `gas_sources:` in the exercise YAML |
| 7. Sensor engine | No robot spawned yet | normal — re-run after stage 4 |
| 8. CoT bridge | not running, or wrong target_host | `--target_host=$IPHONE_IP` |
| 9. API gateway health | port collision (FreeTAKServer?) | `port:=8090` |
| 10. SDK round-trip | spawn rejected (boundary not yet set) | rerun |

For virtual-only labs (no camera, no robots), pass `--no-camera`:

```bash
python3 /ros2_ws/tools/smoke_test_lab.py --exercise "$EX" --no-camera
```

## Stage 3 — register a physical robot (2 min, optional)

Power on an ESP32 robot. It should send a `REG <tag_id> <radius_mm>` UDP packet to the platform on first boot. Verify:

```bash
curl http://localhost:8080/robots
# Expected: a robot with deployment="physical" and the matching tag_id
```

Drive it manually first to confirm the cmd_vel chain works:

```bash
ros2 topic pub --once /robot_<id>/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

The robot should creep forward for ~1 s. If it doesn't move:

- Check OLED for "REG OK"; if not registered, check WiFi + UDP port 9999 firewall
- Check `ros2 topic info /robot_<id>/cmd_vel` shows 1 publisher and 1 subscriber
- Check robot_bridge_node log for "M <linear> <angular>" send confirmations

## Stage 4 — cadets connect and run their solutions (per cadet)

Each cadet, on their own laptop:

```bash
git clone https://github.com/platopod/platopod.git
cd platopod/web/sdk/examples/cadet
pip install ../../sdk            # installs platopod SDK
pip install matplotlib numpy

# Develop at home (no platform): matplotlib animation + score
python3 plume_runtime.py --scenario civilians

# Lab session: connect to the running platform
python3 plume_runtime.py --live --robot <id> \
    --ws ws://<instructor_ip>:8080/api/control
```

Where `<id>` is either:
- the cadet's assigned virtual robot (instructor pre-spawns one per cadet), or
- the physical robot's tag id from the OLED display

## Stage 5 — instructor watches iTAK during cadet runs

iTAK shows simultaneously:
- **Arena boundary** as a green polygon
- **Plume contours** (yellow / orange / red threshold rings — rectangular bbox on iTAK iOS, see [tak-setup.md](tak-setup.md))
- **Each cadet's robot** as a coloured MIL-STD-2525B symbol (BLUE-N for friendly)
- **Trail** showing where each robot has been
- **Engagement events** if any cadet runs a fire scenario

The instructor's role:
- Watch trajectories cross the contamination zones — different algorithms produce visibly different paths
- Note which cadets respect ROE (don't enter red zones, slow near civilians)
- Optionally: trigger `inject_event` perturbations (wind shift, secondary release) to test robustness

## Stage 6 — collect scores

Each cadet's `plume_runtime.py` prints a score line at the end. They submit:
- Their `plume_search.py` (the file they edited)
- A screenshot of their final score

Bulk grading on the instructor's machine:

```bash
# For each cadet's submission file
for cadet_file in submissions/*.py; do
  cp "$cadet_file" /ros2_ws/web/sdk/examples/cadet/plume_search.py
  for scenario in calm windy civilians; do
    python3 /ros2_ws/web/sdk/examples/cadet/plume_runtime.py \
        --scenario "$scenario" --headless --score
  done
done > grades.csv
```

(A polished version of this script is on the to-do list — current loop works for ~30 cadets.)

## Lab-day failure modes

| Symptom | Diagnosis | Action |
|---|---|---|
| Cadet can spawn but `cmd_vel` doesn't move robot | `state_filter` rejecting (status not "active") | check `ros2 topic echo /robots/status --once` |
| Cadet gets `WebSocket connection timeout` | Wrong gateway port or firewall | check `--ws` URL matches running gateway |
| Robot moves backward when forward expected | Heading offset (AprilTag rotation) | re-zero with the calibration utility |
| iTAK suddenly empty | cot_bridge_node died | check stdout; usually a crashed dependency |
| All robots' positions stuck | `vision_node` lost the camera | unplug/replug camera; re-source ROS2 |
| Multiple cadets see each other's robots | (correct — that's the operational picture) | feature, not a bug |

## Pre-lab dry-run procedure

Run the full sequence the day before, with no cadets:

```bash
/ros2_ws/tools/run_demo.sh "$EX" "$IPHONE_IP"
sleep 15
python3 /ros2_ws/tools/smoke_test_lab.py --exercise "$EX"
# Confirm all stages PASS
# Spawn one virtual robot and run the band3 reference solution
cd /ros2_ws/web/sdk/examples/cadet
cp solutions/band3_roe_aware.py plume_search.py
python3 plume_runtime.py --live --robot 1 --ws ws://localhost:8080/api/control
# Confirm: real robot moves, iTAK shows trajectory, score shows DISTINCTION
/ros2_ws/tools/stop_demo.sh
```

If the dry-run passes, the lab session will too.

## Known limitations (heads-up for instructors)

- **iTAK on iOS** draws plume polygons as their bounding-box rectangles. Information value is preserved (colour-coded threshold zones at correct positions); fidelity is lost. Switch to ATAK Civ on Android for full polygon outlines.
- **Multiple cadets sharing one platform** all see each other's robots. Pedagogically this is a feature ("you're all in the same battlespace"), but if you want isolation you'll need one platform instance per cadet.
- **Real robot real-time scores may differ from simulation** because of wheel slip, AprilTag jitter, and sensor noise. ±10 % on tick count is normal; >25 % suggests a hardware issue.

## What's tested vs what needs lab time to validate

The smoke-test script (`tools/smoke_test_lab.py`) covers everything the cadet workflow depends on, **except** the AprilTag camera path and ESP32 UDP path which require the physical hardware. Stages 2 and 3 of the lab procedure (camera + register physical robot) are the only steps without automated validation; both are quick visual-confirm checks.
