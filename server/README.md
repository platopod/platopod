# Server

ROS2 Jazzy server nodes for the Plato Pod platform.

## Packages

### plato_pod (ament_python)

Main Python package containing all ROS2 nodes.

**Implemented nodes:**
- **vision_node** — Overhead camera, AprilTag detection via `AprilTagProvider`, MJPEG streaming
- **arena_model_node** — YAML-first arena boundary, obstacles, zones (virtual-first, camera optional)
- **registry_node** — Localization-agnostic robot registry via ROS2 services
- **robot_bridge_node** — UDP communication with physical ESP32 robots
- **api_gateway_node** — WebSocket + REST API, command pipeline with pluggable `KinematicsModel`
- **virtual_sim_node** — Pluggable kinematics for virtual robots (50 Hz, lightweight mode)
- **sensor_engine_node** — Plugin-based sensors (GPS, gas/MOX, lidar/sonar/FoF). Multiplexes between Python plugins and Gazebo sensor bridge
- **cot_bridge_node** — CoT/ATAK integration (robot positions → ATAK, waypoints ← ATAK)
- **gazebo_bridge_node** — Gazebo 3D simulation (SDF world generation, pose/cmd_vel bridging)

**Planned nodes:**
- **exercise_manager_node** — Exercise lifecycle, teams, scoring

### plato_pod_msgs (ament_cmake)

ROS2 message definitions used by all nodes.

**Messages:**
- `TagPose.msg` — Single tag pose (tag_id, x, y, theta, confidence, timestamp)
- `TagDetections.msg` — All detections from one frame (tags, boundary_tags, robot_tags, origin_visible, frame_id, fps)
- `HomographyMatrix.msg` — 3x3 pixel-to-arena homography (matrix[9], valid, timestamp)
- `Obstacle.msg` — Static obstacle polygon (label, vertices_x[], vertices_y[], original_type)
- `Zone.msg` — Scoring zone polygon (name, team, vertices_x[], vertices_y[], hold_time_seconds, original_type)
- `ArenaModel.msg` — Full arena model (boundary, obstacles[], zones[], origin_tag, timestamp, boundary_valid)

## Build

```bash
# Inside Docker container (see docker/README.md)
cd /ros2_ws
colcon build
source install/setup.bash
```

Both `plato_pod` and `plato_pod_msgs` are built by colcon automatically.

## Vision Node

### Launch

```bash
# Find your camera device first
v4l2-ctl --list-devices
# Example output:
#   C922 Pro Stream Webcam (usb-...):  /dev/video4, /dev/video5
#   Integrated_Webcam_HD (usb-...):    /dev/video0, /dev/video1, ...
# Use the first /dev/videoN number for your external camera (e.g. 4)

# Launch with C922 webcam, autofocus off, no zoom
ros2 run plato_pod vision_node --ros-args \
  -p camera_device:=4 \
  -p camera_autofocus:=false \
  -p camera_focus:=0 \
  -p camera_zoom:=100 \
  -p debug_overlay:=true

# Default parameters (uses /dev/video0)
ros2 run plato_pod vision_node

# Via launch file
ros2 launch plato_pod vision.launch.py
ros2 launch plato_pod vision.launch.py camera_device:=4 debug_overlay:=true
```

### Virtual-only mode

If no USB camera is detected, the node starts in virtual-only mode automatically. It publishes empty detection messages and serves the REST endpoints, allowing other nodes and the dashboard to run without physical hardware.

### ROS2 topics

| Topic | Type | Description |
|-------|------|-------------|
| `/tags/detections` | `plato_pod_msgs/TagDetections` | All detected tags with arena poses per frame |
| `/arena/homography` | `plato_pod_msgs/HomographyMatrix` | Pixel-to-arena homography from boundary corners |
| `/camera/info` | `sensor_msgs/CameraInfo` | Camera intrinsic parameters |

Verify topics are publishing:

```bash
ros2 topic list
ros2 topic echo /tags/detections
ros2 interface show plato_pod_msgs/msg/TagPose
ros2 interface show plato_pod_msgs/msg/TagDetections
```

### HTTP endpoints (MJPEG + REST)

The vision node runs an HTTP server (default port 8081) alongside the ROS2 node.

| Endpoint | Description |
|----------|-------------|
| `GET /camera/stream` | Raw MJPEG camera feed |
| `GET /camera/stream/debug` | MJPEG feed with tag outlines, IDs, axes, boundary |
| `GET /arena/homography` | Current homography matrix as JSON |
| `GET /camera/info` | Camera intrinsics as JSON |
| `GET /health` | Health check (camera status, OpenCV availability) |

Test endpoints:

```bash
# Health check
curl http://localhost:8081/health

# Camera info
curl http://localhost:8081/camera/info

# Homography (503 until boundary tags are detected)
curl http://localhost:8081/arena/homography

# MJPEG stream — open in a browser
# http://localhost:8081/camera/stream
# http://localhost:8081/camera/stream/debug
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_device` | int | 0 | V4L2 device index (or string path to video file) |
| `camera_width` | int | 1280 | Capture width in pixels |
| `camera_height` | int | 720 | Capture height in pixels |
| `camera_fps` | int | 30 | Capture frame rate |
| `camera_autofocus` | bool | false | Enable autofocus (disable for calibration consistency) |
| `camera_focus` | int | -1 | Manual focus (0=infinity, 250=macro, -1=camera default). Only used when autofocus is off |
| `camera_zoom` | int | 100 | Zoom level (100=1x, 500=5x). Use `v4l2-ctl -d /dev/videoN --list-ctrls` for range |
| `camera_pixel_format` | string | mjpg | Camera pixel format: `mjpg` (hardware compressed) or `yuyv` (raw) |
| `calibration_file` | string | (share dir) | Camera intrinsics YAML file |
| `apriltag_config` | string | (share dir) | AprilTag settings YAML file |
| `debug_overlay` | bool | false | Enable debug annotations on the debug MJPEG stream |
| `detection_enabled` | bool | true | Enable AprilTag detection (disable for stream-only latency testing) |
| `mjpeg_quality` | int | 10 | JPEG encoding quality (0-100, lower = faster streaming, more artifacts) |
| `mjpeg_port` | int | 8081 | HTTP server port for MJPEG/REST |

## Arena Model Node

### Launch

```bash
# Basic — boundary from detected tags, no obstacles/zones
ros2 run plato_pod arena_model_node

# With exercise config (obstacles + zones)
ros2 run plato_pod arena_model_node --ros-args \
  -p exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml

# Via launch file
ros2 launch plato_pod arena.launch.py
ros2 launch plato_pod arena.launch.py \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml
```

The node subscribes to `/tags/detections` from the vision node, extracts boundary tag poses (IDs 100-103), computes the convex hull, and publishes the arena model.

### ROS2 topics

| Topic | Type | Description |
|-------|------|-------------|
| `/arena/model` | `plato_pod_msgs/ArenaModel` | Full arena model (boundary + obstacles + zones) |
| `/arena/boundary` | `geometry_msgs/PolygonStamped` | Boundary polygon only (for RViz2) |

Verify:

```bash
ros2 topic echo /arena/model --once
ros2 topic echo /arena/boundary --once
ros2 interface show plato_pod_msgs/msg/ArenaModel
```

### HTTP endpoints

The arena model node runs an HTTP server (default port 8082).

| Endpoint | Description |
|----------|-------------|
| `GET /arena/model` | Full arena model as JSON |
| `GET /health` | Health check (model availability, boundary validity) |

```bash
curl -s http://localhost:8082/arena/model | python3 -m json.tool
curl -s http://localhost:8082/health | python3 -m json.tool
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `exercise_file` | string | "" | Path to exercise YAML (empty = free-play, no obstacles/zones) |
| `boundary_staleness_timeout` | float | 10.0 | Seconds before warning about unseen boundary tag |
| `rest_port` | int | 8082 | HTTP server port for REST API |
| `circle_vertices` | int | 16 | Vertices for circle-to-polygon approximation |
| `origin_tag` | int | 101 | Origin tag ID |

### Boundary staleness

If a boundary corner tag (100-103) is not visible for longer than `boundary_staleness_timeout` seconds, the node logs a warning but continues using the last known position. The boundary updates dynamically when tags are moved.

## Registry Node + Robot Bridge

### Launch

```bash
# Launch both registry and bridge together
ros2 launch plato_pod registry.launch.py

# Or individually
ros2 run plato_pod registry_node &
ros2 run plato_pod robot_bridge_node
```

The registry node manages the robot registry (physical + virtual). The bridge node handles UDP communication with physical ESP32 robots.

### ROS2 services (registry_node)

| Service | Type | Description |
|---------|------|-------------|
| `~/register_physical` | `RegisterPhysical` | Register a physical robot (called by bridge) |
| `~/spawn_virtual` | `SpawnVirtual` | Spawn a virtual robot with boundary/collision validation |
| `~/remove_robot` | `RemoveRobot` | Remove a virtual robot |
| `~/reset_robot` | `ResetRobot` | Force-reset a physical robot (mark inactive) |
| `~/list_robots` | `ListRobots` | Query all registered robots |

Test services:

```bash
# List robots
ros2 service call /registry_node/list_robots plato_pod_msgs/srv/ListRobots

# Spawn a virtual robot (requires arena boundary to be established)
ros2 service call /registry_node/spawn_virtual plato_pod_msgs/srv/SpawnVirtual \
  "{x: 0.3, y: 0.3, theta: 0.0, radius: 0.028}"
```

### ROS2 topics

| Topic | Type | Description |
|-------|------|-------------|
| `/robots/status` | `plato_pod_msgs/RobotStatusList` | All robots with poses (10 Hz) |

### Robot bridge (physical robots)

The bridge listens on UDP port 9999 for ESP32 robot registration messages (`REG <tag_id> <radius_mm>`). It translates ROS2 `/robot_{id}/cmd_vel` topics to UDP motor commands and monitors robot connectivity via heartbeat pings.

### Parameters

**registry_node:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `default_radius` | float | 0.028 | Default robot radius in metres |
| `publish_rate_hz` | float | 10.0 | Status publish rate |

**robot_bridge_node:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `udp_port` | int | 9999 | UDP server port |
| `heartbeat_interval_sec` | float | 2.0 | Heartbeat ping interval |
| `heartbeat_timeout_sec` | float | 6.0 | Timeout before marking robot inactive |
| `max_linear_speed` | float | 0.3 | Linear velocity clamp (m/s) |
| `max_angular_speed` | float | 2.0 | Angular velocity clamp (rad/s) |

## API Gateway

### Launch

```bash
ros2 run plato_pod api_gateway_node
ros2 launch plato_pod gateway.launch.py
```

The API gateway is the single client-facing entry point. It provides REST endpoints for robot management and a WebSocket endpoint for real-time control.

### REST endpoints (port 8080)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/robots` | GET | List all registered robots |
| `/robots/spawn` | POST | Spawn a virtual robot `{"x", "y", "theta", "radius"}` |
| `/robots/{id}` | DELETE | Remove a virtual robot |
| `/robots/{id}/reset` | POST | Force-reset a physical robot |
| `/arena/model` | GET | Current arena model |
| `/health` | GET | Health check |

### WebSocket endpoint

```
WS /api/control
```

Single connection per client, multi-robot control. Messages are JSON.

**Client sends:** `cmd_vel` (velocity command), `subscribe` / `unsubscribe` (observation)

**Server sends:** `pose` (10Hz per subscribed robot), `event` (boundary_contact, collision_contact, command_clamped, watchdog_timeout, control_released), `error` (robot_busy, not_found, invalid_command, robot_inactive), `subscribe_ack`

### Command pipeline

Each cmd_vel passes through: speed limiter (clamp to max) -> boundary filter (predict + clamp at walls/obstacles) -> collision filter (predict + clamp at other robots). Events are generated at each stage.

### Control policy

First client to send cmd_vel acquires control. Velocity watchdog (500ms) zeros velocity but keeps control. Control timeout (30s) releases the robot for other clients.

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | int | 8080 | HTTP/WebSocket server port |
| `max_linear_speed` | float | 0.2 | Linear velocity clamp (m/s) |
| `max_angular_speed` | float | 2.0 | Angular velocity clamp (rad/s) |
| `watchdog_timeout_ms` | int | 500 | Velocity watchdog timeout |
| `control_timeout_sec` | int | 30 | Control release timeout |
| `pose_rate_hz` | float | 10.0 | Pose broadcast rate |
| `lookahead_dt` | float | 0.1 | Prediction lookahead for filtering |

### Python SDK

The SDK is at `web/sdk/`. Install and use:

```bash
cd web/sdk && pip install -e .
```

```python
from platopod import Arena

arena = Arena("ws://192.168.4.1:8080/api/control")
robots = arena.list_robots()
arena.cmd_vel(robot_id=1, linear_x=0.1, angular_z=0.0)
arena.sleep(1.0)
arena.cmd_vel(robot_id=1, linear_x=0.0, angular_z=0.0)
arena.close()
```

### Keyboard demo

```bash
cd web/sdk
python3 examples/keyboard_demo.py --robot-id 1
```

## Virtual Simulation Node

### Launch

```bash
ros2 run plato_pod virtual_sim_node
ros2 launch plato_pod virtual_sim.launch.py
```

Computes differential-drive kinematics for virtual robots at 50 Hz. Subscribes to `/robot_{id}/cmd_vel`, publishes `/robot_{id}/pose` (Pose2D). Automatically tracks virtual robot lifecycle from `/robots/status`.

### Testing with command-line

```bash
# Spawn a virtual robot via the API gateway
curl -X POST http://localhost:8080/robots/spawn -H 'Content-Type: application/json' \
  -d '{"x": 0.5, "y": 0.5}'

# Send velocity command
ros2 topic pub /robot_1/cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.5}}" &

# Watch the pose update at 50 Hz
ros2 topic echo /robot_1/pose
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tick_rate_hz` | int | 50 | Simulation update frequency |
| `max_linear_speed` | float | 0.2 | Maximum linear speed (m/s) |
| `max_angular_speed` | float | 2.0 | Maximum angular speed (rad/s) |
| `acceleration_limit` | float | 0.0 | Acceleration limit (m/s², 0 = instant) |
| `velocity_noise_stddev` | float | 0.0 | Gaussian noise on velocity |
| `pose_drift_stddev` | float | 0.0 | Gaussian noise on pose (odometry drift) |

## Web Dashboard

The dashboard is at `web/static/index.html`, served by the API gateway at:

```
http://<server-ip>:8080/
```

### Two rendering modes

**Map mode (default):** 2D top-down view of the arena. No camera needed. Arena boundary, obstacles, zones, and robots drawn on a dark canvas with a 10cm grid. Works immediately when the arena model is loaded from the exercise YAML.

**Camera mode (toggle):** Live MJPEG camera feed as background with pixel-based overlay for detected tags. Toggle via the "Camera overlay" checkbox. Requires the vision node running with a camera.

### What it shows

- **Arena boundary** — green polygon from the exercise YAML or detected boundary tags
- **Static obstacles** — grey filled polygons with labels
- **Scoring zones** — semi-transparent team-colored regions with labels
- **Robots** — solid circles (physical) or dashed circles (virtual) with direction arrows and ID labels
- **Origin marker** — green dot at (0,0) in map mode
- **10cm grid** — subtle background grid for scale reference (map mode)
- **HUD** (top-right) — robot count, WebSocket status, arena dimensions, current mode
- **Toggle controls** (top-left) — camera overlay, boundary, obstacles, zones, robots, labels
- **Click-to-inspect** — click a robot to see details

### How it works

The dashboard fetches data from these endpoints (all on port 8080):

| Endpoint | Purpose | Refresh |
|----------|---------|---------|
| `GET /robots` | Robot list with poses | Every 3s |
| `GET /arena/model` | Boundary polygon + obstacles + zones | Every 3s |
| `GET /arena/homography` | 3x3 pixel↔arena transform matrix | Every 3s |
| `WS /api/control` | Real-time pose updates (subscribe to all robots) | Live |

The **homography matrix** maps between camera pixel coordinates and arena coordinates (metres). It's computed from all detected AprilTags. The dashboard inverts this matrix to project arena positions onto the camera feed for AR overlay alignment.

The camera feed is loaded directly from the vision node (`http://<host>:8081/camera/stream`) via an `<img>` tag (no CORS issues for images). Other data goes through the gateway proxy on port 8080.

### AR overlay accuracy

The overlay alignment depends on **camera calibration**. Without calibration, the vision node estimates camera intrinsics from the resolution and an assumed FOV, which can cause the overlay to be offset by several centimetres. After calibration, alignment is accurate to within a few pixels.

### Technology

Single HTML file with inline CSS and vanilla JavaScript. No build tools, no frameworks. Uses HTML5 Canvas 2D API for overlay rendering. Works in Chrome, Firefox, Safari, and Edge.

## Camera Calibration

One-time procedure per camera to compute intrinsic parameters. **Required for accurate AR overlay alignment and pose estimation.**

### Why calibrate

Without calibration, the vision node uses estimated defaults (100° FOV pinhole model). This causes:
- AR overlay misalignment in the dashboard
- Pose estimation errors of several centimetres at frame edges
- Inaccurate homography matrix

After calibration:
- AR overlay aligns precisely with physical tags
- Pose accuracy improves to ±5mm
- Homography correctly maps between pixels and metres

### Procedure

1. Print a **checkerboard pattern** (9×6 inner corners, 25mm square size) on A4 paper
2. Run the calibration tool inside Docker:

```bash
ros2 run plato_pod camera_calibrate \
  --squares-x 9 --squares-y 6 --square-size 0.025 \
  --frames 20 --camera 4 \
  --output config/camera_calibration.yaml
```

3. Hold the checkerboard in front of the camera at various angles and distances
4. The tool captures frames automatically (2-second cooldown between captures)
5. After 20 frames, it computes and saves the calibration
6. Press `q` to abort early

### Output

The calibration file is saved to `server/config/camera_calibration.yaml`:

```yaml
camera_calibration:
  image_width: 1280
  image_height: 720
  camera_matrix:
    - [fx, 0, cx]
    - [0, fy, cy]
    - [0, 0, 1]
  distortion_coefficients: [k1, k2, p1, p2, k5]
  calibration_date: "2026-04-12"
  reprojection_error: 0.25
```

The vision node loads this file automatically on startup. Lower reprojection error = better calibration (aim for < 0.5 pixels).

## Testing

### Unit tests (no Docker, no ROS2, no camera needed)

```bash
cd server

# Run all unit tests
PYTHONPATH=src:$PYTHONPATH python3 -m pytest tests/ -v

# Run a specific test file
PYTHONPATH=src:$PYTHONPATH python3 -m pytest tests/unit/test_config.py -v

# Run with coverage
PYTHONPATH=src:$PYTHONPATH python3 -m pytest tests/ --cov=src --cov-report=term-missing
```

All unit tests run without ROS2, without a camera, and without OpenCV (overlay drawing tests that need OpenCV are skipped automatically).

**Test files:**

| File | Tests | What it covers |
|------|-------|----------------|
| `test_config.py` | 19 | YAML loading, tag classification, tag sizes, default calibration |
| `test_camera.py` | 7 | VirtualCamera frame source |
| `test_detection.py` | 18 | Tag classification, tag size map, confidence scoring, grayscale conversion |
| `test_pose_estimation.py` | 22 | Transforms, inverse transforms, 2D pose extraction, camera-to-arena |
| `test_homography.py` | 10 | Homography computation (DLT), point mapping, batch transforms |
| `test_debug_overlay.py` | 9 | Overlay functions, no crashes on edge cases |
| `test_mjpeg_server.py` | 12 | FrameStore thread safety, REST endpoints |
| `test_calibration.py` | 10 | Object point generation, calibration state, YAML round-trip |
| `test_geometry.py` | 34 | Convex hull, point-in-polygon, polygon containment, shape conversions |
| `test_arena_model.py` | 31 | Obstacle/zone parsing, exercise YAML loading, validation, serialisation |
| `test_udp_protocol.py` | 31 | UDP message parsing, formatting, velocity clamping |
| `test_robot_registry.py` | 32 | Spawn validation, registration, removal, reset, pose updates |
| `test_command_pipeline.py` | 30 | Speed clamping, position prediction, boundary/collision, terrain modifier |
| `test_control_manager.py` | 19 | Acquire/release, velocity watchdog, control timeout |
| `test_ws_protocol.py` | 34 | WebSocket JSON parsing, formatting, validation, inject_event |
| `test_api_gateway_server.py` | 18 | REST endpoints, WebSocket cmd_vel/subscribe, stores |
| `test_kinematics.py` | 20 | Diff-drive update, theta normalization, accel limit, noise |
| `test_kinematics_model.py` | 10 | DifferentialDrive, Omnidirectional, model registry |
| `test_pose.py` | 10 | PoseSource enum, RobotPose frozen dataclass |
| `test_sensor_engine.py` | 20 | Engine config, presets, environment context, state management |
| `test_sensor_plugins.py` | 12 | GPS, lidar, sonar, FoF plugin compute + defaults |
| `test_sensor_presets.py` | 7 | Preset composition, gas_scout, ideal, competitive |
| `test_gas_sensor.py` | 17 | MOX ODE dynamics, state evolution, plume interaction |
| `test_spatial_field.py` | 30 | Gaussian plume, elevation, uniform, composite, iso-contour |
| `test_virtual_layer_loader.py` | 13 | YAML → EnvironmentContext with gas/terrain/background |
| `test_geo_reference.py` | 20 | WGS84 transform, round-trip, rotation, scale factor |
| `test_cot_protocol.py` | 25 | CoT XML generation, parsing, nav goals, UID extraction |
| `test_cot_transport.py` | 8 | UDP unicast/multicast, TCP transport (mocked) |
| `test_gazebo_world_builder.py` | 18 | SDF generation: walls, obstacles, zones, robots, heightmap |
| `test_gazebo_provider.py` | 11 | Quaternion→yaw, pose tracking, GAZEBO_SIM source |
| `test_gazebo_bridge_plugins.py` | 10 | Lidar/IMU bridge passthrough, fallback, thread safety |
| `test_gaden_bridge.py` | 10 | GADEN gas passthrough, MOX dynamics, Python fallback |
| `test_terrain_pipeline.py` | 17 | DEM loading, resampling, PNG generation, YAML parsing |
| `test_ghost_model_manager.py` | 11 | Ghost spawn/update/remove, scale factor, gz service mocking |
| `test_replay.py` | 14 | GPX loading, interpolation, heading wraparound, YAML tracks |

### Integration tests (Docker required)

```bash
# Inside Docker container
cd /ros2_ws
colcon build && source install/setup.bash

# 1. Verify message package built correctly
ros2 interface show plato_pod_msgs/msg/TagPose
ros2 interface show plato_pod_msgs/msg/TagDetections
ros2 interface show plato_pod_msgs/msg/HomographyMatrix

# 2. Start the vision node (virtual-only mode without camera)
ros2 run plato_pod vision_node &
sleep 2

# 3. Verify topics are publishing
ros2 topic list | grep -E "tags|camera|arena"
ros2 topic echo /tags/detections --once

# 4. Verify HTTP endpoints
curl -s http://localhost:8081/health | python3 -m json.tool
curl -s http://localhost:8081/camera/info | python3 -m json.tool

# 5. Stop the node
kill %1
```

### Integration tests with camera

```bash
# With USB camera connected to /dev/video0
ros2 run plato_pod vision_node --ros-args -p debug_overlay:=true &
sleep 2

# Verify detection is running
ros2 topic hz /tags/detections    # should show >= 15 Hz

# Open in browser
# http://localhost:8081/camera/stream        (raw feed)
# http://localhost:8081/camera/stream/debug  (annotated feed)

# Place AprilTag 101 in view, then check arena poses
ros2 topic echo /tags/detections

# Place all 4 boundary tags (100-103) and verify homography
curl -s http://localhost:8081/arena/homography | python3 -m json.tool
```

## Configuration

- `config/apriltag_settings.yaml` — Tag family, sizes, ID ranges, detector parameters
- `config/camera_calibration.yaml` — Camera intrinsics (created by calibration tool)

## Exercise Scenarios

Exercise YAML files define complete tactical scenarios. They live in `config/exercises/` and are loaded at launch via the `exercise_file` parameter.

### Scenario 1: Capture the Flag

**File:** `config/exercises/capture-the-flag.yaml`

Two teams (blue/red) compete to reach the opponent's flag zone while defending their own. Arena has two wall obstacles creating a corridor.

| Property | Value |
|----------|-------|
| Arena | 0.84m x 0.59m |
| Teams | Blue (2 robots), Red (2 robots) |
| Obstacles | 2 rectangular walls |
| Scoring | Zone capture — hold opponent's flag zone for 10 seconds |
| Sensors | `basic_scout` (GPS + sonar) |
| Duration | 5 minutes |
| Rules | Tag-freeze on contact (5s freeze, 2s immunity) |

**Launch (virtual only — no camera, no robots):**
```bash
ros2 launch plato_pod atak_test.launch.py \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml \
  target_host:=<ATAK_IP>
```

**Launch (physical robots on desk):**
```bash
ros2 launch plato_pod classroom.launch.py \
  camera_device:=4 \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml
```

**What to test:**
1. Virtual: spawn robots via `curl -X POST http://localhost:8080/robots/spawn -H 'Content-Type: application/json' -d '{"x":0.2,"y":0.2}'`
2. Physical: power on ESP32 robots, verify they appear on dashboard
3. Drive robots via SDK or dashboard — verify boundary enforcement and collision avoidance

### Scenario 2: Gas Plume Search

**File:** `config/exercises/gas-plume-search.yaml`

An augmented reality exercise where robots search for a virtual gas source. A Gaussian plume disperses downwind from a hidden source point. Each robot carries a virtual MOX gas sensor with realistic first-order response dynamics — the sensor resistance changes gradually as the robot moves through concentration gradients. Teams compete to locate and report the source first.

| Property | Value |
|----------|-------|
| Arena | 0.84m x 0.59m |
| Teams | Alpha (2 robots), Bravo (2 robots) |
| Obstacles | 1 building (rectangle) |
| Virtual layers | Gas: Gaussian plume at (0.7, 0.45), release_rate=200, wind +X at 1.5 m/s |
| Scoring | Source localisation — hold the gas source zone for 5 seconds |
| Sensors | `gas_scout` (GPS + MOX gas sensor) |
| Duration | 10 minutes |

**Launch (virtual only — no camera, no robots):**
```bash
ros2 launch plato_pod atak_test.launch.py \
  exercise_file:=/ros2_ws/config/exercises/gas-plume-search.yaml \
  target_host:=<ATAK_IP> \
  default_preset:=gas_scout
```

**Launch (physical robots on desk):**
```bash
ros2 launch plato_pod classroom.launch.py \
  camera_device:=4 \
  exercise_file:=/ros2_ws/config/exercises/gas-plume-search.yaml \
  default_preset:=gas_scout
```

**What to test:**
1. Spawn robots and verify gas sensor readings change as robots approach (0.7, 0.45)
2. Check sensor data via WebSocket: subscribe to sensors for a robot, observe `concentration` and `raw_resistance` values
3. Verify MOX dynamics: resistance changes gradually (not instantly) as robot enters/exits the plume
4. Inject a wind change mid-exercise via WebSocket:
   ```json
   {"type": "inject_event", "event_type": "update_wind",
    "admin_token": "admin-secret", "data": {"speed": 3.0, "direction": 1.57}}
   ```
5. Verify plume direction changes affect sensor readings

### Creating New Scenarios

Copy an existing YAML and modify. The schema supports:

```yaml
exercise:
  id: "unique-id"
  name: "Display Name"
  description: "..."

  arena:
    boundary: [[x,y], ...]         # polygon vertices in metres
    obstacles:
      - type: rectangle/circle      # with x, y, width/height or radius
        label: "name"

  teams:
    - name: "team_name"
      colour: "#hex"

  robots:
    physical:
      - tag_id: N                   # AprilTag ID
        team: "team_name"
        sensor_preset: "preset_name"
    virtual: []                     # spawned at runtime

  rules:
    max_linear_speed: 0.15          # m/s
    max_angular_speed: 1.5          # rad/s
    duration_minutes: 10

  scoring:
    type: "zone_capture"
    zones:
      - name: "zone_name"
        type: circle/rectangle
        x: 0.5
        y: 0.3
        radius: 0.05               # or width/height for rectangle
        team: "scoring_team"
        hold_time_seconds: 10

  # Optional — virtual data layers for AR exercises
  virtual_layers:
    gas_sources:
      - name: "gas"
        x: 0.5
        y: 0.3
        release_rate: 100.0
        wind_speed: 2.0
        wind_direction: 0.0        # radians, direction wind blows TO
        diffusion_coeff: 0.05

    terrain:
      grid_data: [[0,0,...], ...]   # elevation grid (metres)
      origin: [0, 0]
      resolution: 0.1              # grid cell size in metres

    background_fields:
      - name: "radiation"
        value: 0.1                  # constant value everywhere

    environment:
      wind_speed: 2.0
      wind_direction: 0.0
      temperature: 22.0
```

**Available sensor presets:** `minimal` (GPS), `basic_scout` (GPS + sonar), `full_suite` (GPS + lidar + sonar + FoF), `gas_scout` (GPS + gas), `ideal` (GPS + lidar, zero noise), `competitive` (GPS + sonar + FoF, high noise)

**Available vehicle roles (ATAK symbols):** `default`, `tank`, `apc`, `recon`, `cbrn_recon`, `artillery`, `hostile`, `hostile_tank`, `unknown`, `sensor`

## Classroom Setup — Physical Robots on Desktop Arena

This section covers running physical ESP32 robots on a desktop arena with AprilTag camera localisation. This is the standard setup for programming classes.

### Hardware

- Server: Raspberry Pi 5 / Jetson / mini PC running the Docker container
- USB camera mounted overhead, looking down at the arena
- 2-4 ESP32-C3 Plato Pod robots with AprilTags (IDs 1-4, 50mm)
- 4 boundary AprilTags (IDs 100-103, 70mm) at arena corners
- WiFi — all devices on the same network

### Arena layout

```
    103 ─────────────────── 102
     │                       │
     │     0.84m × 0.59m     │
     │                       │
     │  robots go here       │
     │                       │
    101 ─────────────────── 100
  (origin)
```

Print AprilTags from the `tag36h11` family. Tag 101 is the coordinate origin. Place boundary tags at the four corners of a flat surface.

### Launch

```bash
# Inside Docker container
cd /ros2_ws && colcon build && source install/setup.bash

# Find camera device
v4l2-ctl --list-devices

# Start everything — single command
ros2 launch plato_pod classroom.launch.py \
  camera_device:=4 \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml
```

This starts: vision node (AprilTag detection + MJPEG), arena model, registry + robot bridge (UDP), virtual sim, sensor engine, API gateway with dashboard.

### Verify

1. **Camera feed:** `http://<server-ip>:8081/camera/stream/debug` — should show tag outlines and IDs
2. **Robots registered:** `curl http://localhost:8080/robots` — physical robots appear after power-on
3. **Dashboard:** `http://<server-ip>:8080/` — 2D map with robots, toggle "Camera overlay" for AR view

### Student SDK

```bash
cd web/sdk && pip install -e .
```

```python
from platopod import Arena

arena = Arena("ws://<server-ip>:8080/api/control")

# List available robots
robots = arena.list_robots()
print(f"Robots: {[r['robot_id'] for r in robots]}")

# Drive robot 1
arena.cmd_vel(robot_id=1, linear_x=0.1, angular_z=0.0)
arena.sleep(2.0)
arena.cmd_vel(robot_id=1, linear_x=0.0, angular_z=0.0)

# Read GPS sensor
arena.subscribe_sensors(robot_id=1, sensors=["gps"], rate_hz=10)
gps = arena.get_sensor(robot_id=1, sensor="gps")
print(f"Position: ({gps['x']:.3f}, {gps['y']:.3f})")
arena.close()
```

**Control policy:** First student to send `cmd_vel` acquires control. Auto-releases after 30s of inactivity. Other students can spawn virtual robots alongside physical ones via `arena.spawn(x=0.4, y=0.3)`.

### Troubleshooting

| Problem | Fix |
|---------|-----|
| Tags not detected | Check lighting, reduce `camera_zoom:=100`, try different `camera_device` |
| Robots don't register | Check WiFi. Look for UDP packets in `robot_bridge_node` logs |
| Position jumps | Camera needs calibration: `ros2 run plato_pod camera_calibrate` |
| Dashboard empty | Check `curl http://localhost:8080/robots` — boundary tags may not be visible |
| `robot_busy` error | Another student controls the robot. Wait 30s or ask them to stop |

## ATAK / TAK Integration

Plato Pod sends the complete tactical picture to TAK clients via the Cursor on Target (CoT) protocol. All units — physical and virtual — appear as standard MIL-STD-2525B symbols. Sensor data, scoring zones, obstacles, and CBRN hazard overlays are included.

**Two deployment options:**

| Mode | Setup | Use case |
|------|-------|----------|
| **Direct** | CoT bridge sends UDP to each ATAK device | 1-5 devices, simple setup |
| **TAK Server** | CoT bridge sends to TAK Server, which relays to all clients | 5+ operators, devices join/leave during exercise |

For TAK Server, use [FreeTAKServer](https://github.com/FreeTAKTeam/FreeTakServer) (open source) or an official TAK Server. Point `target_host` at the server IP instead of an individual device.

**See [`docs/tak-setup.md`](../docs/tak-setup.md) for a complete walkthrough of FreeTAKServer + WinTAK on Linux** (via CrossOver/Wine), including map source configuration and troubleshooting.

### Prerequisites

- **ATAK-CIV** on Android (Google Play, free), **WinTAK** on Windows (tak.gov), or **iTAK** on iOS
- Device and server on the same network
- Docker container running

### Quick Start

```bash
# Inside Docker container
cd /ros2_ws && colcon build && source install/setup.bash

# Replace 192.168.1.42 with your ATAK device's IP
ros2 launch plato_pod atak_test.launch.py \
  target_host:=192.168.1.42 \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml
```

Then spawn robots:
```bash
curl -X POST http://localhost:8080/robots/spawn \
  -H 'Content-Type: application/json' -d '{"x":0.2,"y":0.2}'
curl -X POST http://localhost:8080/robots/spawn \
  -H 'Content-Type: application/json' -d '{"x":0.6,"y":0.4}'
```

### What to verify

1. **Symbols appear on ATAK** — navigate to UNSW Canberra (-35.2975, 149.1012), zoom in, look for MIL-STD-2525B symbols (recon by default)
2. **Symbols move** — drive robots via dashboard or SDK, observe movement on ATAK (2 Hz update)
3. **Arena boundary** — a polygon shape should appear on ATAK showing the arena outline
4. **Bidirectional waypoints** — long-press on ATAK map to drop a waypoint, check server logs for `Nav goal from ATAK` message
5. **Mixed physical+virtual** — if physical platopods are on the arena, both physical and virtual robots appear as identical symbols on ATAK

### ATAK Configuration

In ATAK-CIV: Settings > Network Preferences > Network Connection Preferences > ensure TAK Input is enabled on UDP port 4242.

The CoT bridge sends to the configured `target_host:target_port`. Default port is 4242 (ATAK's standard SA input port).

### Geo-Reference

The default geo origin is UNSW Canberra campus. Override for your location:
```bash
ros2 launch plato_pod atak_test.launch.py \
  target_host:=192.168.1.42 \
  geo_origin_lat:=-33.8688 \
  geo_origin_lon:=151.2093
```

The arena boundary (e.g., 0.84m x 0.59m) maps to a small patch of ground at the geo origin. All robot positions are within this patch.

### Runtime Event Injection

During a running exercise, an admin can inject events via WebSocket:

```bash
# Connect via websocat or wscat
websocat ws://localhost:8080/api/control

# Place a gas source
{"type":"inject_event","event_type":"place_gas_source","admin_token":"admin-secret","data":{"x":0.5,"y":0.3,"release_rate":150.0,"wind_speed":2.0,"wind_direction":0.0}}

# Update wind
{"type":"inject_event","event_type":"update_wind","admin_token":"admin-secret","data":{"speed":3.0,"direction":1.57}}

# Reset exercise state
{"type":"inject_event","event_type":"reset_exercise","admin_token":"admin-secret"}
```

### Troubleshooting

| Problem | Fix |
|---------|-----|
| No symbols on ATAK | Check firewall: `sudo ufw allow 4242/udp`. Verify phone IP: `ping <phone-ip>` |
| Symbols don't move | Robots may not be spawned. Check: `curl http://localhost:8080/robots` |
| Wrong location | Adjust `geo_origin_lat` / `geo_origin_lon` |
| Waypoints not received | Ensure inbound port 4242 is open. Check server log for `Nav goal from ATAK` |

## Module Architecture

See `docs/ARCHITECTURE.md` for the full architecture diagram, layer descriptions, and module inventory.

All `*_node.py` files are thin ROS2 shells. All other modules are pure Python, independently testable with pytest (618 tests, no ROS2 needed).

Key abstractions:
- `pose.py` — `RobotPose` + `PoseSource` (localization-agnostic)
- `robot.py` — unified `Robot` model (deployment, localization_id, kinematics_model)
- `kinematics_model.py` — pluggable `KinematicsModel` (DifferentialDrive, Omnidirectional)
- `localization_provider.py` — pluggable `LocalizationProvider` protocol
- `sensor_plugins/` — GPS, LiDAR, Sonar, FoF, Gas (MOX digital twin), Gazebo bridges (lidar, IMU), GADEN bridge
- `spatial_field.py` — `SpatialField` protocol (GaussianPlume, Elevation, Uniform, Composite)
- `sensor_engine.py` — per-robot state, environment context, Gazebo sensor multiplexing
- `virtual_layer_loader.py` — loads `virtual_layers` from exercise YAML
- `terrain_pipeline.py` — DEM → Gazebo heightmap PNG pipeline
- `ghost_model_manager.py` — physical robot → scaled Gazebo ghost model
- `replay.py` — GPX/YAML track loading + interpolation for exercise replay
- `command_pipeline.py` — velocity filtering with terrain speed modifier
- `providers/` — AprilTagProvider, GazeboProvider (future: GPS/IMU, RF anchor)
- `gazebo_world_builder.py` — SDF world generation from exercise YAML + DEM terrain
- `geo_reference.py` — WGS84 arena↔lat/lon with scale factor for classroom↔outdoor mapping
- `cot_protocol.py` + `cot_transport.py` — Cursor on Target XML + UDP/TCP

**Launch files:**
- `classroom.launch.py` — physical robots with camera (programming class)
- `atak_test.launch.py` — virtual robots with ATAK output
- `terrain.launch.py` — Gazebo with real DEM terrain + sensor bridging
- `replay.launch.py` — GPS track replay
- `simulation.launch.py` — mode switcher (lightweight / gazebo / gazebo_terrain / replay)

The Python SDK is at `web/sdk/platopod/` (separate package).
