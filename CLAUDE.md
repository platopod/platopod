# CLAUDE.md — Plato Pod Development Instructions

## Project Overview

Plato Pod is an augmented reality tactical simulation platform that operates at multiple scales — from desktop arenas with small differential-drive robots localised by AprilTag cameras, to outdoor exercises with military units carrying GPS/IMU for localisation. Physical and virtual elements coexist on the same arena: real robots interact with virtual units (blue/red force), virtual gas plumes, terrain overlays, obstacles, and engagement rules to create sophisticated tactical training scenarios. The system uses ROS2 Jazzy. Target hardware: NVIDIA Jetson, mini PC with GPU, or Raspberry Pi 5. Development happens on a Linux workstation (Ubuntu 24.04, WSL2, or native).

**Repository:** https://github.com/platopod/platopod
**Branch:** v2-platform

## Architecture Summary

```
firmware/     → ESP32-C3 (ESP-IDF, C, UDP protocol)
server/       → ROS2 nodes (Python 3.12)
web/          → Browser dashboard (HTML/JS, no framework)
hardware/     → KiCad PCB + STL chassis (not code)
config/       → Exercise YAML templates
```

The server is the core — it runs on ROS2 Jazzy. Cross-node payloads are typed messages defined in `server/plato_pod_msgs/`; see `docs/message-catalog.md` for the polyglot contract.

**Core nodes:**
- `arena_model_node` — arena boundary, obstacles, zones from exercise YAML
- `registry_node` — robot registry, spawning, discovery, `apply_damage` service
- `api_gateway_node` — WebSocket + REST API, command pipeline, event injection
- `robot_bridge_node` — UDP communication with physical ESP32 robots
- `virtual_sim_node` — virtual robot kinematics (lightweight mode)
- `sensor_engine_node` — sensor computation; publishes typed `SensorReading.msg`
- `gazebo_bridge_node` — Gazebo lifecycle, terrain, ghost models, sensor bridging
- `cot_bridge_node` — CoT/ATAK integration (positions, zones, sensor data, engagement events)
- `replay_node` — GPS track replay from recorded exercises
- `vision_node` — AprilTag detection and MJPEG streaming

**Tactical nodes (engagement, OPFOR, world state, line-of-sight):**
- `world_state_node` — single source of truth for cover, civilians, IEDs, EW emitters, jamming/dead zones, weather, ROE, weapons (publishes latched `/world/*` topics)
- `opfor_node` — autonomous OPFOR FSM; subscribes `/world/*`, publishes `cmd_vel` and typed `FireIntent` on `/fire_weapon`
- `engagement_node` — central fire evaluator; subscribes `FireIntent`, publishes typed `EngagementOutcome` on `/engagement_events`
- `los_python_node` — line-of-sight service via 2D Python ray model
- `los_gazebo_node` — line-of-sight service via Gazebo lidar (same `EvaluateLos.srv` contract; pick a backend)

**Polyglot story:** the cross-node contract is the catalog of typed ROS2 messages and services in `plato_pod_msgs/`. Any language with a ROS2 binding (Python, C++, MATLAB, Rust, Java) can implement or replace any node. See `docs/message-catalog.md`.

### Physical robot communication

Physical robots communicate with the server via **plain UDP** (not micro-ROS). The `robot_bridge_node` translates between ROS2 topics and the ESP32 UDP protocol:

```
ROS2 topic /robot_{id}/cmd_vel → robot_bridge_node → UDP "M <linear> <angular>" → ESP32
ESP32 → UDP "REG <tag_id> <radius_mm>" → robot_bridge_node → registry_node
```

UDP protocol commands (see `firmware/main/main.c` for reference implementation):

| Command | Direction | Format | Response |
|---------|-----------|--------|----------|
| Move | Server → Robot | `M <linear> <angular>` | `OK` |
| Stop | Server → Robot | `S` | `OK` |
| LED on/off | Server → Robot | `L1` / `L0` | `OK` |
| RGB colour | Server → Robot | `C <r> <g> <b>` | `OK` |
| Display text | Server → Robot | `D <text>` | `OK` |
| Register | Robot → Server | `REG <tag_id> <radius_mm>` | `OK <robot_id>` |
| Heartbeat | Server → Robot | `H` | `OK` |
| Ping | Either | `P` | `PONG` |

micro-ROS integration is planned as future work once ESP-IDF compatibility issues are resolved.

## Development Environment

### Docker (recommended)

The project includes a Docker setup with ROS2 Jazzy and all dependencies pre-installed:

```bash
cd docker
docker compose build          # first time only
docker compose up -d          # start container
docker compose exec ros bash  # open shell inside container
```

The container runs with `--net=host` and `--privileged` for camera and WiFi dongle access. Source directories are mounted as volumes — edit code on the host, build inside the container. See `docker/README.md` for full details.

### Native ROS2 Jazzy (alternative)

```bash
# Install ROS2 Jazzy on Ubuntu 24.04
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install ros-jazzy-desktop

# Source ROS2 in every terminal
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Python dependencies

```bash
pip install shapely numpy websockets fastapi uvicorn pyyaml pytest pytest-asyncio --break-system-packages
```

### ESP-IDF (for firmware only)

```bash
# Only needed when working on firmware/ directory
# See: https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/
```

## Coding Standards

### Python (server/)

- **Python 3.12+**, type hints on all function signatures
- **Functional style preferred** — pure functions where possible, minimize class state
- **Dataclasses** for data structures, not dicts
- **ROS2 conventions:** node names in snake_case, topic names with forward slashes
- **No global state** — all state lives in node classes or is passed via function arguments
- **Docstrings** on all public functions and classes (Google style)
- **Line length:** 100 characters max
- **Imports:** standard library, then third-party, then local, separated by blank lines

### C (firmware/)

- **ESP-IDF style** — follow Espressif coding conventions
- **Function paradigm** — minimize global state, pass context structs
- **Naming:** snake_case for functions and variables, UPPER_CASE for constants
- **Comments:** explain why, not what

### JavaScript (web/)

- **Vanilla JS** — no frameworks for Phase 1
- **ES6+ modules** where supported
- **Single HTML file** for Phase 1 dashboard

## Build and Test

### Server (ROS2 nodes)

```bash
# Build
cd server
colcon build

# Source workspace
source install/setup.bash

# Run all tests
colcon test
colcon test-result --verbose

# Run a specific node
ros2 run plato_pod arena_model_node

# Launch everything
ros2 launch plato_pod server.launch.py
```

### Unit tests

```bash
# Run Python unit tests directly
cd server
python -m pytest tests/ -v

# Run a specific test file
python -m pytest tests/test_ray_caster.py -v

# Run with coverage
python -m pytest tests/ --cov=src --cov-report=term-missing
```

### Integration tests

```bash
# Start the server, then run integration tests
ros2 launch plato_pod server.launch.py &
sleep 5
python -m pytest tests/integration/ -v
```

### Firmware

```bash
cd firmware
idf.py set-target esp32c3
idf.py build
idf.py flash monitor  # requires connected ESP32
```

## Test Strategy

### Test structure

```
server/
├── src/           # implementation
└── tests/
    ├── unit/      # fast, no ROS2 dependency
    │   ├── test_ray_caster.py
    │   ├── test_arena_model.py
    │   ├── test_kinematics.py
    │   ├── test_sensor_plugins.py
    │   └── test_noise_injector.py
    ├── integration/  # requires running ROS2 nodes
    │   ├── test_robot_registry.py
    │   ├── test_command_pipeline.py
    │   ├── test_sensor_engine.py
    │   └── test_websocket_api.py
    └── conftest.py   # shared fixtures
```

### What to test

- **Unit tests:** Pure functions (ray casting, kinematics, noise injection, polygon validation). These must run without ROS2.
- **Integration tests:** Node interactions (spawn robot → send cmd_vel → check pose update). These require a running ROS2 graph.
- **Every new function gets a test.** Write the test first when the expected behaviour is clear.

### Test conventions

- Use `pytest` with `pytest-asyncio` for async WebSocket tests
- Fixtures in `conftest.py` for common setup (arena model, sample robots)
- Test names: `test_<function>_<scenario>_<expected_result>`
- Example: `test_ray_cast_against_boundary_returns_nearest_hit`

## Development Workflow

### Before starting a task

1. **Propose a plan** — outline the files to create/modify, key functions, and test approach. Present to the user for approval before writing code.
2. **Check existing code** — understand existing patterns and abstractions before proposing new ones.

### While implementing

1. **Implement incrementally** — one function or component at a time, with tests.
2. **Commit frequently** — each commit should be a logical unit that passes tests.
3. **Run tests after every change** — never commit broken tests.
4. **Ask the user** for design decisions when the spec is ambiguous or when multiple valid approaches exist. Don't guess at intent.

### After implementing

1. **Run the full test suite** — unit and integration.
2. **Present a summary** — what was implemented, what tests pass, any deviations from the plan.

## Commit Messages

Format: `[component] short description`

Examples:
```
[arena] implement boundary polygon from AprilTag corners
[registry] add robot spawn with collision validation
[api] websocket cmd_vel handler with speed limiting
[sensors] lidar plugin with configurable resolution
[tests] unit tests for ray caster against polygons
[firmware] NVS storage for tag_id and radius
[web] phase 1 dashboard with camera feed overlay
[config] capture-the-flag exercise template
```

## Key Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| ROS2 Jazzy | latest | Middleware |
| Python | 3.12+ | Server language |
| Shapely | ≥2.0 | Computational geometry (ray casting) |
| NumPy | latest | Numerics, noise generation |
| pupil-apriltags | latest | AprilTag detection and pose estimation |
| FastAPI | latest | REST API + WebSocket server |
| uvicorn | latest | ASGI server for FastAPI |
| websockets | latest | WebSocket client for testing |
| PyYAML | latest | Exercise config parsing |
| pytest | latest | Testing framework |
| pytest-asyncio | latest | Async test support |

## File Naming Conventions

- ROS2 nodes: `server/src/plato_pod/<node_name>_node.py`
- Sensor plugins: `server/plugins/sensors/<sensor_name>_plugin.py`
- Scoring plugins: `server/plugins/scoring/<scoring_type>_plugin.py`
- Tests: `server/tests/unit/test_<module>.py`
- Launch files: `server/launch/<name>.launch.py`
- Config: `server/config/<name>.yaml`
- Exercise templates: `config/exercises/<exercise_name>.yaml`

## Important Constraints

- **GPU is optional.** If GPU is available, use gpu_lidar and GPU rendering. If not, fall back to cpu_lidar and software rendering. The platform must always work without GPU.
- **Target hardware:** NVIDIA Jetson, mini PC with GPU, or Raspberry Pi 5. Code must be efficient — avoid unnecessary memory allocation, prefer generators over lists for large datasets.
- **Gazebo for physics, Python for tactics.** Gazebo owns terrain physics, sensor ray casting, collision dynamics. Python owns engagement rules, scoring, gas simulation (research iteration speed), exercise management.
- **All coordinates in metres relative to arena origin (AprilTag 101 when using camera localisation).** GeoReference with scale_factor handles conversion to WGS84.
- **Physical robots use plain UDP, not micro-ROS.** The `robot_bridge_node` translates between ROS2 topics and the ESP32 UDP protocol.
- **Sensor plugins receive EnvironmentContext and per-robot state.** Gas sensors use stateful MOX ODE dynamics. Lidar/IMU can be bridged from Gazebo or computed in Python fallback.
- **The WebSocket API carries everything** — commands, poses, events, sensor data, environment updates, event injection (admin-gated). One connection per client, multiplexed by message type.
- **Free-play mode is the default.** When no exercise is loaded, no permissions are enforced. The system should be fully functional without the exercise manager.

## Module Architecture

See `docs/ARCHITECTURE.md` for the full layer diagram, `docs/message-catalog.md` for the polyglot ROS2 contract, and `docs/tactical-capabilities.md` for the tactical layer.

**Core abstractions (Layer 3, pure Python):**
- `pose.py` — `RobotPose` + `PoseSource` enum (localization-agnostic)
- `robot.py` — unified `Robot` dataclass (deployment, localization, kinematics, tactical fields)
- `kinematics_model.py` — pluggable `KinematicsModel` (DifferentialDrive, Omnidirectional)
- `sensor_engine.py` — per-robot state, environment context, Gazebo bridge multiplexing
- `spatial_field.py` — `SpatialField` protocol (GaussianPlume, Elevation, Uniform, Composite)
- `terrain_pipeline.py` — DEM → Gazebo heightmap PNG pipeline
- `ghost_model_manager.py` — physical robot → scaled Gazebo ghost model
- `replay.py` — GPX/YAML track loading + interpolation
- `geo_reference.py` — WGS84 arena↔lat/lon with scale factor
- `cot_protocol.py` — CoT XML generation/parsing, MIL-STD-2525B type codes
- `command_pipeline.py` — velocity filtering: state → mobility → fuel → speed → terrain → boundary → collision

**Tactical abstractions (Layer 3, pure Python):**
- `world_state.py` — `WorldState` dataclass + `world_state_from_config(yaml)` (single source of truth)
- `engagement.py` — `evaluate_fire`, `WeaponSpec`, `EngagementOutcome`
- `line_of_sight.py` — `has_line_of_sight` 2D + height-sample model
- `weather.py` — `WeatherState`, `visibility_factor`
- `health.py` — `apply_damage`, `mobility_factor`, `fire_capability`, status thresholds
- `behavior.py` — `BehaviorTree` FSM for OPFOR
- `comms.py` — `evaluate_comms` with jamming/dead zones
- `logistics.py` — `Logistics` (fuel/ammo/water), `consume_*`, `resupply`
- `roe.py` — `check_fire_roe` (weapons hold/tight/free + civilian/friendly checks)
- `sensor_plugins/` — gas, gps, lidar, sonar, fof, thermal, rangefinder, ied_detector, df_receiver, uav_camera
