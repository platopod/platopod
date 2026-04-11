# CLAUDE.md — Plato Pod Development Instructions

## Project Overview

Plato Pod is an educational robotics platform combining physical ESP32-C3 robots and server-side virtual robots on a shared arena. The system uses ROS2 Jazzy on a Raspberry Pi 5 running Ubuntu 24.04. Development happens on a Linux workstation (Ubuntu 24.04, WSL2, or native).

**Repository:** https://github.com/platopod/platopod
**Branch:** v2-platform
**Issue specifications:** `docs/issues/issue_*.md` — read these before implementing anything.

## Architecture Summary

```
firmware/     → ESP32-C3 (ESP-IDF, C, UDP protocol)
server/       → ROS2 nodes (Python 3.12)
web/          → Browser dashboard (HTML/JS, no framework)
hardware/     → KiCad PCB + STL chassis (not code)
config/       → Exercise YAML templates
```

The server is the core — it runs on ROS2 Jazzy and consists of these nodes:
- `arena_model_node` — arena boundary, obstacles, zones (Issue #1)
- `registry_node` — robot registry, spawning, discovery (Issue #2)
- `api_gateway_node` — WebSocket + REST API (Issues #3, #6)
- `robot_bridge_node` — UDP communication with physical ESP32 robots (Issue #2, #3)
- `virtual_sim_node` — virtual robot kinematics (Issue #4)
- `sensor_engine_node` — sensor computation with plugins (Issues #8, #9)
- `exercise_manager_node` — exercise lifecycle, teams, scoring (Issue #7)

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

### Before starting an issue

1. **Read the issue specification** in `docs/issues/issue_N_*.md` completely.
2. **Check dependencies** — are prerequisite issues implemented and tested?
3. **Propose a plan** — outline the files to create/modify, key functions, and test approach. Present to the user for approval before writing code.

### While implementing

1. **Implement incrementally** — one function or component at a time, with tests.
2. **Commit frequently** — each commit should be a logical unit that passes tests.
3. **Run tests after every change** — never commit broken tests.
4. **Ask the user** for design decisions when the spec is ambiguous or when multiple valid approaches exist. Don't guess at intent.

### After implementing

1. **Run the full test suite** — unit and integration.
2. **Verify with RViz2** when implementing nodes that publish visual data.
3. **Present a summary** — what was implemented, what tests pass, any deviations from the spec.
4. **Update the issue spec** if the implementation revealed spec gaps.

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

- **Raspberry Pi 5 (2GB) is the target.** Code must be efficient — avoid unnecessary memory allocation, prefer generators over lists for large datasets.
- **Shapely for geometry, not a physics engine.** We need ray-polygon intersection, not rigid body dynamics.
- **All coordinates in metres relative to AprilTag 101.** This is the universal coordinate system.
- **Physical robots use plain UDP, not micro-ROS.** The `robot_bridge_node` translates between ROS2 topics and the ESP32 UDP protocol. micro-ROS is planned as future work.
- **Sensor plugins are stateless.** They receive a SensorContext and return data. No side effects, no ROS2 subscriptions inside plugins.
- **The WebSocket API carries everything** — commands, poses, events, sensor data. One connection per client, multiplexed by message type.
- **Free-play mode is the default.** When no exercise is loaded, no permissions are enforced. The system should be fully functional without the exercise manager.

## Issue Implementation Order

Issues should be implemented roughly in this order due to dependencies:

1. **Issue #1** — Arena model (foundation)
2. **Issue #2** — Robot registry (needs #1 for spawn validation)
3. **Issue #4** — Virtual simulation (can develop in parallel with #3)
4. **Issue #3** — Control API (needs #1, #2; pulls in #4)
5. **Issue #5** — Dashboard Phase 1 (needs #3)
6. **Issue #8** — Sensor engine + Lidar/Sonar (needs #1, #4)
7. **Issue #6** — Sensor API (needs #8 for data to serve)
8. **Issue #9** — GPS/FoF plugins (needs #7 for teams, #8 for engine)
9. **Issue #7** — Exercise management (needs #2, #3, #6)
10. **Issue #10** — Dashboard Phase 2 (needs everything)

Within each issue, implement and test the core functionality first, then edge cases and optional features.
