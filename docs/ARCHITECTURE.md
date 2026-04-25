# Plato Pod Platform Architecture

## Overview

Plato Pod is an augmented reality tactical simulation platform that operates at multiple physical scales:

- **Desktop** — Small differential-drive robots (ESP32-C3) on a tabletop arena, localised by an overhead camera via AprilTags. Robots carry the kinematic characteristics of real vehicles (tanks, APCs, recon units).
- **Indoor** — Larger robots or robo-dogs in a room-scale arena, localised by camera or RF anchors.
- **Outdoor** — Military units or vehicles carrying GPS receivers and IMUs for localisation and orientation.

At every scale, the platform mixes physical and virtual elements on the same arena. Real robots interact with virtual units (blue and red force), virtual gas plumes, terrain elevation overlays, obstacles, and engagement rules — enabling sophisticated tactical scenario design without requiring all elements to be physically present.

## System Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ CLIENTS                                                                      │
│  Web Dashboard    Python SDK    Keyboard Demo    ATAK (CoT)    REST/curl     │
│  (map + camera)   (platopod)    (curses)         (military)                  │
└──────────┬───────────┬────────────┬───────────────┬──────────────┬───────────┘
           │           │            │               │              │
    ┌──────▼───────────▼────────────▼───────────────┼──────────────▼──────┐
    │                API Gateway (port 8080)         │                     │
    │   REST: /robots, /arena/model, /camera/*       │                     │
    │   WebSocket: /api/control                      │                     │
    └──────────────────┬─────────────────────────────┼────────────────────┘
                       │                             │
    ┌──────────────────┼─────────────────┐    ┌──────▼──────┐
    │                  │                 │    │ CoT Bridge  │
    │   ┌──────────────▼───────────┐     │    │ (port 6969) │
    │   │     Command Pipeline     │     │    │ UDP/TCP→ATAK│
    │   │ speed→boundary→collision │     │    └─────────────┘
    │   └──────────────┬───────────┘     │
    │                  │                 │
    │   ┌──────▼──────┐  ┌──────────┐   │
    │   │  Registry   │  │  Sensor  │   │
    │   │   Node      │  │  Engine  │   │
    │   └──────┬──────┘  └──────────┘   │
    │          │                         │
    │   ┌──────┴────────────────────┐    │
    │   │                          │    │
    │ ┌─▼──────────┐  ┌────────────▼─┐  │
    │ │  Gazebo     │  │Virtual Sim  │  │
    │ │  Bridge     │  │(lightweight)│  │
    │ │  (3D mode)  │  │(Python 50Hz)│  │
    │ └──────┬──────┘  └─────────────┘  │
    │        │                          │
    │   ┌────▼─────┐                    │
    │   │ gz-sim   │                    │
    │   │ (Gazebo) │                    │
    │   └──────────┘                    │
    │                                   │
    │ ┌────────────┐  ┌──────────────┐  │
    │ │ Vision     │  │ Arena Model  │  │
    │ │ Node       │  │ Node         │  │
    │ │(AprilTag)  │  │(YAML-first)  │  │
    │ └────────────┘  └──────────────┘  │
    │                                   │
    │ ┌────────────┐                    │
    │ │Robot Bridge│ ← UDP port 9999   │
    │ │(ESP32 UDP) │                    │
    │ └────────────┘                    │
    └───────────────────────────────────┘
```

## Design Principles

1. **Virtual-first** — arena from YAML, no camera or hardware needed. Exercises run entirely in software for planning, rehearsal, and analysis.
2. **Scale-agnostic** — the same exercise definition drives a desktop arena with AprilTag-tracked robots, or an outdoor field with GPS-equipped units.
3. **Localization-agnostic** — `RobotPose` with `PoseSource` enum. Camera, GPS, RF, or Gazebo — downstream code never knows.
4. **Robot-type-agnostic** — pluggable `KinematicsModel`. Differential drive, omnidirectional, or Ackermann. Small robots carry the characteristics of real vehicles.
5. **Mixed reality** — physical and virtual elements coexist. Virtual gas plumes, terrain overlays, enemies, and engagement rules interact with real robot positions in real time.
6. **Layer discipline** — pure Python domain logic (Layer 3, no ROS2) + thin ROS2 node shells (Layer 2).
7. **Three operating modes** — lightweight (Python kinematics), Gazebo (3D physics), physical+virtual overlay (real robots with virtual data layers). Same API for all.

## Layer Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│ Layer 5: CLIENTS                                                     │
│   Web Dashboard, Python SDK, ATAK (CoT), REST clients                │
├─────────────────────────────────────────────────────────────────────┤
│ Layer 4: EXTERNAL INTERFACES                                         │
│   api_gateway_node (REST+WS :8080)                                   │
│   cot_bridge_node (CoT XML :6969/:4242)                              │
├─────────────────────────────────────────────────────────────────────┤
│ Layer 3: DOMAIN LOGIC (pure Python, no ROS2, 618 tests)              │
│   robot.py, robot_registry.py, command_pipeline.py, control_manager  │
│   arena_model.py, geometry.py, kinematics_model.py, sensor_engine    │
│   pose.py, localization_provider.py, geo_reference.py                │
│   spatial_field.py, virtual_layer_loader.py, terrain_pipeline.py     │
│   ghost_model_manager.py, replay.py                                  │
│   cot_protocol.py, cot_transport.py, gazebo_world_builder.py         │
├─────────────────────────────────────────────────────────────────────┤
│ Layer 2: ROS2 NODES (thin shells)                                    │
│   registry_node, api_gateway_node, robot_bridge_node                 │
│   virtual_sim_node, sensor_engine_node, arena_model_node             │
│   vision_node, cot_bridge_node, gazebo_bridge_node, replay_node      │
├─────────────────────────────────────────────────────────────────────┤
│ Layer 1: LOCALIZATION PROVIDERS (pluggable)                          │
│   AprilTagProvider (camera), GazeboProvider (gz-sim)                  │
│   GpsImuProvider (future), RfAnchorProvider (future)                 │
├─────────────────────────────────────────────────────────────────────┤
│ Layer 0: HARDWARE / SIMULATION                                       │
│   ESP32 robots, USB camera, Gazebo gz-sim, GPS, UWB anchors         │
└─────────────────────────────────────────────────────────────────────┘
```

## Core Abstractions

| Module | Description |
|--------|-------------|
| **`pose.py`** | `RobotPose` + `PoseSource` enum (CAMERA_ARTAG, GPS_IMU, RF_ANCHOR, VIRTUAL_SIM, GAZEBO_SIM). Single interface between localization and platform. |
| **`robot.py`** | `Robot` dataclass: `deployment` (physical/virtual), `localization_id`, `localization_source`, `kinematics_model`, `team`, `sensor_preset`. Used everywhere. |
| **`kinematics_model.py`** | `KinematicsModel` protocol. `DifferentialDrive` and `Omnidirectional` implementations. Pluggable motion prediction. |
| **`localization_provider.py`** | `LocalizationProvider` protocol. Each backend produces `RobotPose` objects. |
| **`spatial_field.py`** | `SpatialField` protocol for virtual data layers. `GaussianPlumeField` (gas), `ElevationField` (terrain), `UniformField`, `CompositeField`. |
| **`sensor_engine.py`** | Per-robot sensor state, `EnvironmentContext` with spatial fields, Gazebo sensor bridge multiplexing. |
| **`geo_reference.py`** | WGS84 arena↔lat/lon with rotation and `scale_factor` for classroom↔outdoor mapping. |
| **`terrain_pipeline.py`** | DEM data → Gazebo heightmap PNG. Supports GeoTIFF, SRTM, numpy arrays. |
| **`ghost_model_manager.py`** | Spawns/updates/removes ghost models in Gazebo for physical desktop robots at scaled positions. |
| **`replay.py`** | GPX/YAML track loading and interpolation for exercise replay. |
| **`command_pipeline.py`** | Velocity filtering: speed limit → terrain modifier → boundary → collision. |

## Operating Modes

| Mode | Launch | What runs | Use case |
|------|--------|-----------|----------|
| `lightweight` | `mode:=lightweight` | `virtual_sim_node` (50 Hz Python kinematics) | Planning, rehearsal, CI, laptops |
| `gazebo` | `mode:=gazebo` | `gazebo_bridge_node` + `gz-sim` (flat arena, 3D physics) | 3D visualisation, demos |
| `gazebo_terrain` | `mode:=gazebo_terrain` | Gazebo with DEM heightmaps + sensor bridging + ghost models | Real terrain exercises, DoD |
| `replay` | `mode:=replay` | `replay_node` plays back recorded GPS tracks | Exercise replay and analysis |
| `physical` | Camera/GPS localisation providers | Real robots + virtual data layers | AR tactical exercises |

```bash
ros2 launch plato_pod simulation.launch.py mode:=lightweight
ros2 launch plato_pod simulation.launch.py mode:=gazebo_terrain \
  exercise_file:=/ros2_ws/config/exercises/adfa-terrain-patrol.yaml \
  scale_factor:=1000
ros2 launch plato_pod atak_test.launch.py \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml \
  target_host:=192.168.1.42
```

All modes publish to identical ROS2 topics. All upstream code (registry, pipeline, gateway, dashboard, CoT) works unchanged.

## Gazebo Integration

**World generation:** `gazebo_world_builder.py` generates SDF worlds from exercise YAML — walls, obstacles, zones, robots, and terrain heightmaps. `terrain_pipeline.py` converts DEM data (GeoTIFF, SRTM) to Gazebo-compatible PNG heightmaps. Pure Python.

**Sensor bridging:** `gazebo_bridge_node.py` launches gz-sim and ros_gz_bridge, configures pose/cmd_vel/lidar/IMU topic bridging for each robot. GPU detection at startup selects gpu_lidar or cpu_lidar. `sensor_plugins/gazebo_bridge.py` passes Gazebo sensor data through the Python sensor engine in the same dict format as Python plugins — downstream code sees identical data regardless of source.

**Ghost models:** When `scale_factor > 1.0`, `ghost_model_manager.py` spawns virtual counterparts of physical desktop robots in the Gazebo world at scaled positions. A robot at (0.3, 0.2) on a 0.84m desk arena appears at (300, 200) in the Gazebo terrain world. Gazebo sensors on the ghost model produce lidar/IMU data that flows back to the physical robot's sensor stream.

**Gas simulation:** `sensor_plugins/gaden_bridge.py` bridges GADEN gas concentration data when available. Falls back to Python `GaussianPlumeField` + MOX ODE dynamics when GADEN is not running.

| Vehicle model | Dimensions | Mass | Max speed | Sensors |
|---------------|-----------|------|-----------|---------|
| **platopod** | 0.08m dia, 0.03m tall | 0.15 kg | — | lidar (2m), IMU, contact |
| **tank** | 3.0 × 1.5 × 1.0m | 50,000 kg | 15 m/s | lidar (500m), IMU, contact |
| **apc** | 2.5 × 1.2 × 0.8m | 15,000 kg | 25 m/s | lidar (200m), IMU, contact |
| **recon** | 1.0 × 0.6 × 0.4m | 500 kg | 40 m/s | lidar (100m), IMU, contact |

## CoT/ATAK Integration

The CoT bridge sends the complete tactical picture to ATAK military tablets and receives waypoints back.

| Data | CoT mechanism | Direction |
|------|---------------|-----------|
| Robot positions + heading | `<point>` + `<track>` with MIL-STD-2525B type codes | → ATAK |
| Sensor readings (gas, etc.) | `<sensor>` detail + `<remarks>` text | → ATAK |
| Arena boundary | Shape event polygon | → ATAK |
| Scoring zones | Semi-transparent team-coloured polygons | → ATAK |
| Obstacles | Grey polygons with labels | → ATAK |
| Waypoints / nav goals | Waypoint CoT events | ← ATAK |

**Components:** `geo_reference.py` (WGS84 with scale_factor), `cot_protocol.py` (XML gen/parse + `robot_id_from_uid`), `cot_transport.py` (UDP/TCP), `cot_bridge_node.py` (subscribes to status/arena/sensors, publishes to ATAK).

## ROS2 Nodes

| Node | Description |
|------|-------------|
| `vision_node` | Camera → AprilTagProvider → /tags/detections, homography, MJPEG stream (port 8081) |
| `arena_model_node` | YAML-first boundary. Publishes /arena/model. Falls back to tag detection if no YAML. |
| `registry_node` | 5 ROS2 services. Localization-agnostic registration. Publishes /robots/status at 10 Hz. |
| `robot_bridge_node` | ESP32 UDP (port 9999). REG→register, cmd_vel→UDP motor, heartbeat. |
| `virtual_sim_node` | 50 Hz kinematics with pluggable KinematicsModel. Lightweight simulation mode. |
| `sensor_engine_node` | Multiplexes Python plugins (gas, GPS, FoF) and Gazebo bridges (lidar, IMU). Auto-applies presets. |
| `api_gateway_node` | FastAPI REST+WS (port 8080). Command pipeline, control ownership, event injection, pose broadcast. |
| `cot_bridge_node` | Robot positions + zones + obstacles + sensor data → CoT XML → ATAK. Inbound waypoints from ATAK. |
| `gazebo_bridge_node` | Generates SDF world, launches gz-sim + ros_gz_bridge, manages ghost models, bridges sensors. |
| `replay_node` | Plays back GPX/YAML tracks as virtual robot positions at configurable speed. |

## ROS2 Messages & Services

### Messages
| Message | Description |
|---------|-------------|
| `RobotPose.msg` | Generic pose from any localization backend |
| `RobotPoses.msg` | Batch of poses from one provider |
| `RobotStatus.msg` | Robot state: deployment, localization_id, localization_source, pose |
| `RobotStatusList.msg` | All robots with timestamp |
| `ArenaModel.msg` | Boundary + obstacles + zones |
| `NavGoal.msg` | Navigation waypoint from ATAK |
| `TagPose.msg` | AprilTag-specific pose (vision node) |
| `TagDetections.msg` | AprilTag detection batch (vision node) |
| `HomographyMatrix.msg` | Camera pixel↔arena transform |
| `Obstacle.msg` / `Zone.msg` | Arena elements |

### Services
| Service | Description |
|---------|-------------|
| `RegisterPhysical.srv` | Register robot (localization_id, source, pose, radius) |
| `SpawnVirtual.srv` | Spawn virtual robot (x, y, theta, radius) |
| `RemoveRobot.srv` / `ResetRobot.srv` | Remove virtual / reset physical |
| `ListRobots.srv` | Query all robots |

## Ports

| Port | Service | Protocol |
|------|---------|----------|
| 8080 | API Gateway | HTTP REST + WebSocket + dashboard |
| 8081 | Vision MJPEG | HTTP stream + tag detections |
| 9999 | Robot Bridge | UDP (ESP32 firmware) |
| 6969 | CoT Bridge (out) | UDP/TCP to ATAK/TAK Server |
| 4242 | CoT Bridge (in) | UDP listener for ATAK waypoints |

## Client-Side

| Component | Description |
|-----------|-------------|
| `web/static/index.html` | Dashboard: 2D map (default) + camera AR overlay (toggle) |
| `web/sdk/platopod/arena.py` | Python SDK: Arena class, sync API over async WebSocket |
| `web/sdk/platopod/keyboard.py` | KeyboardController: curses → cmd_vel at 10 Hz |

## Test Coverage

**618 unit tests** — all run without ROS2, Docker, or hardware. Covers domain logic, spatial fields, sensor plugins, gas dynamics, terrain pipeline, geo-reference, CoT protocol, replay interpolation, and ghost model management.

## Quick Start

```bash
# Lightweight mode (no Gazebo, no camera)
ros2 launch plato_pod simulation.launch.py mode:=lightweight &
ros2 run plato_pod arena_model_node --ros-args \
  -p exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml &
ros2 run plato_pod registry_node &
ros2 run plato_pod sensor_engine_node &
ros2 run plato_pod api_gateway_node &
curl -X POST http://localhost:8080/robots/spawn \
  -H 'Content-Type: application/json' -d '{"x":0.4,"y":0.3}'
# Open http://localhost:8080/

# Gazebo mode (3D physics)
ros2 launch plato_pod simulation.launch.py mode:=gazebo \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml \
  headless:=false
```
