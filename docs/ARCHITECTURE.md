# Plato Pod Platform Architecture

## Overview

Plato Pod is an augmented reality tactical simulation platform that operates at multiple physical scales:

- **Desktop** — Small differential-drive robots (ESP32-C3) on a tabletop arena, localised by an overhead camera via AprilTags. Robots carry the kinematic characteristics of real vehicles (tanks, APCs, recon units).
- **Indoor** — Larger robots or robo-dogs in a room-scale arena, localised by camera or RF anchors.
- **Outdoor** — Military units or vehicles carrying GPS receivers and IMUs.

At every scale, the platform mixes physical and virtual elements on the same arena. Real robots interact with virtual units (blue and red force), virtual gas plumes, terrain elevation overlays, obstacles, weapons engagements, casualties, communications failures, and rules of engagement — enabling sophisticated tactical scenario design without requiring all elements to be physically present.

## Design principles

1. **Polyglot by design** — the public contract is a catalog of typed ROS2 messages and services (see [`message-catalog.md`](message-catalog.md)). Any language with a ROS2 binding (Python, C++, MATLAB, Rust, Java) can implement or replace any node.
2. **Single source of truth for the world** — `world_state_node` parses the exercise YAML once and publishes cover, civilians, IEDs, EW emitters, jamming zones, weather, ROE, and the weapons catalog as latched topics. Every other node subscribes; nobody re-parses YAML at runtime.
3. **Two backends behind one contract** — physics-y operations (line-of-sight, terrain occlusion) have a Python fallback and a Gazebo backend that satisfy the same service contract. Pick by launching the corresponding node.
4. **Layer discipline** — pure-language algorithm modules (Layer 3) + thin ROS2 node shells (Layer 2). Algorithm modules are testable without ROS2 and replaceable in any language.
5. **Virtual-first** — arena from YAML, no camera or hardware needed. Exercises run entirely in software for planning, rehearsal, and CI.
6. **Scale-agnostic** — the same exercise definition drives a desktop arena with AprilTag-tracked robots or an outdoor field with GPS units. `scale_factor` in the geo block does the mapping.
7. **Localization-agnostic** — `RobotPose` with `PoseSource` enum. Camera, GPS, RF, or Gazebo — downstream code never knows.
8. **Robot-type-agnostic** — pluggable `KinematicsModel`. Differential drive, omnidirectional, or Ackermann.
9. **Mixed reality** — physical and virtual elements coexist; the engagement, sensor, comms, and logistics layers don't distinguish between them.

## System overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ EXTERNAL CLIENTS                                                             │
│  Web Dashboard    Python SDK    ATAK / WinTAK    REST clients                │
└─────────┬──────────────┬─────────────┬─────────────┬─────────────────────────┘
          │              │             │ CoT XML     │
          │ WebSocket    │             ▼             │ REST
          ▼              ▼     ┌───────────────┐     ▼
    ┌────────────────────────────────────────────────────────────────────┐
    │  api_gateway_node  (FastAPI :8080  +  WebSocket /api/control)      │
    │  cot_bridge_node   (UDP :6969 → ATAK; UDP :4242 ← waypoints)       │
    └────────┬─────────────────────────────┬─────────────────────────────┘
             │                             │
             │ FireIntent, Observation     │ subscribes to /robots/status,
             │ cmd_vel, etc.               │ /world/*, /engagement_events,
             ▼                             │ /robot_*/sensors/*
    ┌─────────────────────────────────────────────────────────────────────┐
    │ TACTICAL LAYER (typed ROS2 messages)                                │
    │                                                                     │
    │  world_state_node  ──► /world/cover, /civilians, /ied_zones,        │
    │                        /ew_emitters, /jamming, /dead_zones,         │
    │                        /weather, /roe, /weapons   (all latched)     │
    │                                                                     │
    │  registry_node     ──► /robots/status (pose, status, team, health,  │
    │                        weapons, vehicle_role, thermal_signature)    │
    │                        ~/spawn_virtual, ~/apply_damage, …           │
    │                                                                     │
    │  opfor_node        ──► /robot_<id>/cmd_vel, /fire_weapon (FireIntent)│
    │                       (autonomous OPFOR FSM: PATROL→OBSERVE→…)      │
    │                                                                     │
    │  engagement_node   ──► /engagement_events (EngagementOutcome)       │
    │                       (PoK, ROE, LoS, civilians, suppression,       │
    │                        damage RPC to registry_node)                 │
    │                                                                     │
    │  los_python_node   │                                                │
    │  los_gazebo_node   ┴► EvaluateLos service (interchangeable backends)│
    └────────┬─────────────────────────────────────────────────────────────┘
             │
             │ /robot_<id>/cmd_vel filtered through command pipeline
             ▼
    ┌──────────────────────────────────────────────────────────────────────┐
    │ MOTION LAYER                                                         │
    │                                                                      │
    │ command pipeline (in api_gateway_node):                              │
    │   state → mobility → fuel → speed → terrain → boundary → collision   │
    │                                                                      │
    │ ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                │
    │ │virtual_sim   │  │gazebo_bridge │  │robot_bridge  │                │
    │ │(50 Hz Python)│  │(3D physics)  │  │(UDP :9999)   │                │
    │ └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                │
    │        ▼                 ▼                 ▼                        │
    │ /robot_<id>/pose    Gazebo entities    ESP32 robots                 │
    └──────────────────────────────────────────────────────────────────────┘
             │
             │ pose → registry → /robots/status (loop closes)
             ▼
    ┌──────────────────────────────────────────────────────────────────────┐
    │ SENSING LAYER                                                        │
    │  sensor_engine_node ──► /robot_<id>/sensors/<name> (SensorReading)   │
    │  vision_node        ──► /tags/detections, MJPEG :8081                │
    │  arena_model_node   ──► /arena/model                                 │
    └──────────────────────────────────────────────────────────────────────┘
```

The arrows are typed ROS2 messages; this diagram is the public contract. Replace any node with a non-Python implementation that speaks the same messages and the rest of the platform doesn't notice.

## Layer architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│ LAYER 5: CLIENTS                                                      │
│   Web Dashboard, Python SDK, ATAK (CoT), REST clients,                │
│   any-language ROS2 nodes that speak the message catalog              │
├──────────────────────────────────────────────────────────────────────┤
│ LAYER 4: EXTERNAL INTERFACES                                          │
│   api_gateway_node   (REST + WebSocket :8080)                         │
│   cot_bridge_node    (CoT XML :6969 / :4242)                          │
├──────────────────────────────────────────────────────────────────────┤
│ LAYER 3: ALGORITHM MODULES (pure Python today, language-neutral       │
│            via ROS2 contracts; see message-catalog.md)                │
│   robot.py, robot_registry.py, command_pipeline.py, control_manager   │
│   arena_model.py, geometry.py, kinematics_model.py, sensor_engine     │
│   pose.py, geo_reference.py, spatial_field.py, terrain_pipeline.py    │
│   ghost_model_manager.py, replay.py                                   │
│                                                                       │
│   TACTICAL                                                            │
│   engagement.py, line_of_sight.py, weather.py, health.py              │
│   behavior.py, comms.py, logistics.py, roe.py                         │
│   world_state.py     (single source of truth, YAML loader)            │
│                                                                       │
│   PROTOCOLS                                                           │
│   cot_protocol.py, cot_transport.py, ws_protocol.py, udp_protocol.py  │
│                                                                       │
│   sensor_plugins/    gas, gps, lidar, sonar, fof,                     │
│                      thermal, rangefinder, ied_detector,              │
│                      df_receiver, uav_camera (digital twins)          │
├──────────────────────────────────────────────────────────────────────┤
│ LAYER 2: ROS2 NODES (thin shells)                                     │
│   registry_node, api_gateway_node, robot_bridge_node                  │
│   virtual_sim_node, sensor_engine_node, arena_model_node              │
│   vision_node, cot_bridge_node, gazebo_bridge_node, replay_node       │
│   world_state_node, opfor_node, engagement_node                       │
│   los_python_node, los_gazebo_node                                    │
├──────────────────────────────────────────────────────────────────────┤
│ LAYER 1: LOCALIZATION PROVIDERS (pluggable)                           │
│   AprilTagProvider, GazeboProvider                                    │
│   GpsImuProvider (future), RfAnchorProvider (future)                  │
├──────────────────────────────────────────────────────────────────────┤
│ LAYER 0: HARDWARE / SIMULATION                                        │
│   ESP32 robots, USB camera, Gazebo gz-sim, GPS, UWB anchors           │
└──────────────────────────────────────────────────────────────────────┘
```

## Where the polyglot boundary lives

The contract between modules is the catalog of typed ROS2 messages and services. Anything below the boundary can be re-implemented in any language with a ROS2 binding:

| Cross-node payload | ROS2 contract | Today's implementation |
|---|---|---|
| Robot state | `RobotStatus.msg`, `RobotStatusList.msg` | Python `registry_node` |
| Fire intent | `FireIntent.msg` on `/fire_weapon` | Python `opfor_node`, `api_gateway_node` |
| Engagement outcome | `EngagementOutcome.msg` on `/engagement_events` | Python `engagement_node` |
| Target observation | `Observation.msg` on `/report_observation` | Python `api_gateway_node` |
| Sensor reading | `SensorReading.msg` (envelope) | Python `sensor_engine_node` |
| World cover | `WorldCover.msg` on `/world/cover` (latched) | Python `world_state_node` |
| Civilians, IEDs, EW, jamming, dead zones | `*List.msg` on `/world/…` (latched) | Python `world_state_node` |
| Weather, ROE, weapons | `WeatherState`, `RoeRules`, `WeaponCatalog` (latched) | Python `world_state_node` |
| Apply damage to robot | `ApplyDamage.srv` | Python `registry_node` |
| Spawn / remove / reset / list robots | `Spawn/Remove/Reset/ListRobots.srv` | Python `registry_node` |
| Line-of-sight query | `EvaluateLos.srv` | `los_python_node` (2D) **or** `los_gazebo_node` (lidar) |

See [`message-catalog.md`](message-catalog.md) for field-level reference and worked examples in C++, MATLAB, and Rust.

## Core abstractions

| Module | Description |
|--------|-------------|
| `pose.py` | `RobotPose` + `PoseSource` enum (CAMERA_ARTAG, GPS_IMU, RF_ANCHOR, VIRTUAL_SIM, GAZEBO_SIM). Single interface between localization and platform. |
| `robot.py` | `Robot` dataclass: `deployment` (physical/virtual), `localization_id`, `localization_source`, `kinematics_model`, `team`, `vehicle_role`, `health`, `weapons`, `thermal_signature`, `logistics`, `sensor_preset`. Used everywhere. |
| `kinematics_model.py` | `KinematicsModel` protocol. `DifferentialDrive` + `Omnidirectional`. |
| `localization_provider.py` | `LocalizationProvider` protocol. Each backend produces `RobotPose` objects. |
| `spatial_field.py` | `SpatialField` protocol for virtual data layers. `GaussianPlumeField` (gas), `ElevationField` (terrain), `UniformField`, `CompositeField`. |
| `sensor_engine.py` | Per-robot sensor state, `EnvironmentContext` with spatial fields and `point_sources` (IEDs, EW emitters, civilians), Gazebo sensor bridge multiplexing. |
| `geo_reference.py` | WGS84 arena ↔ lat/lon with rotation and `scale_factor`. |
| `terrain_pipeline.py` | DEM → Gazebo heightmap PNG. Supports GeoTIFF, SRTM, numpy arrays. |
| `ghost_model_manager.py` | Spawns/updates/removes ghost models in Gazebo for physical robots at scaled positions. |
| `world_state.py` | `WorldState` dataclass + `world_state_from_config(yaml)`; the canonical world definition. |
| `command_pipeline.py` | Velocity filtering: state → mobility → fuel → speed → terrain → boundary → collision. |
| `engagement.py` | `evaluate_fire(actor, target, weapon, ...)` — PoK, range falloff, LoS, suppression. |
| `line_of_sight.py` | `has_line_of_sight` 2D + height-sample (matches `EvaluateLos.srv`). |
| `behavior.py` | `BehaviorTree` FSM for OPFOR. |
| `health.py`, `comms.py`, `logistics.py`, `roe.py`, `weather.py` | See [`tactical-capabilities.md`](tactical-capabilities.md). |

## ROS2 nodes

### Core platform

| Node | Description |
|------|-------------|
| `vision_node` | Camera → AprilTagProvider → `/tags/detections`, homography, MJPEG stream (port 8081). |
| `arena_model_node` | YAML-first boundary. Publishes `/arena/model`. Falls back to tag detection if no YAML. |
| `registry_node` | Robot lifecycle services (spawn/remove/reset/list/register/apply_damage). Publishes `/robots/status` at 10 Hz with the full tactical field set. |
| `robot_bridge_node` | ESP32 UDP (port 9999). REG → register, cmd_vel → UDP motor, heartbeat. |
| `virtual_sim_node` | 50 Hz kinematics with pluggable `KinematicsModel`. Lightweight simulation mode. |
| `gazebo_bridge_node` | Generates SDF world, launches gz-sim + ros_gz_bridge, manages ghost models, bridges sensors. |
| `replay_node` | Plays back GPX/YAML tracks as virtual robot positions at configurable speed. |
| `sensor_engine_node` | Multiplexes Python plugins and Gazebo sensor bridges. Publishes typed `SensorReading` envelope on `/robot_<id>/sensors/<name>`. |
| `api_gateway_node` | FastAPI REST + WS (port 8080). Command pipeline, control ownership, event injection, pose broadcast. Forwards WS `fire_weapon`/`report_observation` to typed `FireIntent`/`Observation` topics. |
| `cot_bridge_node` | `/robots/status` + `/arena/model` + `/engagement_events` + sensor topics → CoT XML → ATAK. Inbound waypoints from ATAK. |

### Tactical

| Node | Description |
|------|-------------|
| `world_state_node` | Loads exercise YAML once and publishes the world (cover, civilians, IEDs, EW, jamming, dead zones, weather, ROE, weapons) on latched `/world/*` topics. Single source of truth. |
| `opfor_node` | Spawns each OPFOR unit via `~/spawn_virtual`, runs `BehaviorTree.tick()` per unit at 10 Hz. Subscribes to `/world/cover`, `/world/weather`, `/world/weapons`. Publishes `cmd_vel` and typed `FireIntent` on `/fire_weapon`. |
| `engagement_node` | Subscribes to `/fire_weapon` (typed) and `/report_observation`. Runs `roe.check_fire_roe` + `engagement.evaluate_fire`, calls `~/apply_damage`, publishes typed `EngagementOutcome` on `/engagement_events`. Pulls cover, civilians, weather, ROE, weapons from `/world/*` topics. |
| `los_python_node` | Serves `EvaluateLos.srv` via the 2D Python ray model + cover polygons. |
| `los_gazebo_node` | Serves `EvaluateLos.srv` via a Gazebo lidar query. Falls back to `visible=true rationale=gazebo_unavailable` when Gazebo isn't present. |

## Operating modes

| Mode | Launch | What runs | Use case |
|------|--------|-----------|----------|
| `lightweight` | `mode:=lightweight` | `virtual_sim_node` (50 Hz Python kinematics) | Planning, rehearsal, CI, laptops |
| `gazebo` | `mode:=gazebo` | `gazebo_bridge_node` + `gz-sim` (flat arena, 3D physics) | 3D visualisation, demos |
| `gazebo_terrain` | `mode:=gazebo_terrain` | Gazebo with DEM heightmaps + sensor bridging + ghost models | Real terrain exercises |
| `replay` | `mode:=replay` | `replay_node` plays back recorded GPS tracks | Exercise replay and analysis |
| `physical` | Camera/GPS localisation providers | Real robots + virtual data layers | AR tactical exercises |

```bash
ros2 launch plato_pod simulation.launch.py mode:=lightweight
ros2 launch plato_pod simulation.launch.py mode:=gazebo_terrain \
  exercise_file:=/ros2_ws/config/exercises/adfa-terrain-patrol.yaml \
  scale_factor:=1000
ros2 launch plato_pod atak_test.launch.py \
  exercise_file:=/ros2_ws/config/exercises/dowsett-field-ctf.yaml \
  target_host:=192.168.1.42
```

All modes publish to identical ROS2 topics. All upstream code (registry, pipeline, gateway, dashboard, CoT) works unchanged.

## Tactical exercise launch sequence

A full tactical exercise needs the world owner, the engagement evaluator, the LoS service, and the OPFOR brain in addition to the core platform:

```bash
EX=/ros2_ws/config/exercises/cordon-and-search.yaml

ros2 launch plato_pod world_state.launch.py exercise_file:=$EX &
ros2 launch plato_pod simulation.launch.py mode:=lightweight \
  exercise_file:=$EX &
ros2 launch plato_pod los.launch.py backend:=python &
ros2 launch plato_pod engagement.launch.py exercise_file:=$EX &
ros2 launch plato_pod opfor.launch.py exercise_file:=$EX &
ros2 launch plato_pod cot_bridge.launch.py exercise_file:=$EX &
```

`world_state_node` should start first so its latched topics are available when consumers connect. Order doesn't otherwise matter — DDS discovery handles the rest.

## Gazebo integration

**World generation:** `gazebo_world_builder.py` generates SDF worlds from exercise YAML — walls, obstacles, zones, robots, terrain heightmaps. `terrain_pipeline.py` converts DEM data (GeoTIFF, SRTM) to Gazebo-compatible PNG heightmaps. Pure Python.

**Sensor bridging:** `gazebo_bridge_node.py` launches gz-sim and `ros_gz_bridge`, configures pose/cmd_vel/lidar/IMU topic bridging for each robot. GPU detection at startup selects gpu_lidar or cpu_lidar. `sensor_plugins/gazebo_bridge.py` passes Gazebo sensor data through the Python sensor engine in the same dict format as Python plugins — downstream code sees identical data regardless of source.

**Line-of-sight:** `los_gazebo_node` serves the same `EvaluateLos.srv` that `los_python_node` does, but resolves visibility against Gazebo's lidar geometry. Engagement and sensor consumers don't change when you swap backends.

**Ghost models:** When `scale_factor > 1.0`, `ghost_model_manager.py` spawns virtual counterparts of physical desktop robots in the Gazebo world at scaled positions. A robot at (0.3, 0.2) on a 0.84 m desk arena appears at (300, 200) in a 1× scale Gazebo terrain world.

**Gas simulation:** `sensor_plugins/gaden_bridge.py` bridges GADEN gas concentration data when available. Falls back to Python `GaussianPlumeField` + MOX ODE dynamics when GADEN is not running.

| Vehicle model | Dimensions | Mass | Max speed | Sensors |
|---------------|-----------|------|-----------|---------|
| **platopod** | 0.08 m dia, 0.03 m tall | 0.15 kg | — | lidar (2 m), IMU, contact |
| **tank** | 3.0 × 1.5 × 1.0 m | 50,000 kg | 15 m/s | lidar (500 m), IMU, contact |
| **apc** | 2.5 × 1.2 × 0.8 m | 15,000 kg | 25 m/s | lidar (200 m), IMU, contact |
| **recon** | 1.0 × 0.6 × 0.4 m | 500 kg | 40 m/s | lidar (100 m), IMU, contact |

## CoT/ATAK integration

The CoT bridge sends the complete tactical picture to ATAK military tablets and receives waypoints back.

| Data | CoT mechanism | Direction |
|------|---------------|-----------|
| Robot positions + heading | `<point>` + `<track>` with MIL-STD-2525B type codes | → ATAK |
| Sensor readings | `<sensor>` detail + `<remarks>` text | → ATAK |
| Engagement events | `b-r-f-h-c` alert with weapon/outcome remarks | → ATAK |
| Casualty markers | `a-h-G-X` (destroyed) / `a-f-G-X` (wounded) | → ATAK |
| IED hazards | `u-d-c-c` (CBRN drawing) | → ATAK |
| Civilians | `a-n-G` (neutral) | → ATAK |
| Arena boundary | Shape event polygon | → ATAK |
| Scoring zones | Semi-transparent team-coloured polygons | → ATAK |
| Obstacles | Grey polygons with labels | → ATAK |
| Waypoints / nav goals | Waypoint CoT events | ← ATAK |

**Components:** `geo_reference.py` (WGS84 with `scale_factor`), `cot_protocol.py` (XML gen/parse + helpers for engagement/casualty/IED/civilian markers), `cot_transport.py` (UDP/TCP), `cot_bridge_node.py` (subscribes to status/arena/sensors/engagement events, publishes to ATAK).

## Ports

| Port | Service | Protocol |
|------|---------|----------|
| 8080 | API gateway | HTTP REST + WebSocket + dashboard |
| 8081 | Vision MJPEG | HTTP stream + tag detections |
| 9999 | Robot bridge | UDP (ESP32 firmware) |
| 6969 | CoT bridge (out) | UDP/TCP to ATAK / FreeTAKServer |
| 4242 | CoT bridge (in) | UDP listener for ATAK waypoints |

## Client-side

| Component | Description |
|-----------|-------------|
| `web/static/index.html` | Dashboard: 2D map (default) + camera AR overlay (toggle) |
| `web/sdk/platopod/arena.py` | Python SDK: `Arena` class, sync API over async WebSocket |
| `web/sdk/platopod/keyboard.py` | KeyboardController: curses → cmd_vel at 10 Hz |
| Any-language ROS2 client | Speak the message catalog; see [`message-catalog.md`](message-catalog.md) |

## Test coverage

**838 unit tests** — all run without ROS2, Docker, or hardware. Covers domain logic, spatial fields, sensor plugins, gas dynamics, terrain pipeline, geo-reference, CoT protocol, replay interpolation, ghost-model management, the full tactical layer (engagement, health, LoS, weather, behaviour, comms, logistics, ROE), and `world_state` aggregation.

```bash
cd server
PYTHONPATH=src python3 -m pytest tests/unit/ -v
```

## Quick start

```bash
# Lightweight mode (no Gazebo, no camera)
ros2 launch plato_pod simulation.launch.py mode:=lightweight &
ros2 run plato_pod arena_model_node --ros-args \
  -p exercise_file:=/ros2_ws/config/exercises/dowsett-field-ctf.yaml &
ros2 run plato_pod registry_node &
ros2 run plato_pod sensor_engine_node &
ros2 run plato_pod api_gateway_node &
curl -X POST http://localhost:8080/robots/spawn \
  -H 'Content-Type: application/json' -d '{"x":0.4,"y":0.3}'
# Open http://localhost:8080/

# Full tactical exercise
EX=/ros2_ws/config/exercises/cordon-and-search.yaml
ros2 launch plato_pod world_state.launch.py exercise_file:=$EX &
ros2 launch plato_pod simulation.launch.py mode:=lightweight exercise_file:=$EX &
ros2 launch plato_pod los.launch.py backend:=python &
ros2 launch plato_pod engagement.launch.py exercise_file:=$EX &
ros2 launch plato_pod opfor.launch.py exercise_file:=$EX &
ros2 launch plato_pod cot_bridge.launch.py exercise_file:=$EX
```
