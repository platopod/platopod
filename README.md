# Plato Pod

**An augmented reality tactical simulation platform for military training exercises.**

Plato Pod creates tactical training scenarios where physical and virtual elements coexist on the same arena. Small robots on a desktop represent real military vehicles moving through real geographic terrain. Virtual units, CBRN hazards, weapon engagements, casualties, communications failures, logistics, civilian populations, and rules of engagement are layered on top. The result appears on ATAK military tablets as a standard tactical picture — operators cannot tell which units are physical and which are virtual.

The platform operates at any scale: a classroom desk, an indoor facility, or an outdoor training area with GPS-equipped units. The same exercise definition drives all scales.

## Architecture at a glance

```
                        Exercise YAML  (config/exercises/*.yaml)
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────┐
│  ROS2 server — message-driven, language-neutral                    │
│                                                                    │
│  world_state_node ─►  /world/cover, /world/civilians, /world/roe,  │
│                       /world/weapons, /world/weather, … (latched)  │
│                                                                    │
│  registry_node    ─►  /robots/status (pose, health, team, …)       │
│                       services: spawn_virtual, apply_damage, …     │
│                                                                    │
│  opfor_node       ─►  FireIntent, cmd_vel  (autonomous OPFOR FSM)  │
│  api_gateway_node ─►  FireIntent, Observation  (player-initiated)  │
│                                                                    │
│  engagement_node  ─►  EngagementOutcome  (PoK, ROE, LoS, damage)   │
│  sensor_engine    ─►  SensorReading      (typed envelope)          │
│  los_python_node  │                                                │
│  los_gazebo_node  ┴►  EvaluateLos service (interchangeable)        │
│                                                                    │
│  cot_bridge_node  ─►  Cursor on Target XML to ATAK / FreeTAKServer │
│  gazebo_bridge    ─►  3D physics, terrain, sensor ray casts         │
│  vision_node      ─►  AprilTag detection (indoor localisation)      │
└────────────────────────────────────────────────────────────────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              ▼                     ▼                     ▼
        Web Dashboard          ATAK / WinTAK         ESP32 robots
        (operator UI)          (tactical map)        (UDP, LAN)
```

The boundaries between modules are typed ROS2 messages and services — see [`docs/message-catalog.md`](docs/message-catalog.md). Any language with a ROS2 binding (Python, C++, MATLAB, Rust, Java) can implement or replace a node.

## Key capabilities

- **Mixed reality** — physical robots interact with virtual units, CBRN hazards, terrain effects, and engagement rules on the same arena.
- **Scale-agnostic** — the same exercise runs on a desktop arena (AprilTag-tracked) or on real terrain with GPS-equipped units (`scale_factor` in the exercise YAML maps between them).
- **Real terrain** — load DEM elevation data from real geographic regions into Gazebo for physics-accurate simulation, line-of-sight, and sensor generation.
- **Tactical engagement modelling** — weapons with PoK and range falloff, line-of-sight against cover and terrain, weather attenuation, casualties with mobility/fire degradation, suppression, friendly-fire detection, and full ROE rules (weapons hold/tight/free + civilian proximity).
- **Autonomous OPFOR** — finite-state-machine virtual hostiles patrol, observe, engage, and retreat against blue-team players.
- **Sensor digital twins** — gas/MOX, thermal, laser rangefinder, IED detector, RF direction finder, UAV camera, GPS, lidar — with both Gazebo-bridged and Python-fallback backends behind the same message contract.
- **Communications & logistics** — comms degradation in dead zones and jamming, per-unit fuel/ammo/water, resupply via admin event injection.
- **ATAK interoperability** — every unit, sensor reading, engagement event, and casualty appears on military tablets as MIL-STD-2525B symbols. Operators send waypoints back from ATAK.
- **Polyglot by design** — typed ROS2 messages are the public contract. Replace any node with a C++/MATLAB/Rust implementation without touching the rest of the stack.
- **Programmable** — Python SDK for autonomous robot control, plus the message catalog for any other language.

## Relationship to TAK

TAK (Team Awareness Kit) — ATAK, WinTAK, TAK VR — is a situational-awareness and C2 display system used by the ADF and allied forces. It shows positions on a map and shares data between operators. **TAK does not simulate anything.**

Plato Pod is the simulation engine that creates the tactical situation and feeds it to TAK. It generates the world: robot physics, virtual units, CBRN hazards, terrain interaction, sensor data, weapon engagements, casualty effects, exercise rules, and scoring. The combined picture — physical and virtual — is sent to TAK via the Cursor on Target (CoT) protocol, where it appears as a standard tactical display indistinguishable from a real operation.

```
Plato Pod (creates the training battlespace)
    │
    │  CoT XML: unit positions, sensor readings, engagements, casualties, hazards
    │
    ▼
FreeTAKServer (CoT relay, optional)
    │
    ├──► ATAK on tablets    (2D tactical map)
    ├──► WinTAK on laptops  (2D tactical map)
    └──► TAK VR in headsets (3D immersive view)
```

TAK can display a real operation or replay recorded data, but it cannot create a mixed-reality training exercise where physical robots interact with virtual threats on real terrain with engagement rules and scored objectives. That is what Plato Pod does — and it outputs to the same TAK ecosystem operators use in real operations.

**Tested clients:** WinTAK (Windows / Linux via CrossOver), ATAK-CIV (Android), iTAK (iOS). See [`docs/tak-setup.md`](docs/tak-setup.md) for setup instructions including FreeTAKServer deployment.

## Repository structure

```
platopod/
├── firmware/                    # ESP32-C3 robot firmware (ESP-IDF, UDP)
├── server/                      # ROS2 server (Python 3.12)
│   ├── plato_pod_msgs/          #   ROS2 message + service catalog (the public contract)
│   │   ├── msg/                 #     RobotStatus, FireIntent, EngagementOutcome,
│   │   │                        #     SensorReading, WorldCover, WeatherState, …
│   │   └── srv/                 #     SpawnVirtual, ApplyDamage, EvaluateLos, …
│   ├── src/plato_pod/           #   ROS2 nodes + Layer 3 algorithm modules
│   ├── launch/                  #   ROS2 launch files
│   ├── config/                  #   arena and sensor configuration
│   └── tests/                   #   pytest unit tests (838 + 1 skipped)
├── models/                      # Gazebo SDF vehicle models
├── hardware/                    # PCB designs (KiCad) + 3D-printable chassis
├── web/                         # Web dashboard + Python SDK
├── config/exercises/            # Exercise YAML scenarios
├── tools/                       # Standalone development utilities (CoT viewer, …)
└── docs/                        # Documentation
    ├── ARCHITECTURE.md          #   Layer architecture, design principles
    ├── message-catalog.md       #   The polyglot contract (msgs + srvs)
    ├── tactical-capabilities.md #   Engagement, casualty, comms, ROE, behaviour
    ├── scenario-authoring.md    #   How to write a new exercise YAML
    ├── setup-guide.md           #   Docker + ROS2 install
    └── tak-setup.md             #   FreeTAKServer + WinTAK setup
```

## Hardware

**Robot (Plato Pod):**
- ESP32-C3 SuperMini (22.5×18 mm, RISC-V, WiFi, UDP protocol)
- DRV8833 dual motor driver + 2× N20 gear motors
- SSD1306 OLED display (128×64, I2C)
- WS2812B RGB LED for status
- 3.7 V 500 mAh LiPo + TP4056 USB-C charger
- 55 mm circular chassis with AprilTag marker

**Server:**
- NVIDIA Jetson, mini PC with GPU, or Raspberry Pi 5
- USB camera (optional, for AprilTag localisation)
- WiFi AP for robot communication

## Quick start

```bash
# Docker (recommended)
cd docker && docker compose up -d && docker compose exec ros bash

# Inside container
cd /ros2_ws && colcon build && source install/setup.bash

# Tactical exercise on the desk (camera required)
ros2 launch plato_pod world_state.launch.py \
  exercise_file:=/ros2_ws/config/exercises/cordon-and-search.yaml &
ros2 launch plato_pod classroom.launch.py camera_device:=4 \
  exercise_file:=/ros2_ws/config/exercises/cordon-and-search.yaml &
ros2 launch plato_pod opfor.launch.py \
  exercise_file:=/ros2_ws/config/exercises/cordon-and-search.yaml &
ros2 launch plato_pod engagement.launch.py \
  exercise_file:=/ros2_ws/config/exercises/cordon-and-search.yaml &
ros2 launch plato_pod los.launch.py backend:=python &

# Virtual only (no camera, no robots) — for ATAK demos
ros2 launch plato_pod atak_test.launch.py \
  exercise_file:=/ros2_ws/config/exercises/dowsett-field-ctf.yaml

# Gazebo with real terrain
ros2 launch plato_pod simulation.launch.py mode:=gazebo_terrain \
  exercise_file:=/ros2_ws/config/exercises/adfa-terrain-patrol.yaml

# Dashboard: http://localhost:8080/
```

### Python SDK example

```python
from platopod import Arena

arena = Arena("ws://localhost:8080/api/control")
arena.subscribe_sensors(robot_id=1, sensors=["gps", "gas"], rate_hz=10)

while True:
    gas = arena.get_sensor(robot_id=1, sensor="gas")
    if gas and gas["concentration"] > 50:
        arena.cmd_vel(robot_id=1, linear_x=0.0, angular_z=0.5)  # turn away
    else:
        arena.cmd_vel(robot_id=1, linear_x=0.15, angular_z=0.0)  # advance
    arena.sleep(0.1)
```

A non-Python equivalent — using `rclcpp`, MATLAB ROS Toolbox, `r2r`, or `rcljava` — works the same way; subscribe/publish on the same topics. See [`docs/message-catalog.md`](docs/message-catalog.md) for worked C++/MATLAB/Rust examples.

## Exercise scenarios

| Scenario | Description |
|----------|-------------|
| **Capture the Flag** | Two teams compete to reach opponent's flag zone. Boundary enforcement, tag-freeze rules. |
| **Gas Plume Search** | Locate a virtual CBRN source using gas sensors. Gaussian plume + MOX dynamics. |
| **Dowsett Field CTF** | CTF mapped to real Dowsett Field at ADFA, per-robot vehicle roles for ATAK. |
| **ADFA Terrain Patrol** | Patrol real ADFA campus terrain. SRTM elevation data, CBRN hazard. |
| **CBRN Recon Patrol** | Identify a chemical release source under threat from hostile recon. Gas + thermal + engagement. |
| **Defensive Position** | Hold a perimeter against multi-wave OPFOR with indirect fire and resupply logistics. |
| **Cordon and Search** | Surround a building cluster, find IEDs, manage civilians under tight ROE. |
| **Joint Coordination** | Multi-platoon coordination with UAV ISR, CAS, GPS jamming, EW emitters. |

See [`docs/scenario-authoring.md`](docs/scenario-authoring.md) for how to write your own.

## Documentation

- **[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)** — layer architecture, design principles, ROS2 nodes
- **[`docs/message-catalog.md`](docs/message-catalog.md)** — the polyglot contract: every message and service, with C++/MATLAB/Rust examples
- **[`docs/tactical-capabilities.md`](docs/tactical-capabilities.md)** — engagement, casualty, comms, logistics, ROE, OPFOR behaviour
- **[`docs/scenario-authoring.md`](docs/scenario-authoring.md)** — writing a new exercise YAML
- **[`docs/setup-guide.md`](docs/setup-guide.md)** — development environment setup
- **[`docs/tak-setup.md`](docs/tak-setup.md)** — FreeTAKServer + WinTAK on Linux

## Team

- **Artem Lensky** — project lead, UNSW Canberra

## License

MIT License — see [LICENSE](LICENSE) for details.

## Acknowledgements

Developed at the School of Engineering and Technology, UNSW Canberra / Australian Defence Force Academy.
