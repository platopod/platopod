# Plato Pod

**An augmented reality tactical simulation platform for military training exercises.**

Plato Pod operates at multiple physical scales — from desktop arenas with small differential-drive robots localised by AprilTag cameras, to outdoor exercises with GPS/IMU-equipped units. Physical and virtual elements coexist on the same arena: real robots interact with virtual units, gas plumes, terrain overlays, and engagement rules to create tactical training scenarios.

Four simulation modes: **lightweight** (Python kinematics), **Gazebo** (3D physics), **Gazebo terrain** (real DEM heightmaps + sensor bridging), or **replay** (GPS track playback). Switch with one launch argument.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│              ROS2 Server (Jetson / Mini PC / Pi 5)           │
│                                                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │
│  │  Vision   │  │  Sensor  │  │ Gazebo   │  │    CoT      │  │
│  │   Node    │  │  Engine  │  │ Bridge   │  │   Bridge    │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────┬──────┘  │
│       │              │             │               │         │
│  ┌────┴──────────────┴─────────────┴───────────────┴──────┐  │
│  │          Arena Model + Registry + Ghost Models         │  │
│  └────────────────────────┬───────────────────────────────┘  │
│                           │                                  │
│  ┌────────────────────────┴───────────────────────────────┐  │
│  │              API Gateway (WebSocket + REST)             │  │
│  └────────────────────────┬───────────────────────────────┘  │
└───────────────────────────┼──────────────────────────────────┘
                            │
              ┌─────────────┼─────────────┐
              │             │             │
        ┌─────┴─────┐ ┌────┴────┐ ┌──────┴──────┐
        │  Python   │ │   Web   │ │    ATAK     │
        │   SDK     │ │Dashboard│ │ (tactical)  │
        └───────────┘ └─────────┘ └─────────────┘
```

## Key Features

- **Multi-scale** — desktop arenas (AprilTag camera), indoor (RF anchors), outdoor (GPS/IMU)
- **Mixed reality** — physical and virtual units coexist with virtual gas plumes, terrain, enemies
- **Real terrain** — DEM heightmap pipeline loads real geographic data into Gazebo
- **ATAK integration** — MIL-STD-2525B symbols on military tablets, bidirectional waypoints
- **Gazebo sensor bridge** — physics-accurate lidar, IMU from Gazebo (GPU or CPU fallback)
- **GADEN gas simulation** — filament-based gas dispersion with MOX sensor digital twin
- **Ghost models** — desktop robots get scaled virtual counterparts in Gazebo terrain
- **Exercise replay** — load GPS tracks from prior exercises, replay on real terrain
- **Scale factor** — 0.84m desktop arena represents 840m of real terrain at 1000:1
- **4 vehicle types** — platopod, tank, APC, recon with MIL-STD type codes
- **Python SDK** — synchronous API for autonomous robot programming
- **618 unit tests** — all run without ROS2 or hardware

## Repository Structure

```
platopod/
├── firmware/              # ESP32-C3 robot firmware (ESP-IDF, UDP protocol)
├── server/                # ROS2 server nodes (Python 3.12)
│   ├── src/plato_pod/     #   node implementations + domain logic
│   ├── launch/            #   ROS2 launch files
│   ├── config/            #   arena and sensor configuration
│   └── tests/             #   618 unit tests
├── models/                # Gazebo SDF vehicle models (platopod, tank, APC, recon)
├── hardware/              # PCB designs (KiCad) and 3D-printable chassis
├── web/                   # Web dashboard (HTML/JS) + Python SDK
├── config/
│   └── exercises/         # Exercise YAML templates
└── docs/                  # Architecture documentation
```

## Hardware

**Robot (Plato Pod):**
- ESP32-C3 SuperMini (22.5x18mm, RISC-V, WiFi, UDP protocol)
- DRV8833 dual motor driver + 2x N20 gear motors
- SSD1306 OLED display (128x64, I2C)
- WS2812B RGB LED for status
- 3.7V 500mAh LiPo + TP4056 USB-C charger
- 55mm circular chassis with AprilTag marker

**Server:**
- NVIDIA Jetson, mini PC with GPU, or Raspberry Pi 5
- USB camera (optional, for AprilTag localisation)
- WiFi AP for robot communication

## Quick Start

```bash
# Docker (recommended)
cd docker && docker compose up -d && docker compose exec ros bash

# Inside container
cd /ros2_ws && colcon build && source install/setup.bash

# Lightweight mode (no Gazebo, no camera)
ros2 launch plato_pod atak_test.launch.py \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml \
  target_host:=192.168.1.42

# Gazebo with real terrain
ros2 launch plato_pod simulation.launch.py mode:=gazebo_terrain \
  exercise_file:=/ros2_ws/config/exercises/adfa-terrain-patrol.yaml \
  scale_factor:=1000
```

### Python SDK

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

## Exercise Scenarios

| Scenario | Description | Virtual layers |
|----------|-------------|----------------|
| **Capture the Flag** | Two teams compete to reach opponent's flag zone | Boundary enforcement, tag-freeze rules |
| **Gas Plume Search** | Locate a virtual gas source using MOX sensors | Gaussian plume, MOX sensor dynamics |
| **ADFA Terrain Patrol** | Patrol real ADFA campus terrain at 1000:1 scale | SRTM heightmap, CBRN hazard, ATAK output |

## Team

- **Artem Lensky** — Project lead, UNSW Canberra

## License

MIT License — see [LICENSE](LICENSE) for details.

## Acknowledgements

Developed at the School of Engineering and Technology, UNSW Canberra / Australian Defence Force Academy.
