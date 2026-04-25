# Plato Pod

**An augmented reality tactical simulation platform for military training exercises.**

Plato Pod creates tactical training scenarios where physical and virtual elements coexist on the same arena. Small robots on a desktop represent real military vehicles moving through real geographic terrain. Virtual units, CBRN hazards, terrain effects, and engagement rules are layered on top. The result appears on ATAK military tablets as a standard tactical picture — operators cannot tell which units are physical and which are virtual.

The platform operates at any scale: a classroom desk, an indoor facility, or an outdoor training area with GPS-equipped units. The same exercise definition drives all scales.

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

## Key Capabilities

- **Scale-agnostic** — the same exercise runs on a desktop arena or on real terrain with GPS-equipped units. The platform handles the coordinate mapping.
- **Mixed reality** — physical robots interact with virtual units, CBRN hazards, terrain effects, and engagement rules on the same arena.
- **Real terrain** — load elevation data from real geographic regions into Gazebo for physics-accurate simulation and sensor generation.
- **ATAK interoperability** — all units appear as MIL-STD-2525B symbols on military tablets. Operators can send waypoints back from ATAK.
- **Physics-accurate sensors** — lidar, IMU, and gas sensors computed by Gazebo's physics engine, with Python fallbacks when Gazebo is not running.
- **Exercise replay** — record GPS tracks from real field exercises, replay them in the classroom on the same terrain.
- **Programmable** — Python SDK for autonomous robot control. Students write algorithms; the platform handles localisation, sensors, and safety.

## Repository Structure

```
platopod/
├── firmware/              # ESP32-C3 robot firmware (ESP-IDF, UDP protocol)
├── server/                # ROS2 server nodes (Python 3.12)
│   ├── src/plato_pod/     #   node implementations + domain logic
│   ├── launch/            #   ROS2 launch files
│   ├── config/            #   arena and sensor configuration
│   └── tests/             #   unit tests
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

# Physical robots on a desk (camera required)
ros2 launch plato_pod classroom.launch.py camera_device:=4 \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml

# Virtual only (no camera, no robots)
ros2 launch plato_pod atak_test.launch.py \
  exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml

# Gazebo with real terrain
ros2 launch plato_pod simulation.launch.py mode:=gazebo_terrain \
  exercise_file:=/ros2_ws/config/exercises/adfa-terrain-patrol.yaml

# Dashboard: http://localhost:8080/
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

| Scenario | Description |
|----------|-------------|
| **Capture the Flag** | Two teams compete to reach opponent's flag zone. Boundary enforcement, tag-freeze rules. |
| **Gas Plume Search** | Locate a virtual CBRN source using gas sensors. Gaussian plume dispersion with MOX sensor dynamics. |
| **ADFA Terrain Patrol** | Patrol real ADFA campus terrain. SRTM elevation data, CBRN hazard, ATAK tactical picture. |

See `server/README.md` for full scenario documentation, ATAK integration guide, and classroom setup instructions.

## Team

- **Artem Lensky** — Project lead, UNSW Canberra

## License

MIT License — see [LICENSE](LICENSE) for details.

## Acknowledgements

Developed at the School of Engineering and Technology, UNSW Canberra / Australian Defence Force Academy.
