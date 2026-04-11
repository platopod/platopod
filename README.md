# Plato Pod

**An open-source educational robotics platform for teaching programming, multi-robot coordination, and wargaming through hands-on robot control.**

Plato Pod is a classroom-ready robotics platform combining physical and virtual robots on a shared arena. An overhead camera tracks physical robots via AprilTag markers, while virtual robots are simulated server-side with identical kinematics. A unified API lets students control both types identically — writing Python scripts for obstacle avoidance, formation control, sensor fusion, and team-based wargaming exercises.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                  ROS2 Server (Raspberry Pi 5)                │
│                                                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │
│  │  Vision   │  │  Sensor  │  │ Virtual  │  │  Exercise   │  │
│  │   Node    │  │  Engine  │  │   Sim    │  │  Manager    │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────┬──────┘  │
│       │              │             │               │         │
│  ┌────┴──────────────┴─────────────┴───────────────┴──────┐  │
│  │                    Arena Model                         │  │
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
        │  Python   │ │   Web   │ │  Projector  │
        │   SDK     │ │Dashboard│ │   AR View   │
        └───────────┘ └─────────┘ └─────────────┘
```

## Key Features

- **Physical + virtual robots** on the same arena with unified control API
- **Simulated sensors** — Lidar, Sonar, GPS, Friend-or-Foe — computed server-side via ray casting
- **Dynamic arena** — boundary defined by AprilTags, obstacles and zones configurable mid-session
- **Exercise engine** — structured sessions with teams, scoring, game effects (tag, freeze, capture)
- **AR dashboard** — live camera feed with IPM rectification and sensor visualisation overlay
- **Projector AR** — virtual elements projected onto the physical arena surface
- **Python SDK** — clean API for students, with future C/MATLAB bindings planned

## Repository Structure

```
platopod/
├── firmware/              # ESP32-C3 robot firmware (ESP-IDF + micro-ROS)
│   ├── main/              #   application source
│   └── components/        #   micro-ROS ESP-IDF component
├── server/                # ROS2 server nodes
│   ├── src/               #   node implementations
│   ├── launch/            #   ROS2 launch files
│   ├── config/            #   arena and sensor configuration
│   ├── plugins/
│   │   ├── sensors/       #   sensor plugins (Lidar, Sonar, GPS, FoF)
│   │   └── scoring/       #   scoring plugins
│   └── recordings/        #   exercise replay data
├── hardware/              # PCB designs (KiCad) and 3D-printable chassis
│   ├── pcb/
│   ├── chassis/
│   └── bom/
├── web/                   # Web dashboard (HTML/JS)
│   ├── src/
│   └── static/
├── config/
│   └── exercises/         # Exercise YAML templates
└── docs/
    ├── issues/            # Issue specifications
    └── shopping-lists/    # Component procurement lists
```

## Hardware

**Robot (Plato Pod):**
- ESP32-C3 SuperMini (22.5×18mm, RISC-V, WiFi, UDP protocol)
- DRV8833 dual motor driver + 2× N20 gear motors
- SSD1306 OLED display (128×64, I2C)
- WS2812B RGB LED for status
- 3.7V 500mAh LiPo + TP4056 USB-C charger
- 55mm circular chassis with AprilTag marker

**Server:**
- Raspberry Pi 5 (2GB) running Ubuntu 24.04 + ROS2 Jazzy
- OV2710 USB camera (1080p, 100° FOV, overhead mount)
- MT7612U USB WiFi (5GHz AP for students)
- Optional: UMBOLITE HY320 mini projector for arena AR projection

## Quick Start

### Prerequisites

- [ESP-IDF v5.2+](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/)
- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
- Python 3.12+
- Shapely ≥2.0, NumPy

### Student (Python SDK)

```python
from platopod import Arena

arena = Arena("ws://192.168.4.1:8080/api/control")
robots = arena.list_robots()

arena.subscribe_sensors(robot_id=1, sensors=["gps", "lidar_2d"], rate_hz=10)

while True:
    gps = arena.get_sensor(robot_id=1, sensor="gps")
    scan = arena.get_sensor(robot_id=1, sensor="lidar_2d")

    if scan and min(scan["ranges"]) < 0.1:
        arena.cmd_vel(robot_id=1, linear_x=0.0, angular_z=0.5)
    else:
        arena.cmd_vel(robot_id=1, linear_x=0.15, angular_z=0.0)

    arena.sleep(0.1)
```

### Instructor

```python
from platopod import AdminClient

admin = AdminClient("http://192.168.4.1:8080", token="admin-secret")
admin.load_exercise("capture-the-flag.yaml")
admin.start()
```

## Issue Tracker

Development is tracked via 10 core issues. See [`docs/issues/`](docs/issues/) for full specifications.

| # | Issue | Description |
|---|-------|-------------|
| 0 | Vision Node | Camera capture, AprilTag detection, pose estimation, MJPEG stream |
| 1 | Arena Boundary and Model | Dynamic boundary, obstacles, zones, coordinate system |
| 2 | Robot Spawning and Discovery | Physical/virtual robot registry, AprilTag matching |
| 3 | Unified Robot Control API | WebSocket commands, collision filtering, Python SDK |
| 4 | Virtual Robot Simulation | Server-side differential drive kinematics |
| 5 | Web Dashboard Phase 1 | Live camera + AR overlay |
| 6 | Unified Sensor API | Sensor subscription, streaming, team sharing |
| 7 | Exercise and Team Management | Admin, teams, scoring, game effects |
| 8 | Range Sensors | Sensor engine, ray caster, Lidar, Sonar plugins |
| 9 | Position/ID Sensors | GPS, Friend-or-Foe plugins |
| 10 | Web Dashboard Phase 2 | IPM, sensor viz, interactive control, projector AR |

## Team

- **Artem Lensky** — Project lead, UNSW Canberra
- **Rhi-Anne Chng** — Hardware and firmware (FYP student)
- **Haoren** — Server software and vision pipeline (Research Assistant)

## License

MIT License — see [LICENSE](LICENSE) for details.

## Acknowledgements

Developed at the School of Engineering and Technology, UNSW Canberra / Australian Defence Force Academy.
