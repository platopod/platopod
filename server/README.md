# Server

ROS2 Jazzy server nodes for the Plato Pod platform.

## Nodes

- **vision_node** — Overhead camera capture, AprilTag detection, robot pose estimation
- **sensor_sim_node** — Simulated proximity, line, and obstacle sensors based on arena map and robot poses
- **api_gateway** — WebSocket API for student access to robot control and sensor data
- **micro-ROS agent** — Bridges micro-ROS UDP traffic from ESP32 robots into the ROS2 network

## Setup

```bash
# Install ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# Install micro-ROS agent
sudo snap install micro-ros-agent

# Build the workspace
cd server
colcon build
source install/setup.bash
```

## Launch

```bash
ros2 launch plato_pod server.launch.py
```

## Configuration

- `config/arena.yaml` — Arena dimensions, obstacle positions
- `config/apriltag_settings.yaml` — Tag family, size, camera parameters
