# Server

ROS2 Jazzy server nodes for the Plato Pod platform.

## Nodes

- **vision_node** — Overhead camera capture, AprilTag detection, robot pose estimation
- **sensor_sim_node** — Simulated proximity, line, and obstacle sensors based on arena map and robot poses
- **api_gateway** — WebSocket + REST API for student access to robot control and sensor data
- **robot_bridge_node** — UDP communication with physical ESP32 robots (translates ROS2 cmd_vel to UDP motor commands)

Note: micro-ROS agent integration is planned as future work. Currently, physical robots communicate via plain UDP.

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

## Running the robot bridge (for physical ESP32 robots)

```bash
ros2 run plato_pod robot_bridge_node
```

## Configuration

- `config/arena.yaml` — Arena dimensions, obstacle positions
- `config/apriltag_settings.yaml` — Tag family, size, camera parameters
