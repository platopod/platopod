# Docker Development Environment

## Quick Start

```bash
cd docker
docker compose build          # build the image (first time only)
docker compose up -d          # start container in background
docker compose exec ros bash  # open a shell inside the container
```

## What's Inside the Container

- Ubuntu 24.04 + ROS2 Jazzy Desktop
- micro-ROS agent
- Python dependencies (Shapely, NumPy, FastAPI, pytest)
- hostapd + dnsmasq for WiFi AP
- OpenCV (headless) for camera processing

## Device Access

### Camera

The Logitech C920 is passed through as `/dev/video0`. Verify inside the container:

```bash
v4l2-ctl --list-devices
ffplay /dev/video0   # quick test (if X11 forwarding enabled)
```

If your camera is on a different device number, edit `docker-compose.yaml`:
```yaml
devices:
  - /dev/video2:/dev/video0
```

### WiFi Dongle (MT7612U)

The container runs with `--privileged` and `--net=host`, giving it full access to the host's network interfaces including USB WiFi dongles.

To set up the WiFi AP for ESP32 robots:

```bash
# Inside the container
sudo ./docker/setup_wifi_ap.sh wlan1
```

Verify the interface name first:
```bash
iw dev
```

## Development Workflow

### Build and test ROS2 nodes

```bash
# Inside the container
cd /ros2_ws
colcon build
source install/setup.bash
colcon test
```

### Run the server

```bash
# Inside the container
ros2 launch plato_pod server.launch.py
```

### Run the robot bridge (for physical ESP32 robots)

```bash
# Inside the container
ros2 run plato_pod robot_bridge_node
```

### Access from your host

Since the container uses `--net=host`, all ports are directly accessible:

- **Web dashboard:** http://localhost:8080
- **REST API:** http://localhost:8080/robots
- **WebSocket:** ws://localhost:8080/api/control

Python SDK scripts run on your host (outside Docker) and connect to localhost:

```python
from platopod import Arena
arena = Arena("ws://localhost:8080/api/control")
```

## RViz2

To use RViz2 for visual debugging, you need X11 forwarding:

```bash
# On your host, allow X connections
xhost +local:docker

# The DISPLAY variable is already passed through in docker-compose.yaml
# Inside the container:
rviz2
```

## Stopping

```bash
docker compose down           # stop and remove container
docker compose down -v        # also remove build cache volumes
```

## Rebuilding

After changing the Dockerfile:
```bash
docker compose build --no-cache
docker compose up -d
```

After changing only Python code — no rebuild needed. The source directory is mounted as a volume. Just re-run `colcon build` inside the container.
