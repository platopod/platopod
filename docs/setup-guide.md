# Plato Pod — Development Setup Guide

Complete step-by-step instructions to go from zero to a running development environment with Docker and ROS2.

## Prerequisites

- Ubuntu 24.04 (native or WSL2)
- Git installed (`sudo apt install git`)
- Docker installed (see Step 1 if not)
- GitHub account with access to https://github.com/platopod/platopod

---

## Step 1: Install Docker (skip if already installed)

```bash
# Remove old versions
sudo apt remove docker docker-engine docker.io containerd runc 2>/dev/null

# Install Docker
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add your user to the docker group (avoids needing sudo)
sudo usermod -aG docker $USER

# Log out and back in for the group change to take effect, then verify:
docker run hello-world
```

---

## Step 2: Clone and build

```bash
git clone https://github.com/platopod/platopod.git
cd platopod/docker

# Build the image (takes 5-10 minutes the first time)
docker compose build

# Start the container in the background
docker compose up -d

# Open a shell inside the container
docker compose exec ros bash

# Inside the container — build and test
cd /ros2_ws
colcon build
source install/setup.bash
python3 -m pytest src/plato_pod/tests/ -v
```

---

## Step 3: Verify the environment

**ROS2 and Python dependencies:**

```bash
# Inside the container
ros2 --help
python3 -c "import shapely; import numpy; import fastapi; print('All dependencies OK')"
```

**Camera access (if camera is plugged in):**

```bash
# Inside the container
v4l2-ctl --list-devices

# Quick capture test
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print(f'Camera OK: {frame.shape}' if ret else 'Camera FAILED')
cap.release()
"
```

**WiFi dongle (if MT7612U is plugged in):**

```bash
# Inside the container
iw dev
# You should see wlan1 (or similar) for the USB dongle
```

---

## Step 4: Run an exercise

### Virtual only (no camera, no robots)

```bash
# Inside the container
ros2 launch plato_pod atak_test.launch.py \
  exercise_file:=/ros2_ws/config/exercises/dowsett-field-ctf.yaml

# In another terminal, spawn virtual robots
curl -X POST http://localhost:8080/robots/spawn \
  -H 'Content-Type: application/json' -d '{"x":0.4,"y":0.3}'

# Open dashboard: http://localhost:8080/
```

### Physical robots on desk arena

```bash
# Inside the container (camera required)
ros2 launch plato_pod classroom.launch.py \
  camera_device:=4 \
  exercise_file:=/ros2_ws/config/exercises/dowsett-field-ctf.yaml

# Verify camera: http://localhost:8081/camera/stream/debug
# Dashboard: http://localhost:8080/
```

### Gazebo with real terrain

```bash
# Inside the container (GPU recommended)
ros2 launch plato_pod simulation.launch.py mode:=gazebo_terrain \
  exercise_file:=/ros2_ws/config/exercises/adfa-terrain-patrol.yaml \
  scale_factor:=1000
```

### Full tactical exercise (engagement + OPFOR + ROE + casualties)

`world_state_node` should start first so its latched topics are available when consumers connect:

```bash
# Inside the container
EX=/ros2_ws/config/exercises/cordon-and-search.yaml

ros2 launch plato_pod world_state.launch.py exercise_file:=$EX &
ros2 launch plato_pod simulation.launch.py mode:=lightweight exercise_file:=$EX &
ros2 launch plato_pod los.launch.py backend:=python &
ros2 launch plato_pod engagement.launch.py exercise_file:=$EX &
ros2 launch plato_pod opfor.launch.py exercise_file:=$EX &
ros2 launch plato_pod cot_bridge.launch.py exercise_file:=$EX &

# Verify the world is published
ros2 topic echo /world/cover --once
ros2 topic echo /world/roe --once

# Verify engagement events flow (then trigger one from the dashboard)
ros2 topic echo /engagement_events
```

See [`tactical-capabilities.md`](tactical-capabilities.md) for the tactical layer overview, [`scenario-authoring.md`](scenario-authoring.md) for writing your own exercises, and [`message-catalog.md`](message-catalog.md) for the polyglot ROS2 contract.

---

## Daily Development Workflow

### Starting a session

```bash
# Terminal 1: Start Docker (if not already running)
cd platopod/docker
docker compose up -d

# Terminal 2: Open a shell inside Docker for building/testing
docker compose exec ros bash

# Inside Docker
cd /ros2_ws && colcon build && source install/setup.bash
```

### Building and testing

```bash
# Inside Docker
cd /ros2_ws

# Build
colcon build
source install/setup.bash

# Run all unit tests (fast, no ROS2 nodes needed)
python3 -m pytest src/plato_pod/tests/ -v

# Run a specific test file
python3 -m pytest src/plato_pod/tests/unit/test_spatial_field.py -v

# Launch a specific node for manual testing
ros2 run plato_pod arena_model_node --ros-args \
  -p exercise_file:=/ros2_ws/config/exercises/capture-the-flag.yaml
```

### Python SDK (outside Docker)

```bash
cd platopod/web/sdk
pip install -e .

python3 -c "
from platopod import Arena
arena = Arena('ws://localhost:8080/api/control')
robots = arena.list_robots()
print(f'Found {len(robots)} robots')
arena.close()
"
```

---

## Connecting Physical ESP32 Robots

### 1. Set up WiFi AP

```bash
# Inside Docker
sudo bash /ros2_ws/docker/setup_wifi_ap.sh wlan1
```

### 2. Flash an ESP32

```bash
# On your host (ESP-IDF must be installed natively, not in Docker)
cd platopod/firmware
idf.py set-target esp32c3
idf.py menuconfig   # set WiFi SSID, password, server IP
idf.py build
idf.py flash monitor
```

### 3. Verify connection

```bash
# Inside Docker — the robot should register via UDP
curl http://localhost:8080/robots
# Should show the robot with "type": "physical"
```

---

## Troubleshooting

### Docker container won't start
```bash
docker compose logs ros
docker compose build --no-cache
docker compose up -d
```

### Camera not detected inside Docker
```bash
# On host, check device number
ls /dev/video*
# If it's /dev/video2, update camera_device parameter
```

### ROS2 nodes can't communicate
```bash
echo $ROS_DOMAIN_ID   # should be 0
docker inspect platopod-ros | grep NetworkMode
# Should show: "NetworkMode": "host"
```
