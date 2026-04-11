# Plato Pod — Development Setup Guide

Complete step-by-step instructions to go from zero to a running development environment with Docker, ROS2, and Claude Code.

## Prerequisites

- Ubuntu 24.04 (native or WSL2)
- Git installed (`sudo apt install git`)
- Docker installed (see Step 1 if not)
- Claude Code installed (see Step 4 if not)
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

## Step 2: Create the orphan branch and push initial structure

```bash
# Navigate to your local clone of the repo
cd ~/platopod    # or wherever your clone is

# Create the orphan branch (clean, no shared history with main)
git checkout --orphan v2-platform
git rm -rf .

# Unpack the initial structure (download platopod-v2-initial.zip from Claude first)
unzip -o ~/Downloads/platopod-v2-initial.zip
mv platopod-v2/* platopod-v2/.* .
rmdir platopod-v2

# Commit and push
git add .
git commit -m "v2-platform: Initial project structure with issues, Docker, and Claude Code instructions"
git push origin v2-platform
```

Verify at https://github.com/platopod/platopod — you should see the `v2-platform` branch with the new structure.

---

## Step 3: Build and start the Docker container

```bash
cd ~/platopod/docker

# Build the image (takes 5-10 minutes the first time)
docker compose build

# Start the container in the background
docker compose up -d

# Verify it's running
docker compose ps
```

**Test that it works:**

```bash
# Open a shell inside the container
docker compose exec ros bash

# Inside the container, verify ROS2 is available
ros2 --help

# Verify Python dependencies
python3 -c "import shapely; import numpy; import fastapi; print('All dependencies OK')"

# Exit the container shell
exit
```

**Test camera access (if camera is plugged in):**

```bash
docker compose exec ros bash

# Check camera device
v4l2-ctl --list-devices

# Quick capture test (saves one frame)
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print(f'Camera OK: {frame.shape}' if ret else 'Camera FAILED')
cap.release()
"
exit
```

**Test WiFi dongle (if MT7612U is plugged in):**

```bash
docker compose exec ros bash

# Check wireless interfaces
iw dev

# You should see wlan1 (or similar) for the USB dongle
# wlan0 is typically your laptop's built-in WiFi

# Set up the AP (only when you need ESP32 robots to connect)
sudo bash /ros2_ws/docker/setup_wifi_ap.sh wlan1

exit
```

---

## Step 4: Install Claude Code

```bash
# Claude Code requires Node.js 18+
# Check if you have it:
node --version

# If not installed or too old:
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install nodejs

# Install Claude Code globally
npm install -g @anthropic-ai/claude-code

# Verify installation
claude --version
```

---

## Step 5: Configure Claude Code for the project

```bash
# Navigate to the project root
cd ~/platopod

# Make sure you're on the v2-platform branch
git checkout v2-platform

# Launch Claude Code
claude
```

Claude Code will automatically read `CLAUDE.md` and understand:
- The project architecture and all 10 issues
- Coding standards and conventions
- How to build and test inside Docker
- The development workflow (propose → approve → implement → test)

---

## Step 6: Start developing

In Claude Code, tell it which issue to work on:

```
Let's start implementing Issue #1 (Dynamic Arena Boundary and Arena Model).
Read the spec in docs/issues/issue_1_arena_boundary_and_model.md and propose
an implementation plan.
```

Claude Code will:
1. Read the issue specification
2. Propose which files to create, key functions, and test approach
3. Wait for your approval before writing code
4. Implement incrementally, running tests after each change
5. Present results for review

**To run commands inside Docker from Claude Code**, use:

```bash
docker compose -f docker/docker-compose.yaml exec ros bash -c "cd /ros2_ws && colcon build"
docker compose -f docker/docker-compose.yaml exec ros bash -c "cd /ros2_ws && python3 -m pytest src/plato_pod/tests/ -v"
```

Or open a persistent shell in a separate terminal:

```bash
cd ~/platopod/docker
docker compose exec ros bash
# This terminal stays inside the container — use it for building and testing
```

---

## Daily Development Workflow

### Starting a session

```bash
# Terminal 1: Start Docker (if not already running)
cd ~/platopod/docker
docker compose up -d

# Terminal 2: Open a shell inside Docker for building/testing
docker compose exec ros bash

# Terminal 3: Launch Claude Code
cd ~/platopod
claude
```

### Building and testing (inside Docker, Terminal 2)

```bash
# Build all ROS2 packages
cd /ros2_ws
colcon build
source install/setup.bash

# Run unit tests (fast, no ROS2 nodes needed)
python3 -m pytest src/plato_pod/tests/unit/ -v

# Run integration tests (needs ROS2 nodes running)
ros2 launch plato_pod server.launch.py &
sleep 5
python3 -m pytest src/plato_pod/tests/integration/ -v

# Run a specific node for manual testing
ros2 run plato_pod arena_model_node

# Visual debugging with RViz2 (if X11 forwarding is set up)
rviz2
```

### Testing the Python SDK (outside Docker, on your host)

```bash
# Install the SDK (once)
cd ~/platopod
pip install -e server/  # or wherever the SDK package lives

# Run a test script
python3 -c "
from platopod import Arena
arena = Arena('ws://localhost:8080/api/control')
robots = arena.list_robots()
print(f'Found {len(robots)} robots')
"
```

### Committing changes

```bash
cd ~/platopod
git add .
git commit -m "[component] description of change"
git push origin v2-platform
```

### Ending a session

```bash
# Stop Docker container (preserves build cache)
cd ~/platopod/docker
docker compose down

# Or leave it running if you'll continue later
```

---

## Connecting Physical ESP32 Robots (when hardware arrives)

### 1. Set up WiFi AP

```bash
# Inside Docker
sudo bash /ros2_ws/docker/setup_wifi_ap.sh wlan1
```

### 2. Start the robot bridge node

```bash
# Inside Docker
ros2 run plato_pod robot_bridge_node
```

### 3. Flash and connect an ESP32

```bash
# On your host (ESP-IDF must be installed natively, not in Docker)
cd ~/platopod/firmware
idf.py set-target esp32c3
idf.py menuconfig   # set WiFi SSID=plato-arena, password=platopod123, agent IP=192.168.4.1
idf.py build
idf.py flash monitor
```

### 4. Verify connection

```bash
# Inside Docker — the robot should have registered via UDP
ros2 topic list | grep robot
# You should see /robot_1/cmd_vel, /robot_1/pose, etc.
```

---

## Troubleshooting

### Docker container won't start
```bash
# Check logs
docker compose logs ros

# Rebuild from scratch
docker compose build --no-cache
docker compose up -d
```

### Camera not detected inside Docker
```bash
# On your host, check the device number
ls /dev/video*

# If it's /dev/video2 instead of /dev/video0, edit docker-compose.yaml:
# devices:
#   - /dev/video2:/dev/video0
docker compose down && docker compose up -d
```

### WiFi dongle not showing up
```bash
# On your host, check USB devices
lsusb | grep -i mt7612
# Should show: 0e8d:7612 MediaTek Inc. MT7612U

# Check if the kernel driver loaded
dmesg | tail -20 | grep mt76

# Inside Docker, check interfaces
iw dev
```

### ROS2 nodes can't communicate
```bash
# Check ROS_DOMAIN_ID matches across all terminals
echo $ROS_DOMAIN_ID   # should be 0

# Check that --net=host is in docker-compose.yaml
docker inspect platopod-ros | grep NetworkMode
# Should show: "NetworkMode": "host"
```

### Claude Code can't find the project
```bash
# Make sure you're in the repo root
cd ~/platopod
ls CLAUDE.md   # this file must exist

# Make sure you're on the right branch
git branch     # should show * v2-platform

# Launch Claude Code from the repo root
claude
```
