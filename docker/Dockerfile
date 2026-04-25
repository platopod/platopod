FROM osrf/ros:jazzy-desktop

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    hostapd \
    dnsmasq \
    wireless-tools \
    iw \
    v4l-utils \
    ffmpeg \
    nano \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies (separate RUN to isolate pip errors)
RUN pip3 install --break-system-packages --ignore-installed \
    shapely \
    numpy \
    websockets \
    fastapi \
    "uvicorn[standard]" \
    pyyaml \
    opencv-python-headless \
    pupil-apriltags \
    pytest \
    pytest-asyncio \
    pytest-cov

# Gazebo simulation (optional — for 3D physics mode)
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Source ROS2 in every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Default command: bash
CMD ["/bin/bash"]
