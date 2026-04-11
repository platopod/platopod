## Unified Robot Control API

Define the programmatic control interface for commanding both physical and virtual robots. The API abstracts robot type — clients send velocity commands to a `robot_id` without knowing or caring whether the robot is physical or virtual. A single client session can control multiple robots simultaneously for coordinated strategies.

### Design principles

- **API-first:** The primary interface is a programmatic API. All interactions — keyboard control, web dashboard, autonomous scripts — are clients of this API.
- **Language-agnostic:** The underlying protocol is WebSocket with JSON messages. The initial SDK is Python, with future bindings planned for C, MATLAB, and visual programming languages.
- **Multi-robot:** A single client session can subscribe to and control any number of robots, enabling formation control and multi-robot coordination.

### Control model

Robots are controlled via velocity commands using two fields:

| Field | Description | Unit |
|-------|-------------|------|
| `linear_x` | Forward/backward speed | m/s |
| `angular_z` | Rotational speed (positive = counter-clockwise) | rad/s |

Maximum velocities are configurable per robot and enforced server-side:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_speed` | 0.2 m/s | Clamps `linear_x` magnitude |
| `max_angular_speed` | 2.0 rad/s | Clamps `angular_z` magnitude |

### WebSocket protocol

**Connect:**
```
WS /api/control
```

A single WebSocket connection per client. No per-robot connections needed.

**Client → Server: velocity command**
```json
{
  "type": "cmd_vel",
  "robot_id": 1,
  "linear_x": 0.1,
  "angular_z": 0.5
}
```

Sending a `cmd_vel` to a robot automatically subscribes the client to that robot's pose updates and events. Explicit subscription is not required before sending commands.

**Client → Server: subscribe to robot updates (observation only)**
```json
{
  "type": "subscribe",
  "robot_ids": [1, 3, 5]
}
```

**Server → Client: subscribe acknowledgement**
```json
{
  "type": "subscribe_ack",
  "subscribed": [1, 3],
  "failed": [{ "robot_id": 99, "reason": "not_found" }]
}
```

Subscribe is used when a client wants to observe robots without controlling them. The client receives pose updates and events for all subscribed robots (both explicitly subscribed and auto-subscribed via `cmd_vel`).

**Client → Server: unsubscribe**
```json
{
  "type": "unsubscribe",
  "robot_ids": [3, 5]
}
```

**Client → Server: list robots**

Robot listing uses the REST endpoint defined in Issue #2:
```
GET /robots
```

This is not part of the WebSocket protocol. The Python SDK's `arena.list_robots()` method calls this REST endpoint internally.

**Server → Client: pose update (periodic, e.g. 10 Hz per robot)**
```json
{
  "type": "pose",
  "robot_id": 1,
  "x": 0.25,
  "y": 0.31,
  "theta": 1.62,
  "current_linear_x": 0.1,
  "current_angular_z": 0.0
}
```

The `current_linear_x` and `current_angular_z` fields reflect the robot's actual velocity, which may differ from the last commanded values due to boundary clamping, collision filtering, or motor lag on physical robots.

**Server → Client: event notification**
```json
{
  "type": "event",
  "robot_id": 1,
  "event": "boundary_contact",
  "detail": {
    "edge": "north",
    "clamped": true
  }
}
```

**Server → Client: error**
```json
{
  "type": "error",
  "code": "robot_busy",
  "robot_id": 1,
  "message": "Robot is controlled by another client"
}
```

Error codes:

| Code | Description |
|------|-------------|
| `robot_busy` | Another client has control of this robot |
| `not_found` | Robot ID does not exist in the registry |
| `invalid_command` | Malformed message or missing required fields |
| `robot_inactive` | Robot exists but is not currently active |

### Event types

The server pushes event notifications when state changes occur for subscribed robots:

| Event | Description |
|-------|-------------|
| `boundary_contact` | Robot reached arena perimeter or a static obstacle; command was clamped or blocked. Detail includes `element` (perimeter/obstacle) and `label` |
| `collision_contact` | Robot reached another robot; command was clamped or blocked. Includes `other_robot_id` in detail |
| `command_clamped` | Velocity exceeded maximum and was clamped |
| `watchdog_timeout` | No command received within velocity watchdog period; velocity zeroed. Fires **once** per transition |
| `control_released` | Control of the robot was released due to control timeout or explicit release |
| `robot_frozen` | Robot is frozen by a game effect (Issue #7); commands rejected until freeze expires |
| `robot_disconnected` | Physical robot lost connection |
| `robot_reconnected` | Physical robot reconnected |

These events provide the foundation for simulated sensors in Issue #5 (e.g. a simulated bumper sensor maps directly to `boundary_contact` and `collision_contact` events).

### Velocity watchdog

If no `cmd_vel` is received for a specific robot within a configurable timeout (default 500ms), the server automatically sets that robot's velocity to zero. This ensures robots stop if the client pauses or stops sending commands.

The watchdog:
- Runs independently per robot.
- Fires a `watchdog_timeout` event **once** when velocity transitions from non-zero to zero. It does not fire repeatedly while the robot remains stationary.
- **Does not release control.** The client retains control of the robot and can resume sending commands at any time.

### Control timeout

Separate from the velocity watchdog, a **control timeout** (configurable, default 30 seconds) releases control of a robot if the controlling client sends no `cmd_vel` commands for an extended period. This prevents a client from indefinitely locking a robot it is no longer actively using.

When the control timeout fires:
- A `control_released` event is pushed to the client.
- The robot becomes available for other clients to control.
- The client remains subscribed for observation (pose updates and events continue).

### Server-side command pipeline

Each incoming `cmd_vel` passes through a processing pipeline:

```
Client (WebSocket)
    → API Gateway (validates robot_id, parses JSON, checks control ownership)
    → Exercise Filter (rejects if paused/not running/not authorised — Issue #7)
    → Speed Limiter (clamps to max velocities → command_clamped event)
    → Arena Filter (boundary perimeter + static obstacles — Issue #1 arena model)
    → Collision Filter (robot-robot clearance → collision_contact event)
    → Game Effects (rejects if robot is frozen — Issue #7)
    → Router
        ├→ Physical: cmd_vel → robot_bridge_node → UDP "M <linear> <angular>" → ESP32
        └→ Virtual: cmd_vel → Virtual Simulation Node (Issue #4)
```

### Arena boundary and obstacle filtering

The arena filter uses the arena model (Issue #1) to prevent robots from crossing:

- **Arena boundary perimeter** — the convex hull defined by corner AprilTags.
- **Static obstacles** — walls, barriers, and pillars defined in the exercise configuration or added by the admin mid-exercise.

Before forwarding a command, the server predicts the robot's next position over a short lookahead. If the predicted position (plus robot radius) would intersect the boundary perimeter or any obstacle polygon, the command is modified or rejected. A `boundary_contact` event is pushed to the client, identifying the element hit:

```json
{
  "type": "event",
  "robot_id": 1,
  "event": "boundary_contact",
  "detail": {
    "element": "obstacle",
    "label": "wall_north",
    "clamped": true
  }
}
```

The `element` field is either `"perimeter"` (arena edge) or `"obstacle"` (with the obstacle's `label`).

### Collision avoidance between robots

Extends the arena filtering to robot-robot interactions:

- Before forwarding a command, the server predicts the robot's next position over a short lookahead.
- If the predicted position would bring the robot within contact distance of any other robot (sum of both radii), the command is modified or rejected.
- A `collision_contact` event is pushed to the client, including the `other_robot_id` of the robot involved.
- The filter applies symmetrically — a virtual robot cannot pass through a physical robot and vice versa.

### ROS2 topic structure (internal)

Each robot uses a namespaced topic structure within the ROS2 network:

```
/robot_{id}/cmd_vel       # geometry_msgs/Twist — velocity commands
/robot_{id}/pose          # geometry_msgs/Pose2D — current pose
/robot_{id}/events        # custom message — event notifications
```

The WebSocket API gateway bridges between the JSON WebSocket protocol and these ROS2 topics. Clients never interact with ROS2 directly.

### Python SDK (initial implementation)

The Python SDK wraps the WebSocket protocol and REST endpoints into a clean API:

```python
from platopod import Arena

arena = Arena("ws://192.168.4.1:8080/api/control")

# List available robots (uses REST endpoint from Issue #2)
robots = arena.list_robots()

# Subscribe to robots for observation
arena.subscribe([1, 3])

# Send velocity command (auto-subscribes to robot 1)
arena.cmd_vel(robot_id=1, linear_x=0.1, angular_z=0.0)

# Register event callback
def on_boundary(event):
    print(f"Robot {event.robot_id} hit {event.detail['edge']}")

arena.on("boundary_contact", on_boundary)

# Register error callback
def on_error(error):
    print(f"Error on robot {error.robot_id}: {error.code} — {error.message}")

arena.on_error(on_error)

# Get current pose
pose = arena.get_pose(robot_id=1)

# Release control of a robot
arena.release(robot_id=1)

# Multi-robot control loop
while True:
    arena.cmd_vel(robot_id=1, linear_x=0.1, angular_z=0.0)
    arena.cmd_vel(robot_id=3, linear_x=0.1, angular_z=0.3)
    arena.sleep(0.1)  # 10 Hz control loop
```

### Keyboard control demo

The keyboard control demo is a Python script that uses the SDK, not a built-in feature of the server:

```python
from platopod import Arena, KeyboardController

arena = Arena("ws://192.168.4.1:8080/api/control")
robot_id = int(input("Enter robot ID: "))

# KeyboardController maps arrow keys to cmd_vel calls
# using impulse-based steering:
#   Key down  → start sending cmd_vel at 10 Hz
#   Key held  → continue sending
#   Key up    → send zero velocity
#
# ↑ = forward, ↓ = backward, ← = turn left, → = turn right
# ↑+← = forward-left arc, ↑+→ = forward-right arc

controller = KeyboardController(arena, robot_id, speed=0.15)
controller.run()  # blocks, listens for key events
```

### Shared control policy

Multiple clients can subscribe to the same robot for observation (receiving pose updates and events). However, only one client can send `cmd_vel` to a given robot at a time.

**Acquiring control:** The first client to send a `cmd_vel` to a robot acquires control. The client is automatically subscribed to that robot's updates.

**Releasing control:** Control is released when:
- The client explicitly releases: `arena.release(robot_id)`
- The client's WebSocket disconnects
- The control timeout expires (default 30 seconds of no commands)

**Contention:** Other clients attempting to send `cmd_vel` to a controlled robot receive a `robot_busy` error. They can still subscribe for observation.

**Velocity watchdog vs control timeout:** The velocity watchdog (500ms) zeros the robot's velocity for safety but does not release control. The control timeout (30s) releases control to allow other clients to take over. These are independent timers with different purposes.

### Sensor data integration (see Issue #5)

The WebSocket connection established by this API is also used to deliver sensor data streams defined in Issue #5 (Unified Robot Sensor API). The protocol is designed to carry commands, pose updates, events, and sensor readings over a single connection. No separate sensor endpoint is needed.

### Acceptance criteria

- [ ] Single WebSocket endpoint supporting multi-robot control per session
- [ ] `cmd_vel` commands accepted and forwarded identically for physical and virtual robots
- [ ] Sending `cmd_vel` auto-subscribes the client to that robot
- [ ] Explicit subscribe/unsubscribe for observation with acknowledgement response
- [ ] Subscribe acknowledgement returns success/failure per robot_id
- [ ] Event notifications pushed for boundary contact, collision, clamping, watchdog, control release, and connection changes
- [ ] Error messages returned for robot_busy, not_found, invalid_command, and robot_inactive
- [ ] Speed limiter clamps velocities to configurable per-robot maximums
- [ ] Boundary filter prevents arena boundary crossing (Issue #1)
- [ ] Arena filter prevents crossing static obstacles from arena model (Issue #1)
- [ ] Boundary contact events identify element type (perimeter/obstacle) and label
- [ ] Collision filter prevents robot-robot overlap using sum-of-radii clearance
- [ ] Game effects filter rejects commands for frozen robots (Issue #7)
- [ ] Velocity watchdog (500ms) zeros velocity but does not release control; fires once per transition
- [ ] Control timeout (30s) releases control after prolonged inactivity
- [ ] Shared control policy: one controller per robot, multiple observers allowed
- [ ] Robot listing uses REST endpoint from Issue #2
- [ ] Pose updates include current actual velocity (`current_linear_x`, `current_angular_z`)
- [ ] Python SDK wrapping WebSocket and REST protocols with clean API
- [ ] Keyboard control demo script implemented using the Python SDK
- [ ] Command pipeline works identically for physical and virtual robots
