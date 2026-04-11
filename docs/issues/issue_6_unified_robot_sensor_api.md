## Unified Robot Sensor API

Define the sensor data interface for receiving simulated sensor readings from both physical and virtual robots. Sensor data is delivered over the same WebSocket connection established in Issue #3 (Unified Robot Control API). Individual sensor implementations (Lidar, Sonar, GPS, Friend-or-Foe) are defined in separate issues (#8+). Team and robot assignment is managed by the admin (Issue #7: Exercise and Team Management).

### Design principles

- **Streaming, not polling.** Sensor data is pushed to subscribed clients at a configurable rate. The client always has the latest data available without needing to request it.
- **Event-driven for discrete sensors.** Sensors that produce data only on state changes (e.g. Friend-or-Foe detection) push events rather than continuous streams.
- **Rate-controlled.** Clients specify the desired update rate per sensor. The server decimates sensor output to match, allowing clients to trade bandwidth for responsiveness.
- **Team-aware.** Robots can be assigned to teams by an admin (Issue #7). Sensor data can be shared within a team, enabling cooperative strategies and multi-robot fusion.
- **Sensor-name-agnostic.** The subscription and delivery protocol works with any sensor type. New sensors can be added to the library without modifying the API.

### Sensor library

The platform supports a library of simulated sensors. All sensors are computed server-side using robot poses, the arena boundary (Issue #1), and the arena map (obstacles, virtual walls). No physical sensors are required on the robots.

| Sensor | Type | Output | Issue |
|--------|------|--------|-------|
| 2D Lidar | Continuous | Distance per degree (360 values) | #8 |
| Sonar | Continuous | Distance per segment (e.g. 36 values at 10° resolution) | #9 |
| GPS | Continuous | Position and orientation relative to arena origin (Tag 101) | #10 |
| Friend-or-Foe (FoF) | Event-driven | Detection of team-identified robots with bearing and range | #11 |

Additional sensors can be added to the library without modifying the API — the subscription mechanism is sensor-name-agnostic.

### Sensor computation

All sensors are computed by a **sensor simulation node** on the server. This node:

1. Subscribes to all robot poses (from the vision node for physical robots, from the virtual simulation node (Issue #4) for virtual robots).
2. Maintains the arena model (boundary polygon from Issue #1, obstacle positions, virtual walls).
3. For each robot with configured sensors, computes sensor readings at the sensor's native rate.
4. Publishes readings to per-robot sensor topics.

Sensor computation uses **ray casting** against the arena model. For each sensor ray (e.g. each degree of Lidar), the node computes the intersection with arena boundaries, obstacles, and other robots. The nearest intersection determines the range reading.

The arena model includes:
- Arena boundary polygon (Issue #1)
- Other robots as circles (pose + radius from Issue #2 registry)
- Static obstacles (defined in arena configuration YAML)
- Virtual walls (projected or configured)

### Sensor subscription

Clients subscribe to sensors per robot over the existing WebSocket connection (Issue #3):

**Client → Server: subscribe to sensors**
```json
{
  "type": "subscribe_sensors",
  "robot_id": 1,
  "sensors": ["lidar_2d", "gps"],
  "rate_hz": 10
}
```

**Client → Server: unsubscribe from sensors**
```json
{
  "type": "unsubscribe_sensors",
  "robot_id": 1,
  "sensors": ["lidar_2d"]
}
```

**Server → Client: subscribe acknowledgement**
```json
{
  "type": "subscribe_sensors_ack",
  "robot_id": 1,
  "subscribed": ["lidar_2d", "gps"],
  "failed": [{ "sensor": "sonar", "reason": "not_configured" }]
}
```

If `rate_hz` is omitted, the server uses the sensor's default rate. The client can subscribe to different sensors at different rates by sending multiple subscribe messages.

A client can only subscribe to sensors for robots it is authorised to observe (assigned by admin, Issue #7). Attempting to subscribe to an unassigned robot's sensors returns a `not_authorized` error.

### Sensor data messages

**Continuous sensors** (Lidar, Sonar, GPS) deliver periodic readings:

```json
{
  "type": "sensor",
  "robot_id": 1,
  "sensor": "lidar_2d",
  "timestamp": 1709312445.123,
  "data": {
    "ranges": [0.45, 0.44, 0.46, "...360 values"],
    "angle_min": 0.0,
    "angle_max": 6.283,
    "angle_increment": 0.01745,
    "range_max": 2.0
  }
}
```

```json
{
  "type": "sensor",
  "robot_id": 1,
  "sensor": "sonar",
  "timestamp": 1709312445.123,
  "data": {
    "ranges": [0.45, 0.50, 0.38, "...36 values at 10° segments"],
    "segment_angle_deg": 10.0,
    "range_max": 1.5
  }
}
```

```json
{
  "type": "sensor",
  "robot_id": 1,
  "sensor": "gps",
  "timestamp": 1709312445.123,
  "data": {
    "x": 0.25,
    "y": 0.31,
    "theta": 1.62,
    "origin_tag_id": 101
  }
}
```

The GPS sensor returns position and orientation in the arena coordinate system, with the origin defined by AprilTag 101 (as established in Issue #1). The homography and coordinate system are consistent with the web dashboard (Issue #5).

**Event-driven sensors** (FoF) deliver notifications only when detections occur:

```json
{
  "type": "sensor_event",
  "robot_id": 1,
  "sensor": "fof",
  "timestamp": 1709312445.123,
  "data": {
    "detected_robot_id": 5,
    "team": "red",
    "bearing": 0.785,
    "range": 0.32,
    "classification": "foe"
  }
}
```

### Sensor configuration

Each sensor has configurable parameters that can be set per robot. This allows different robots to have different sensor capabilities (e.g. a "scout" robot with long-range Lidar, a "basic" robot with Sonar only):

**Client → Server: configure sensor**
```json
{
  "type": "configure_sensor",
  "robot_id": 1,
  "sensor": "lidar_2d",
  "config": {
    "range_max": 2.0,
    "angular_resolution_deg": 1.0,
    "noise_stddev": 0.01
  }
}
```

**Server → Client: configuration acknowledgement**
```json
{
  "type": "configure_sensor_ack",
  "robot_id": 1,
  "sensor": "lidar_2d",
  "status": "ok"
}
```

Sensor configuration may be restricted to admin-only for exercise scenarios (Issue #7), preventing students from giving themselves better sensors during a competition.

### Sensor noise model

All continuous sensors support configurable noise to simulate real-world sensor imperfections:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `noise_stddev` | Standard deviation of Gaussian noise added to readings | Sensor-specific |
| `dropout_rate` | Probability of a missing/invalid reading per sample | 0.0 |
| `update_jitter_ms` | Random variation in delivery timing | 0 |

Setting `noise_stddev` to 0 produces ideal readings — useful for debugging and introductory exercises. Increasing noise creates more realistic conditions for testing robust algorithms.

Default noise levels per sensor:

| Sensor | Default `noise_stddev` | Description |
|--------|----------------------|-------------|
| Lidar | 0.005 m | ±5mm range noise |
| Sonar | 0.02 m | ±20mm range noise (less precise than Lidar) |
| GPS | 0.001 m, 0.01 rad | ±1mm position, ±0.6° heading |

### Sensor presets

To simplify configuration for common scenarios, the server provides named sensor presets:

**Client → Server: apply preset**
```json
{
  "type": "apply_sensor_preset",
  "robot_id": 1,
  "preset": "basic_scout"
}
```

Example presets:

| Preset | Sensors | Description |
|--------|---------|-------------|
| `minimal` | GPS only | Simplest configuration for beginners |
| `basic_scout` | GPS + Sonar (front 90°) | Basic navigation and obstacle detection |
| `full_suite` | GPS + Lidar + Sonar + FoF | All sensors, realistic noise |
| `ideal` | GPS + Lidar (no noise) | Perfect sensors for algorithm debugging |
| `competitive` | GPS + Sonar + FoF (high noise) | Challenging conditions for advanced exercises |

Presets are defined in a server-side YAML configuration file. Instructors can create custom presets for specific exercises.

### Teams

Robots can be assigned to teams by an admin (Issue #7). The sensor API provides team-based data sharing:

**Client → Server: subscribe to team sensor feed**
```json
{
  "type": "subscribe_team",
  "team": "blue",
  "shared_sensors": ["gps", "fof"]
}
```

**Server → Client: team sensor update**
```json
{
  "type": "team_update",
  "team": "blue",
  "timestamp": 1709312445.123,
  "robots": {
    "1": {
      "gps": { "x": 0.25, "y": 0.31, "theta": 1.62 },
      "fof": []
    },
    "3": {
      "gps": { "x": 0.40, "y": 0.15, "theta": 0.0 },
      "fof": [{ "detected_robot_id": 5, "team": "red", "bearing": 1.2, "range": 0.5 }]
    }
  }
}
```

**Client → Server: unsubscribe from team**
```json
{
  "type": "unsubscribe_team",
  "team": "blue"
}
```

### Team data sharing rules

- Team assignment is managed by the admin (Issue #7). Students cannot change team assignments.
- Only sensors listed in `shared_sensors` are shared — other sensor data remains private to the controlling client.
- Shared sensor data is available to all clients assigned to that team, not just individual robot controllers. This allows a team coordinator to monitor all teammates.
- The server aggregates shared sensor data from all team members into a single `team_update` message per tick, reducing message count.
- Team updates are delivered at the lowest `rate_hz` among the subscribed sensors to avoid overwhelming slower clients.

### ROS2 internal structure

Sensor data is computed by the sensor simulation node and published to per-robot topics:

```
/robot_{id}/sensors/lidar_2d     # sensor_msgs/LaserScan
/robot_{id}/sensors/sonar        # sensor_msgs/Range (or custom multi-range)
/robot_{id}/sensors/gps          # geometry_msgs/Pose2D
/robot_{id}/sensors/fof          # custom message
/team/{team_name}/shared         # aggregated team data
```

The API gateway subscribes to these topics for subscribed clients and bridges them to the WebSocket as JSON messages. Clients never interact with ROS2 directly.

### Python SDK extensions

The SDK (introduced in Issue #3) extends with sensor methods:

```python
from platopod import Arena

arena = Arena("ws://192.168.4.1:8080/api/control")

# Subscribe to robot with sensors
arena.subscribe_sensors(robot_id=1, sensors=["lidar_2d", "gps"], rate_hz=10)

# Read latest sensor data (non-blocking, returns last received value)
scan = arena.get_sensor(robot_id=1, sensor="lidar_2d")
pos = arena.get_sensor(robot_id=1, sensor="gps")

# Register callback for continuous sensor data
def process_scan(data):
    min_range = min(data["ranges"])
    print(f"Nearest obstacle: {min_range:.2f}m")

arena.on_sensor("lidar_2d", robot_id=1, callback=process_scan)

# Register callback for sensor events
arena.on_sensor("fof", robot_id=1, callback=lambda event: handle_detection(event))

# Configure sensor parameters
arena.configure_sensor(robot_id=1, sensor="lidar_2d", config={
    "range_max": 1.5,
    "noise_stddev": 0.02
})

# Apply a sensor preset
arena.apply_preset(robot_id=1, preset="basic_scout")

# Team operations
arena.subscribe_team("blue", shared_sensors=["gps", "fof"])
arena.on_team_update("blue", callback=lambda data: update_world_model(data))

# Example: reactive obstacle avoidance using Lidar
while True:
    scan = arena.get_sensor(robot_id=1, sensor="lidar_2d")
    if scan and min(scan["ranges"]) < 0.1:
        arena.cmd_vel(robot_id=1, linear_x=0.0, angular_z=0.5)  # turn away
    else:
        arena.cmd_vel(robot_id=1, linear_x=0.15, angular_z=0.0)  # drive forward
    arena.sleep(0.1)
```

### Relationship to control events (Issue #3)

Issue #3 defines `boundary_contact` and `collision_contact` events. These are **control pipeline events** — they fire when a command is filtered. Sensor data is different — it represents what the robot **perceives**, not what happened to its commands.

For example:
- `boundary_contact` event: "your last command was blocked because you hit the north wall"
- Lidar sensor reading: "there is a wall 0.03m ahead at bearing 0°"

Both convey proximity information, but through different mechanisms and at different times. A well-designed student program uses sensor data proactively (detect wall → turn before hitting it) rather than relying on contact events reactively (hit wall → now what?).

### Acceptance criteria

- [ ] Sensor subscription mechanism over existing WebSocket connection (Issue #3)
- [ ] Subscribe acknowledgement returned with success/failure per sensor
- [ ] Client-configurable update rate with server-side decimation
- [ ] Continuous sensor data streaming with defined message formats (Lidar, Sonar, GPS)
- [ ] Event-driven sensor notifications with defined message format (FoF)
- [ ] Per-robot sensor configuration with noise parameters
- [ ] Configuration acknowledgement returned
- [ ] Sensor noise model (Gaussian noise, dropout rate, jitter) applied server-side
- [ ] Sensor presets loadable via API and YAML configuration
- [ ] Team sensor subscription with aggregated team updates
- [ ] Team data sharing respects admin-assigned teams (Issue #7)
- [ ] Only authorised clients can subscribe to a robot's sensors
- [ ] Sensor data computed server-side via ray casting against arena model
- [ ] Sensor data published as ROS2 topics, bridged to WebSocket by API gateway
- [ ] Python SDK extended with `subscribe_sensors`, `get_sensor`, `on_sensor`, `configure_sensor`, `apply_preset`, and team methods
- [ ] API is sensor-name-agnostic — new sensor types can be added without protocol changes
- [ ] Sensor data works identically for physical and virtual robots
