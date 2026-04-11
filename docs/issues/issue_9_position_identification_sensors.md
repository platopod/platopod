## Position and Identification Sensors (GPS + FoF)

Implement two sensor plugins for the sensor engine (Issue #8): GPS and Friend-or-Foe (FoF). Unlike the range sensors in Issue #8, these plugins do not use ray casting — they operate directly on the robot registry data (poses, teams). GPS provides a noisy version of the robot's own position. FoF detects other robots and classifies them by team affiliation.

### GPS plugin

Simulates a global positioning sensor that returns the robot's position and orientation relative to the arena origin (AprilTag 101, as defined in Issue #1).

In the real system, the server already knows every robot's exact pose from camera tracking (physical) or kinematics (virtual). GPS is the mechanism by which this information is exposed to student code — with configurable noise to simulate real-world GPS imprecision.

```python
class GPSPlugin(SensorPlugin):
    name = "gps"
    sensor_type = "continuous"
    default_rate_hz = 5.0
    default_config = {
        "position_noise_stddev": 0.001,   # ±1mm position noise
        "heading_noise_stddev": 0.01,     # ±0.57° heading noise
        "dropout_rate": 0.0,
        "update_jitter_ms": 0
    }

    def compute(self, ctx: SensorContext) -> dict:
        return {
            "x": ctx.robot_pose.x,
            "y": ctx.robot_pose.y,
            "theta": ctx.robot_pose.theta,
            "origin_tag_id": 101
        }
```

The engine applies noise after `compute()` returns:
- `position_noise_stddev` is applied independently to `x` and `y`
- `heading_noise_stddev` is applied to `theta`
- `theta` is re-normalised to [-π, π] after noise injection

This is the simplest plugin — no geometry, no ray casting. Its value is in providing a clean API for students to read position, with configurable degradation for advanced exercises.

**ROS2 topic:** `/robot_{id}/sensors/gps` — publishes `geometry_msgs/Pose2D`

**WebSocket message (Issue #6):**
```json
{
  "type": "sensor",
  "robot_id": 1,
  "sensor": "gps",
  "timestamp": 1709312445.123,
  "data": {
    "x": 0.251,
    "y": 0.309,
    "theta": 1.634,
    "origin_tag_id": 101
  }
}
```

**Configurable parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `position_noise_stddev` | 0.001 m | Gaussian noise on x and y independently |
| `heading_noise_stddev` | 0.01 rad | Gaussian noise on theta (~0.57°) |
| `dropout_rate` | 0.0 | Probability of missing reading per update |
| `update_jitter_ms` | 0 | Random timing variation in delivery |

**GPS noise presets:**

| Scenario | Position noise | Heading noise | Rationale |
|----------|---------------|---------------|-----------|
| Ideal (debugging) | 0.0 | 0.0 | Perfect information for algorithm development |
| Indoor GPS | 0.001 m | 0.01 rad | Very accurate, simulating camera-based tracking |
| Outdoor GPS | 0.05 m | 0.05 rad | Realistic civilian GPS (~5cm CEP) |
| Degraded GPS | 0.5 m | 0.2 rad | Simulating GPS jamming or urban canyon effects |
| GPS denied | dropout_rate: 1.0 | — | Complete GPS loss, robot must use other sensors |

These presets are relevant for military/wargaming exercises — students learn to handle sensor degradation and GPS-denied environments.

### GPS as the pedagogical entry point

GPS is the recommended first sensor for students learning the platform. A minimal control loop using GPS:

```python
from platopod import Arena
import math

arena = Arena("ws://192.168.4.1:8080/api/control")
arena.subscribe_sensors(robot_id=1, sensors=["gps"], rate_hz=5)

# Drive to target position (0.4, 0.3)
target_x, target_y = 0.4, 0.3

while True:
    gps = arena.get_sensor(robot_id=1, sensor="gps")
    if not gps:
        arena.sleep(0.1)
        continue

    dx = target_x - gps["x"]
    dy = target_y - gps["y"]
    distance = math.sqrt(dx**2 + dy**2)

    if distance < 0.02:  # within 2cm of target
        arena.cmd_vel(robot_id=1, linear_x=0.0, angular_z=0.0)
        print("Reached target!")
        break

    # Compute desired heading
    desired_theta = math.atan2(dy, dx)
    angle_error = desired_theta - gps["theta"]

    # Normalise angle to [-π, π]
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

    # Proportional control
    if abs(angle_error) > 0.1:
        arena.cmd_vel(robot_id=1, linear_x=0.0, angular_z=1.0 * angle_error)
    else:
        arena.cmd_vel(robot_id=1, linear_x=0.1, angular_z=0.5 * angle_error)

    arena.sleep(0.1)
```

This example demonstrates fundamental robotics concepts (goal-seeking, proportional control, angle normalisation) using only GPS — no Lidar or Sonar needed.

### Friend-or-Foe (FoF) plugin

Simulates a team identification sensor that detects nearby robots and classifies them as friend or foe based on team affiliation (Issue #7). Unlike range sensors, FoF is **event-driven** — it pushes detection notifications when robots enter or exit detection range, rather than streaming continuous data.

The FoF sensor models a system where robots broadcast their team identity and other robots with FoF sensors can receive these broadcasts within a configurable range and field of view.

```python
class FoFPlugin(SensorPlugin):
    name = "fof"
    sensor_type = "event"
    default_rate_hz = 5.0
    default_config = {
        "detection_range": 0.5,        # maximum detection distance (m)
        "field_of_view_deg": 360.0,    # detection arc (360 = omnidirectional)
        "identification_range": 0.3,   # range within which team is identified
        "noise_bearing_stddev": 0.05,  # bearing noise (rad, ~2.9°)
        "noise_range_stddev": 0.02,    # range noise (m)
        "false_positive_rate": 0.0,    # probability of phantom detection per tick
        "false_negative_rate": 0.0     # probability of missing a real detection
    }

    def compute(self, ctx: SensorContext) -> list[dict]:
        config = ctx.config
        max_range = config["detection_range"]
        id_range = config["identification_range"]
        fov = math.radians(config["field_of_view_deg"])
        half_fov = fov / 2.0

        detections = []

        for robot in ctx.other_robots:
            dx = robot.pose.x - ctx.robot_pose.x
            dy = robot.pose.y - ctx.robot_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            # Check range
            if distance > max_range:
                continue

            # Check field of view
            bearing = math.atan2(dy, dx) - ctx.robot_pose.theta
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi

            if abs(bearing) > half_fov:
                continue

            # False negative check
            if ctx.config.get("false_negative_rate", 0) > 0:
                # Engine handles this via noise injector
                pass

            # Determine classification
            if distance <= id_range and robot.team is not None and ctx.robot_team is not None:
                if robot.team == ctx.robot_team:
                    classification = "friend"
                else:
                    classification = "foe"
            else:
                classification = "unknown"  # detected but too far to identify

            detections.append({
                "detected_robot_id": robot.robot_id,
                "team": robot.team if distance <= id_range else None,
                "bearing": bearing,
                "range": distance,
                "classification": classification
            })

        return detections
```

**Detection vs identification:** The FoF sensor has two ranges:

| Range | What the robot knows |
|-------|---------------------|
| Within `detection_range` but outside `identification_range` | "Something is at bearing 45°, range 0.4m" — `classification: "unknown"` |
| Within `identification_range` | "Robot 3 (red team, foe) is at bearing 45°, range 0.25m" — `classification: "foe"` |
| Outside `detection_range` | Nothing detected |

This models real-world IFF (Identification Friend or Foe) systems where detection range exceeds identification range.

**ROS2 topic:** `/robot_{id}/sensors/fof` — publishes custom `FoFDetections` message

**WebSocket message — detections (periodic, e.g. 5 Hz):**
```json
{
  "type": "sensor",
  "robot_id": 1,
  "sensor": "fof",
  "timestamp": 1709312445.123,
  "data": {
    "detections": [
      {
        "detected_robot_id": 5,
        "team": "red",
        "bearing": 0.785,
        "range": 0.25,
        "classification": "foe"
      },
      {
        "detected_robot_id": 3,
        "team": null,
        "bearing": -1.2,
        "range": 0.45,
        "classification": "unknown"
      }
    ]
  }
}
```

**WebSocket message — state change event (pushed on detection/loss):**
```json
{
  "type": "sensor_event",
  "robot_id": 1,
  "sensor": "fof",
  "timestamp": 1709312445.123,
  "data": {
    "event": "new_detection",
    "detected_robot_id": 5,
    "classification": "foe",
    "bearing": 0.785,
    "range": 0.25
  }
}
```

```json
{
  "type": "sensor_event",
  "robot_id": 1,
  "sensor": "fof",
  "timestamp": 1709312446.500,
  "data": {
    "event": "lost_detection",
    "detected_robot_id": 5,
    "last_bearing": 0.9,
    "last_range": 0.51
  }
}
```

The FoF plugin produces both continuous data (current detections list) and events (new detection, lost detection). The engine handles both delivery modes.

**Configurable parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `detection_range` | 0.5 m | Maximum distance for detecting any robot |
| `identification_range` | 0.3 m | Maximum distance for team identification |
| `field_of_view_deg` | 360° | Detection arc (360 = omnidirectional, 90 = forward cone) |
| `noise_bearing_stddev` | 0.05 rad | Bearing noise (~2.9°) |
| `noise_range_stddev` | 0.02 m | Range noise |
| `false_positive_rate` | 0.0 | Phantom detections per tick |
| `false_negative_rate` | 0.0 | Probability of missing a real robot |

### FoF and team data sharing

The FoF sensor works in conjunction with team data sharing (Issue #6). When a blue team robot detects a red team robot, that detection can be shared with all blue team members via the team subscription:

```python
# Blue team coordinator receives all team FoF data
arena.subscribe_team("blue", shared_sensors=["gps", "fof"])

def on_team_update(data):
    # Aggregate all foe detections from the team
    all_foes = []
    for robot_id, sensors in data["robots"].items():
        if "fof" in sensors:
            for det in sensors["fof"]["detections"]:
                if det["classification"] == "foe":
                    all_foes.append(det)
    print(f"Team sees {len(all_foes)} foes")

arena.on_team_update("blue", callback=on_team_update)
```

This enables cooperative strategies: one robot scouts and shares detections, while others manoeuvre based on the fused picture.

### FoF in wargaming context

The FoF sensor is directly relevant to military training scenarios:

| Scenario | FoF configuration | Lesson |
|----------|-------------------|--------|
| Full situational awareness | Long range, no noise | Baseline — students learn to use detection data |
| Fog of war | Short identification range, high noise | Decision-making under uncertainty |
| IFF failure | false_positive_rate: 0.1 | Friendly fire risk management |
| Stealth vs detection | Asymmetric ranges per team | Electronic warfare concepts |
| GPS-denied + FoF only | GPS dropout, FoF active | Navigation using relative detections only |

### Sensor presets using GPS and FoF

Extending the presets from Issue #6:

| Preset | GPS config | FoF config |
|--------|-----------|------------|
| `minimal` | Default (low noise) | — |
| `basic_scout` | Default | — |
| `full_suite` | Default | 0.5m detection, 0.3m identification, 360° |
| `ideal` | No noise | 1.0m detection, 1.0m identification, no noise |
| `competitive` | High noise (0.05m) | 0.3m detection, 0.15m identification, 10% false negative |
| `gps_denied` | dropout_rate: 1.0 | 0.5m detection, active |
| `fog_of_war` | High noise (0.1m) | 0.2m detection, 0.1m identification, high noise |

### Dependencies

- **Issue #7** — team assignments (required for FoF classification)
- **Issue #8** — sensor engine node (plugin infrastructure, scheduler, noise injector)
- **Issue #6** — sensor API protocol (subscription, configuration, WebSocket messages)
- **Issue #1** — arena coordinate system (GPS origin at Tag 101)

### Acceptance criteria

- [ ] GPS plugin returns position and orientation relative to Tag 101 origin
- [ ] GPS noise applied independently to x, y, and theta
- [ ] GPS theta re-normalised to [-π, π] after noise injection
- [ ] GPS dropout simulates complete position loss
- [ ] GPS noise presets cover ideal through GPS-denied scenarios
- [ ] FoF plugin detects robots within configurable detection range
- [ ] FoF field of view limits detection to configurable arc
- [ ] FoF distinguishes detection range (something is there) from identification range (friend or foe)
- [ ] Robots outside identification range classified as "unknown"
- [ ] FoF bearing and range noise applied
- [ ] FoF false positive and false negative rates configurable
- [ ] FoF produces both continuous detections list and state change events (new/lost)
- [ ] FoF data shareable via team subscription (Issue #6)
- [ ] Both plugins integrate with sensor engine via standard `SensorPlugin` interface
- [ ] Both plugins configurable via `configure_sensor` API (Issue #6)
- [ ] Sensor presets correctly configure GPS and FoF parameters
- [ ] GPS usable as standalone beginner exercise (drive-to-target example works)
- [ ] FoF functional in team-based wargaming exercises (Issue #7)
