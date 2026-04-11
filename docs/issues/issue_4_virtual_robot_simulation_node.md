## Virtual Robot Simulation Node

Implement the server-side kinematics engine that makes virtual robots move. When a virtual robot receives a `cmd_vel` command (routed by the command pipeline in Issue #3), this node computes the resulting motion using differential drive kinematics and publishes the updated pose. Virtual robots behave identically to physical robots from the perspective of all other system components.

### Role in the architecture

For physical robots, the ESP32 drives the motors and the overhead camera observes the resulting pose. For virtual robots, this node replaces both — it computes the motion mathematically and publishes the pose directly:

```
                    Physical robot path:
cmd_vel → micro-ROS agent → ESP32 → motors → camera observes → pose published

                    Virtual robot path:
cmd_vel → Virtual Simulation Node → kinematics computed → pose published
```

Both paths result in the same output: a pose on `/robot_{id}/pose`. All downstream consumers (boundary filter, collision filter, sensor simulation, web dashboard, student API) are agnostic to the source.

### Differential drive kinematics

At each simulation tick (default 50 Hz), the node updates each virtual robot's pose using the standard differential drive model:

```
x     += linear_x * cos(theta) * dt
y     += linear_x * sin(theta) * dt
theta += angular_z * dt
theta  = normalise(theta)  // wrap to [-π, π]
```

Where:
- `(x, y, theta)` is the robot's current pose in arena coordinates (metres, radians)
- `linear_x` is the commanded forward velocity (m/s)
- `angular_z` is the commanded rotational velocity (rad/s)
- `dt` is the time step (1/tick_rate, e.g. 0.02s at 50 Hz)

This is an ideal kinematic model — no wheel slip, no inertia, no motor dynamics. The robot responds instantly to velocity commands.

### Simulation parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tick_rate_hz` | 50 | Simulation update frequency |
| `max_linear_speed` | 0.2 m/s | Maximum forward/backward speed (enforced redundantly with Issue #3 speed limiter) |
| `max_angular_speed` | 2.0 rad/s | Maximum rotational speed |

These defaults match the physical Plato Pod's capabilities so that virtual and physical robots behave comparably.

### Optional realism parameters

For more realistic simulation (e.g. when training neural networks or testing robust control algorithms), the node supports optional dynamics:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `acceleration_limit` | None (instant) | Maximum acceleration (m/s²). When set, velocity ramps rather than jumping instantly |
| `velocity_noise_stddev` | 0.0 | Gaussian noise added to the executed velocity, simulating motor imprecision |
| `pose_drift_stddev` | 0.0 | Gaussian noise added to the pose per tick, simulating odometry drift |

When all optional parameters are at their defaults, the simulation is ideal — deterministic and perfectly responsive. This is the recommended starting point for students. Instructors can increase noise and limits for advanced exercises.

### Lifecycle management

The virtual simulation node manages the lifecycle of virtual robots in coordination with the robot registry (Issue #2):

- **Spawn:** When a virtual robot is created (Issue #2), the simulation node creates an internal state entry with the initial pose and zero velocity.
- **Update:** Each tick, the node iterates over all active virtual robots, applies kinematics, and publishes updated poses.
- **Remove:** When a virtual robot is deleted (Issue #2), the simulation node removes its state entry and stops publishing.
- **Zero velocity:** When no `cmd_vel` has been received within the watchdog timeout (Issue #3), the robot's velocity is already zeroed by the command pipeline. The simulation node simply applies zero velocity, resulting in no pose change.

### Boundary and collision enforcement

The simulation node computes the raw kinematic update but does **not** enforce boundaries or collisions itself. That responsibility belongs to the boundary filter and collision filter in the command pipeline (Issues #1 and #3), which modify or reject `cmd_vel` commands before they reach this node.

However, as a safety net, the simulation node clamps the resulting pose to remain within the arena boundary. If floating-point accumulation causes a robot to drift slightly outside the boundary, the pose is projected back to the nearest valid position. This is a fallback — under normal operation, the upstream filters prevent this from occurring.

### ROS2 integration

**Subscribed topics (per virtual robot):**
```
/robot_{id}/cmd_vel       # geometry_msgs/Twist — velocity commands
```

**Published topics (per virtual robot):**
```
/robot_{id}/pose          # geometry_msgs/Pose2D — current pose (x, y, theta)
```

The node publishes poses at the tick rate (50 Hz). Downstream consumers (API gateway, sensor simulation) can decimate as needed.

**Node name:** `virtual_sim_node`

**Parameters (configurable via ROS2 parameter server or YAML config):**
```yaml
virtual_sim_node:
  ros__parameters:
    tick_rate_hz: 50
    max_linear_speed: 0.2
    max_angular_speed: 2.0
    acceleration_limit: 0.0      # 0 = instant
    velocity_noise_stddev: 0.0
    pose_drift_stddev: 0.0
```

### Multiple virtual robots

The node manages all virtual robots in a single process. Each robot is an independent state entry (pose + current velocity). The tick loop iterates over all entries, so computational cost scales linearly with the number of virtual robots. For the expected scale (tens of robots, not thousands), this is efficient.

### Testing with RViz2

The virtual simulation node can be tested immediately using RViz2 without any custom UI:

1. Launch the simulation node and spawn a virtual robot via the REST API (Issue #2).
2. Open RViz2 and add a `Pose` display subscribing to `/robot_{id}/pose`.
3. Publish `cmd_vel` commands from the terminal:
   ```bash
   ros2 topic pub /robot_1/cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0.1}, angular: {z: 0.5}}"
   ```
4. Observe the pose arrow moving and rotating in RViz2.

This provides immediate visual feedback during development without requiring the web dashboard (Issue #5) or Python SDK.

### Relationship to turtlesim

This node is architecturally similar to ROS2's `turtlesim` — both subscribe to `cmd_vel` and publish pose using 2D kinematics. Key differences:

| Aspect | turtlesim | Virtual Simulation Node |
|--------|-----------|------------------------|
| Coordinate system | Arbitrary pixel space | Arena coordinates (metres) |
| Boundary handling | Wraps or stops at window edge | Defers to upstream filter (Issue #1) |
| Multiple robots | One node per turtle | Single node, all virtual robots |
| Rendering | Built-in window | Defers to web dashboard (Issue #5) |
| Noise model | None | Optional velocity and drift noise |

### Acceptance criteria

- [ ] Virtual simulation node subscribes to `cmd_vel` and publishes pose for each virtual robot
- [ ] Differential drive kinematics correctly computed at configurable tick rate
- [ ] Pose published at tick rate (default 50 Hz)
- [ ] Theta normalised to [-π, π] after each update
- [ ] State created on virtual robot spawn, removed on deletion
- [ ] Zero velocity results in no pose change
- [ ] Safety-net pose clamping to arena boundary
- [ ] Optional acceleration limit ramps velocity instead of instant response
- [ ] Optional velocity noise and pose drift for realistic simulation
- [ ] All parameters configurable via ROS2 parameter server
- [ ] Single node manages all virtual robots efficiently
- [ ] Testable via RViz2 and command-line `ros2 topic pub`
- [ ] Pose output indistinguishable from physical robot pose output
