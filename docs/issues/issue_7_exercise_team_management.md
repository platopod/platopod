## Exercise and Team Management

Define the admin layer for managing classroom and wargaming sessions. An exercise is a structured session where an instructor configures the arena, assigns robots to students and teams, controls exercise lifecycle (setup → running → paused → complete), and manages permissions. This issue provides the authority model that governs who can control which robots and access which data across Issues #2, #3, and #6.

### Design principles

- **Exercise as a first-class concept.** An exercise is a configuration file (YAML/JSON) that describes the entire session: teams, robots, sensor loadouts, arena configuration, and objectives. Loading an exercise provisions everything automatically.
- **Configuration-driven, not click-driven.** Instructors prepare exercise configs in advance and load them. Manual adjustments are possible during the session but the config is the primary interface. This scales to large classes without tedious per-robot UI interaction.
- **Minimal authentication.** No user accounts. Instructors authenticate with an admin token. Students join exercises with a name/callsign and receive a session token. Simple enough for a classroom, secure enough to prevent accidental interference.
- **Live adjustability.** The instructor can modify team assignments, sensor parameters, arena obstacles, and robot permissions mid-exercise without restarting.

### Roles

| Role | Capabilities |
|------|-------------|
| **Admin (instructor)** | Create/load/start/pause/resume/end exercises. Assign robots to students and teams. Configure sensors. Spawn/remove virtual robots. Modify arena. View all robots and data. Force-reset robots. |
| **Participant (student)** | Join an exercise with a name. Control assigned robots only. Subscribe to own robots' sensors and team shared data. Observe (but not control) unassigned robots on the dashboard. |
| **Observer** | View-only access to the dashboard and all pose data. No control, no sensor data. For guest viewers, other instructors, or recording. |

### Authentication

**Admin token:** A static token configured in the server's environment or config file. Passed as a header or query parameter on all admin API calls:

```
Authorization: Bearer <admin-token>
```

**Student session:** Students join an exercise via a REST endpoint. No pre-registration required:

```
POST /exercise/join
{
  "name": "Alice",
  "callsign": "alpha-1"
}
```

Response:
```json
{
  "session_token": "abc123",
  "exercise_id": "capture-the-flag-01",
  "team": "blue",
  "assigned_robots": [1, 3]
}
```

The session token is included in subsequent WebSocket connections:
```
WS /api/control?token=abc123
```

**Observer:** No authentication required. The web dashboard (Issue #5) is accessible without a token and shows pose data and camera feed. Sensor data and control require a session.

### Exercise configuration file

An exercise is defined as a YAML file. All coordinates are in **metres relative to AprilTag 101** (arena origin, see Issue #1):

```yaml
exercise:
  id: "capture-the-flag-01"
  name: "Capture the Flag"
  description: "Two teams compete to reach the opponent's flag zone"
  
  arena:
    boundary_tags: [100, 101, 102, 103]
    origin_tag: 101
    obstacles:
      - type: rectangle
        x: 0.4
        y: 0.2
        width: 0.1
        height: 0.05
        label: "barrier"

  teams:
    - name: "blue"
      colour: "#3B82F6"
      flag_zone:
        x: 0.1
        y: 0.3
        radius: 0.05
    - name: "red"
      colour: "#EF4444"
      flag_zone:
        x: 0.7
        y: 0.3
        radius: 0.05

  robots:
    physical:
      - tag_id: 1
        team: "blue"
        sensor_preset: "basic_scout"
      - tag_id: 2
        team: "blue"
        sensor_preset: "basic_scout"
      - tag_id: 3
        team: "red"
        sensor_preset: "basic_scout"
      - tag_id: 4
        team: "red"
        sensor_preset: "basic_scout"
    virtual:
      - team: "blue"
        spawn: { x: 0.15, y: 0.25, theta: 0.0 }
        radius: 0.028
        sensor_preset: "minimal"
      - team: "red"
        spawn: { x: 0.65, y: 0.35, theta: 3.14 }
        radius: 0.028
        sensor_preset: "minimal"

  participants:
    - name: "Alice"
      team: "blue"
      robots: [1]        # assigned by tag_id for physical, by index for virtual
    - name: "Bob"
      team: "blue"
      robots: [2, "v1"]  # v1 = first virtual robot on blue team
    - name: "Carol"
      team: "red"
      robots: [3]
    - name: "Dave"
      team: "red"
      robots: [4, "v2"]

  rules:
    max_linear_speed: 0.15
    max_angular_speed: 1.5
    sensor_config_locked: true    # students cannot modify sensor parameters
    friendly_fire: false          # same-team collision avoidance disabled
    duration_minutes: 10
    
  scoring:
    type: "zone_capture"
    zones:
      - name: "blue_flag"
        team: "red"               # red team scores by reaching blue's flag
        position: { x: 0.1, y: 0.3 }
        radius: 0.05
      - name: "red_flag"
        team: "blue"
        position: { x: 0.7, y: 0.3 }
        radius: 0.05
```

### Exercise lifecycle

```
                    load
    [No Exercise] ────────► [Setup]
                              │
                         start│
                              ▼
                           [Running] ◄──── resume
                              │       │
                        pause │       │ (timer/manual)
                              ▼       │
                           [Paused] ──┘
                              │
                          end │
                              ▼
                          [Complete]
                              │
                         reset│
                              ▼
                        [No Exercise]
```

**States:**

| State | Description |
|-------|-------------|
| `no_exercise` | Server is idle. Free-play mode — any client can control any robot (Issue #3 default behaviour). |
| `setup` | Exercise loaded. Physical robots being matched. Virtual robots spawned. Students can join but cannot send commands yet. |
| `running` | Exercise active. Students can control assigned robots. Timer running (if configured). Sensors active. |
| `paused` | Exercise paused. All robot velocities zeroed. Commands rejected. Sensors continue streaming (frozen poses). Students remain connected. |
| `complete` | Exercise ended. All robots stopped. Scores finalised. Students can review results but not send commands. |

### Admin REST API

All admin endpoints require the admin token.

**Load an exercise:**
```
POST /admin/exercise/load
Content-Type: application/yaml

<exercise YAML body>
```

Or load from a named file on the server:
```
POST /admin/exercise/load
{
  "filename": "capture-the-flag-01.yaml"
}
```

Response:
```json
{
  "exercise_id": "capture-the-flag-01",
  "status": "setup",
  "teams": ["blue", "red"],
  "physical_robots_expected": 4,
  "physical_robots_matched": 2,
  "virtual_robots_spawned": 2,
  "participants_joined": 0
}
```

**Exercise lifecycle control:**
```
POST /admin/exercise/start
POST /admin/exercise/pause
POST /admin/exercise/resume
POST /admin/exercise/end
POST /admin/exercise/reset
```

**Get exercise status:**
```
GET /admin/exercise/status
```

Response:
```json
{
  "exercise_id": "capture-the-flag-01",
  "status": "running",
  "elapsed_seconds": 147,
  "remaining_seconds": 453,
  "teams": {
    "blue": {
      "participants": ["Alice", "Bob"],
      "robots": [1, 2, "v1"],
      "score": 0
    },
    "red": {
      "participants": ["Carol", "Dave"],
      "robots": [3, 4, "v2"],
      "score": 1
    }
  },
  "robots": {
    "1": { "status": "active", "team": "blue", "controller": "Alice" },
    "2": { "status": "active", "team": "blue", "controller": "Bob" },
    "3": { "status": "inactive", "team": "red", "controller": null },
    "4": { "status": "active", "team": "red", "controller": "Dave" }
  }
}
```

**Live adjustments during exercise:**

Assign or reassign a robot:
```
POST /admin/exercise/assign
{
  "robot_id": 3,
  "participant": "Carol",
  "team": "red"
}
```

Add an obstacle mid-exercise:
```
POST /admin/arena/obstacle
{
  "type": "circle",
  "x": 0.3,
  "y": 0.4,
  "radius": 0.05,
  "label": "new_barrier"
}
```

Modify a robot's sensor configuration:
```
POST /admin/robot/configure
{
  "robot_id": 1,
  "sensor_preset": "full_suite"
}
```

Broadcast a message to all participants:
```
POST /admin/broadcast
{
  "message": "Red team captured the flag! 30 seconds remaining."
}
```

**Server → All clients: broadcast notification**
```json
{
  "type": "broadcast",
  "from": "admin",
  "message": "Red team captured the flag! 30 seconds remaining.",
  "timestamp": 1709312445.123
}
```

### Permission enforcement

The exercise manager enforces permissions across all existing APIs:

**Issue #2 (Registry):**
- Virtual robot spawning restricted to admin during an exercise. In free-play mode (no exercise), any client can spawn.

**Issue #3 (Control API):**
- `cmd_vel` rejected with `not_authorized` if the robot is not assigned to the sending client.
- `cmd_vel` rejected with `exercise_paused` if the exercise is paused.
- `cmd_vel` rejected with `exercise_not_running` if the exercise is in setup or complete state.
- Speed limits from the exercise config override per-robot defaults.

**Issue #6 (Sensor API):**
- Sensor subscription restricted to assigned robots and team-shared data.
- Sensor configuration locked if `sensor_config_locked: true` in the exercise config.
- Team subscription restricted to the participant's own team.

### Free-play mode

When no exercise is loaded (`no_exercise` state), the server operates in free-play mode:
- No permissions enforced — any client can control any robot (Issue #3 default behaviour).
- Any client can spawn virtual robots.
- Any client can configure sensors.
- No teams, no scoring.

This mode is used for development, testing, and casual exploration. The permission layer is transparent — the system behaves exactly as defined in Issues #2, #3, and #6 without modifications.

### Exercise templates

The server includes built-in exercise templates for common scenarios:

| Template | Description |
|----------|-------------|
| `free_drive` | No teams, no scoring. All robots freely controllable. For learning basic control. |
| `formation` | Single team. Objective: maintain formation while moving. Scoring based on formation accuracy. |
| `tag` | Two teams. Objective: tag opponents by approaching within contact distance. Tagged robots freeze for N seconds. |
| `capture_the_flag` | Two teams. Objective: reach opponent's flag zone. |
| `patrol` | Single team. Objective: visit all waypoints in order. Timed. |
| `search_and_rescue` | Virtual "victims" placed in arena. Robots must locate them using sensors. |

Templates are YAML files stored in `/server/config/exercises/`. Instructors can modify them or create new ones.

### Scoring engine

The exercise config defines optional scoring rules. The scoring engine runs as part of the exercise manager node and evaluates conditions each tick:

**Scoring types:**

| Type | Description |
|------|-------------|
| `zone_capture` | Score when a robot enters a defined zone |
| `tag` | Score when a robot contacts an opponent (sum-of-radii proximity) |
| `waypoint_sequence` | Score when a robot visits waypoints in order |
| `time_trial` | Score based on time to complete an objective |
| `survival` | Score based on time spent without being tagged |

**Score updates pushed to all clients:**
```json
{
  "type": "score_update",
  "teams": {
    "blue": 2,
    "red": 3
  },
  "event": "Carol captured blue flag",
  "timestamp": 1709312445.123
}
```

Custom scoring logic can be added as Python plugins in `/server/plugins/scoring/`.

### Game effects

The scoring engine can trigger gameplay effects that modify robot behaviour through the command pipeline (Issue #3). These effects are temporary states applied to individual robots.

**Frozen state:**

When a game rule triggers a freeze (e.g. a robot is "tagged" by an opponent), the robot enters a `frozen` state:

- All `cmd_vel` commands for the frozen robot are rejected with a `robot_frozen` error.
- The robot's velocity is immediately set to zero.
- A `robot_frozen` event is pushed to all subscribed clients:

```json
{
  "type": "event",
  "robot_id": 1,
  "event": "robot_frozen",
  "detail": {
    "reason": "tagged",
    "by_robot_id": 3,
    "freeze_duration_seconds": 5,
    "expires_at": 1709312450.123
  }
}
```

- After the freeze duration expires, the robot automatically returns to `active` state. A `robot_unfrozen` event is pushed:

```json
{
  "type": "event",
  "robot_id": 1,
  "event": "robot_unfrozen",
  "detail": {
    "frozen_duration_seconds": 5
  }
}
```

- The web dashboard (Issue #5) shows frozen robots with a distinct visual indicator and countdown timer.

**Tag detection:**

Tagging occurs when an opponent robot comes within contact distance (sum of both radii). The scoring engine evaluates this each tick:

- If `tag_rules.enabled: true` in the exercise config, robot-robot contact between opposing teams triggers a tag.
- The tagged robot (the one that was approached) is frozen for `tag_rules.freeze_seconds`.
- If `friendly_fire: false`, same-team contact does not trigger a tag.
- If `friendly_fire: true`, any contact triggers a tag.

Tag detection is evaluated **after** the collision filter in Issue #3 prevents overlap. A tag is triggered when robots are within contact distance (sum of radii + small threshold, e.g. 5mm), not when they fully overlap.

**Game effect configuration in exercise YAML:**

```yaml
rules:
  tag_rules:
    enabled: true
    freeze_seconds: 5
    friendly_fire: false
    immunity_after_unfreeze_seconds: 2  # brief immunity prevents immediate re-tag
```

**Extensibility:**

Additional game effects can be added as the platform evolves:

| Effect | Description |
|--------|-------------|
| `frozen` | Robot cannot move for N seconds |
| `slowed` | Robot's max speed reduced for N seconds |
| `blinded` | Robot's sensor range reduced to zero for N seconds |
| `boosted` | Robot's max speed increased for N seconds |

These are implemented as temporary modifiers in the command pipeline. Each modifier has a duration and auto-expires.

### Admin dashboard

The web dashboard (Issue #5) includes an admin panel accessible via:
```
http://<server-ip>:8080/admin?token=<admin-token>
```

The admin panel provides:

- **Exercise loader:** Drop-down of available exercise YAML files, or upload a new one.
- **Lifecycle controls:** Start, pause, resume, end, reset buttons.
- **Robot status grid:** All robots with status indicators, team colour, assigned participant, and connection state.
- **Participant list:** Joined students with their assigned robots and connection status.
- **Live arena view:** Same camera + AR overlay as the student dashboard, but with admin annotations (team zones, scoring zones, obstacle labels).
- **Score display:** Real-time team scores.
- **Console:** Log of events (robot connections, boundary contacts, scoring events, exercise state changes).
- **Quick actions:** Assign robot, add obstacle, broadcast message, force-stop a robot.

For Phase 1, the admin panel can be a simple functional interface without polish. Phase 2 (Issue #9) enhances it alongside the full AR dashboard.

### ROS2 integration

The exercise manager is a ROS2 node (`exercise_manager_node`) that:

**Publishes:**
```
/exercise/state           # custom message — current exercise state (setup/running/paused/complete)
/exercise/config          # custom message — full exercise configuration
/exercise/scores          # custom message — current scores
/exercise/broadcast       # std_msgs/String — admin broadcast messages
```

**Subscribes:**
```
/robot_{id}/pose          # monitors all robot poses for scoring zone evaluation
```

**Services:**
```
/exercise/load            # load exercise config
/exercise/start           # start exercise
/exercise/pause           # pause exercise
/exercise/resume          # resume exercise
/exercise/end             # end exercise
/exercise/assign_robot    # assign robot to participant/team
```

Other nodes react to exercise state:
- **API gateway** checks `/exercise/state` before processing `cmd_vel` — rejects commands if paused or not running.
- **Registry node** checks exercise config for spawn permissions.
- **Sensor simulation node** checks exercise config for sensor configuration lock.

### WebSocket exercise events

Exercise state changes and scoring events are pushed to all connected clients:

```json
{
  "type": "exercise_event",
  "event": "started",
  "exercise_id": "capture-the-flag-01",
  "timestamp": 1709312445.123
}
```

```json
{
  "type": "exercise_event",
  "event": "paused",
  "message": "Instructor paused the exercise",
  "timestamp": 1709312500.000
}
```

```json
{
  "type": "exercise_event",
  "event": "ended",
  "result": {
    "winner": "red",
    "scores": { "blue": 2, "red": 3 },
    "duration_seconds": 600
  },
  "timestamp": 1709313045.123
}
```

### Python SDK extensions

```python
from platopod import Arena

# Student joining an exercise
arena = Arena("ws://192.168.4.1:8080/api/control")
session = arena.join_exercise(name="Alice", callsign="alpha-1")
print(f"Team: {session.team}")
print(f"Assigned robots: {session.assigned_robots}")

# Register exercise event callbacks
arena.on_exercise("started", lambda e: print("Go!"))
arena.on_exercise("paused", lambda e: print("Paused"))
arena.on_exercise("ended", lambda e: print(f"Winner: {e.result['winner']}"))
arena.on_broadcast(lambda msg: print(f"Admin says: {msg}"))

# Admin operations (requires admin token)
from platopod import AdminClient

admin = AdminClient("http://192.168.4.1:8080", token="admin-secret")
admin.load_exercise("capture-the-flag-01.yaml")
admin.start()

status = admin.get_status()
print(f"Elapsed: {status.elapsed_seconds}s")

admin.assign(robot_id=3, participant="Carol", team="red")
admin.broadcast("30 seconds remaining!")
admin.pause()
admin.resume()
admin.end()
```

### Acceptance criteria

- [ ] Exercise configuration loaded from YAML file
- [ ] Exercise lifecycle: setup → running → paused → complete with state transitions enforced
- [ ] Admin REST API for lifecycle control, assignment, obstacles, sensor config, and broadcast
- [ ] Admin authentication via static token
- [ ] Student join flow with name/callsign, receiving session token and assignments
- [ ] Permission enforcement: students can only control and subscribe to assigned robots
- [ ] `cmd_vel` rejected with appropriate errors for unauthorised, paused, frozen, or non-running states
- [ ] Sensor configuration lockable by exercise config
- [ ] Free-play mode operates without permissions when no exercise is loaded
- [ ] Team assignment managed by admin only
- [ ] Exercise state published to ROS2 topic, all nodes react accordingly
- [ ] Exercise events and broadcasts pushed to all WebSocket clients
- [ ] Scoring engine evaluates zone capture, tag, waypoint, and time-based objectives
- [ ] Score updates pushed to all clients in real-time
- [ ] Tag detection triggers freeze on opponent contact with configurable duration
- [ ] Frozen robots reject all commands, auto-unfreeze after duration
- [ ] Immunity period after unfreeze prevents immediate re-tag
- [ ] Frozen/unfrozen events pushed to all subscribed clients
- [ ] Game effects rendered on dashboard (Issue #5) with countdown
- [ ] Built-in exercise templates for common scenarios
- [ ] Admin dashboard panel with exercise controls, robot status, participant list, and scoring
- [ ] Python SDK extended with `join_exercise`, exercise event callbacks, and `AdminClient`
- [ ] Exercise reset clears all state and returns to idle
