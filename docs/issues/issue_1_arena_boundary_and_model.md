## Dynamic Arena Boundary and Arena Model

Define the arena dynamically using dedicated AprilTag markers placed at the arena corners. The vision node detects these tags and computes the convex hull to establish the arena boundary polygon. Beyond the boundary, the arena model includes static obstacles and scoring zones, forming a single authoritative representation of the environment used by all server components.

### Coordinate system

All positions in the system are expressed in **metres relative to AprilTag 101**, which serves as the arena origin (0, 0). The orientation of Tag 101's natural axes defines the arena axes (X increases to the right, Y increases upward when viewed from the camera).

The coordinate system is grounded in physical reality:
- The camera is calibrated and knows the physical size of each AprilTag (50mm).
- The vision node computes the pose of each tag in metres relative to the camera.
- Tag 101 is designated as the origin. All other tags, robot poses, obstacle positions, and scoring zones are expressed relative to Tag 101.

This coordinate system is shared across all components: boundary filter, collision filter, sensor simulation, virtual robot kinematics, dashboard overlay, and scoring engine.

### Arena boundary

Boundary corner AprilTags (e.g. IDs 100–103) define the arena perimeter. The vision node detects these tags and computes the **convex hull** to establish the boundary polygon.

- Boundary is recomputed whenever corner tags are detected, enabling live arena reconfiguration (move a tag, arena shape updates).
- Reserve a separate AprilTag ID range for boundary markers (e.g. 100+) to distinguish them from robot tags (e.g. 0–15).
- The boundary polygon supports future extension to concave shapes (e.g. L-shaped arenas).

### Corner tag occlusion

Robot centre must remain at least one robot radius inside the boundary to prevent the robot from occluding corner markers. If a corner marker is temporarily occluded, the server retains its last known position. If a corner marker remains undetected for more than N seconds (configurable, default 10s), the server logs a warning but continues using the last known position.

### Arena model

The arena model is a single authoritative data structure describing the complete environment. It is published as a ROS2 topic and served via a REST endpoint, consumed by all server components:

| Consumer | Uses arena model for |
|----------|---------------------|
| Boundary filter (Issue #3) | Preventing robots from crossing perimeter AND obstacles |
| Collision filter (Issue #3) | Robot-robot clearance checks |
| Sensor simulation (Issue #6) | Ray casting for Lidar, Sonar, FoF |
| Virtual simulation (Issue #4) | Safety-net pose clamping |
| Dashboard (Issue #5) | Rendering boundary, obstacles, and zones as overlays |
| Scoring engine (Issue #7) | Evaluating zone capture conditions |

The arena model contains:

```yaml
arena_model:
  origin_tag: 101
  boundary:
    tag_ids: [100, 101, 102, 103]
    polygon: [[0, 0], [0.84, 0], [0.84, 0.59], [0, 0.59]]  # computed from tags
  obstacles: []      # static obstacles (walls, barriers)
  zones: []          # scoring/game zones
```

### Static obstacles

Obstacles are physical or virtual barriers within the arena. They are defined in the exercise configuration (Issue #7) or via the admin API, in metres relative to Tag 101.

**Supported obstacle types:**

```yaml
obstacles:
  # Rectangle (convenience syntax, expanded to 4 vertices internally)
  - type: rectangle
    x: 0.35          # centre x in metres from Tag 101
    y: 0.15          # centre y in metres from Tag 101
    width: 0.15      # metres
    height: 0.03     # metres
    rotation: 0.0    # radians, optional, default 0
    label: "wall_north"

  # Arbitrary polygon (general syntax)
  - type: polygon
    vertices:
      - [0.30, 0.25]
      - [0.35, 0.20]
      - [0.40, 0.25]
    label: "triangle_barrier"

  # Circle (approximated as N-gon for ray casting)
  - type: circle
    x: 0.5
    y: 0.3
    radius: 0.03
    label: "pillar"
```

Internally, all obstacle types are stored as polygons (rectangles expanded to 4 vertices, circles approximated as 16-gon or configurable).

### Obstacle validation

When obstacles are loaded, the server validates each one against the arena boundary:

| Case | Action |
|------|--------|
| All vertices inside the boundary hull | **Accept** — normal case |
| Some vertices outside the boundary hull | **Warning** — obstacle clipped to arena hull, instructor notified |
| All vertices outside the boundary hull | **Error** — obstacle rejected, not added to arena model |
| Obstacle overlaps a robot's current position | **Warning** — robot may be trapped |
| Obstacle overlaps another obstacle | **Warning** — redundant but not harmful, accepted |
| Obstacle overlaps a scoring zone | **Warning** — may be intentional, accepted |

When the arena boundary changes dynamically (corner tags moved), obstacles are re-validated. If a previously valid obstacle now extends outside the new boundary, a warning is pushed to the admin.

### Scoring zones

Scoring zones are regions within the arena used by the scoring engine (Issue #7). They are defined in the exercise configuration:

```yaml
zones:
  - name: "objective"
    type: circle
    x: 0.42
    y: 0.3
    radius: 0.06
    team: null         # any team can capture
    hold_time_seconds: 10

  - name: "blue_base"
    type: rectangle
    x: 0.05
    y: 0.3
    width: 0.08
    height: 0.08
    team: "blue"       # belongs to blue team
```

Zones do not block robot movement — they are detection regions only. Robots can freely enter and exit zones. The scoring engine evaluates robot presence within zones each tick.

Zones follow the same validation rules as obstacles — they must be within the arena boundary.

### Server-side velocity filtering (boundary and obstacles)

The server maintains the arena model and each robot's current pose. When a `cmd_vel` command is received for a robot (Issue #3), the server predicts the resulting position over a short lookahead horizon. If the predicted position (plus robot radius as safety margin) would cross:

- The **arena boundary perimeter** — command is modified or rejected.
- Any **static obstacle** — command is modified or rejected.

Commands that move the robot away from or parallel to a boundary or obstacle are passed through normally. A `boundary_contact` event is pushed to the client indicating which element was hit (perimeter edge or obstacle label).

### ROS2 integration

**Published topics:**
```
/arena/model              # custom message — full arena model (boundary + obstacles + zones)
/arena/boundary           # geometry_msgs/PolygonStamped — boundary polygon only (convenience)
/arena/homography         # custom message — camera-to-arena transform matrix
```

**REST endpoints:**
```
GET /arena/model          # full arena model as JSON
GET /arena/homography     # homography matrix for dashboard overlay (Issue #5)
```

**Admin endpoints (Issue #7):**
```
POST /admin/arena/obstacle      # add obstacle mid-exercise
DELETE /admin/arena/obstacle/{label}  # remove obstacle
POST /admin/arena/zone          # add scoring zone
DELETE /admin/arena/zone/{name}  # remove scoring zone
```

When the arena model changes (boundary recomputed, obstacle added/removed, zone modified), the updated model is published to the ROS2 topic and pushed to all WebSocket clients:

```json
{
  "type": "arena_update",
  "boundary": [[0, 0], [0.84, 0], [0.84, 0.59], [0, 0.59]],
  "obstacles": [
    { "type": "polygon", "vertices": [[0.275, 0.135], [0.425, 0.135], [0.425, 0.165], [0.275, 0.165]], "label": "wall_north" }
  ],
  "zones": [
    { "type": "circle", "x": 0.42, "y": 0.3, "radius": 0.06, "name": "objective" }
  ],
  "timestamp": 1709312445.123
}
```

### Acceptance criteria

- [ ] Tag 101 established as arena coordinate origin, all positions in metres relative to it
- [ ] Corner tags detected and convex hull boundary computed
- [ ] `cmd_vel` commands filtered based on predicted boundary AND obstacle crossing
- [ ] Robot radius accounted for in all clearance checks
- [ ] Occluded corner tags use cached position with staleness warning
- [ ] Arena shape updates dynamically when corner tags are moved
- [ ] Static obstacles defined as rectangles, polygons, or circles in arena coordinates
- [ ] All obstacle types stored internally as polygons
- [ ] Obstacle validation on load: inside-hull check with accept/warn/error
- [ ] Obstacle re-validation on boundary change with admin notification
- [ ] Scoring zones defined and validated within boundary
- [ ] Zones do not block movement — detection regions only
- [ ] Arena model published as single ROS2 topic consumed by all nodes
- [ ] Arena model served via REST endpoint for dashboard
- [ ] Arena model updates pushed to WebSocket clients
- [ ] Admin can add/remove obstacles and zones mid-exercise
- [ ] Student API returns boundary-hit flag with element identification (perimeter/obstacle label)
