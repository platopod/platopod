## Robot Node Spawning and Discovery

Establish how physical and virtual robots become registered in the server's robot registry. Once registered, both types are indistinguishable to client applications — each has a unique ID, a type, and a live pose.

### Robot registry

The server maintains a robot registry — a list of all active robots with the following fields per entry:

| Field | Description |
|-------|-------------|
| `robot_id` | Unique identifier (sequential integer, assigned at registration) |
| `type` | `physical` or `virtual` |
| `pose` | Current (x, y, θ) in arena coordinates |
| `radius` | Robot radius in metres (default 0.028) |
| `status` | `active`, `inactive`, or `error` |
| `tag_id` | AprilTag ID (physical robots only) |

### Physical robot discovery

Physical robot registration involves matching two independent data streams:

**Camera stream:** The vision node detects AprilTag markers and maintains a list of visible tags with their poses. Tags in the robot ID range (e.g. 0–15) that are not yet matched to an online robot are listed as *discovered but unmatched*.

**Robot connection:** When an ESP32 robot boots and connects to the micro-ROS agent, it registers with the server via a micro-ROS **service call** (not a topic publish) containing its AprilTag ID (stored in NVS flash as `u16` during manufacturing/setup) and its robot radius. The server receives the request, attempts to match it against the list of visible tags, and returns a response indicating success or failure.

Using a service call rather than a topic publish ensures reliable delivery — UDP-based topic publishes can be silently lost, whereas a service call provides a request/response handshake with retry logic.

**Matching flow:**

1. Server launches, camera starts broadcasting, vision node detects AprilTags.
2. Detected robot-range tags are listed as *discovered, unmatched*.
3. A physical robot powers on, connects to WiFi, registers with the micro-ROS agent, and sends a registration service call containing its tag ID and radius.
4. Server matches the provided tag ID to the corresponding detected tag.
5. On successful match: a robot entry is created in the registry with `type=physical`, the tag's current pose, the robot's radius, and a newly assigned `robot_id`. The `robot_id` is returned to the firmware in the service response.
6. If the tag ID is not visible to the camera: the service call returns a "deferred" status. The firmware retries periodically (e.g. every 2 seconds) until the tag becomes visible and matching succeeds.
7. If another robot has already registered with the same tag ID: the service call is rejected with a "duplicate tag ID" error. This indicates a firmware configuration mistake.
8. If the robot disconnects (micro-ROS heartbeat lost): the registry entry is marked `inactive`. If the robot reconnects and re-sends the registration service call, it is re-matched and its existing `robot_id` is restored.

### Virtual robot spawning

Virtual robots are created via the student-facing HTTP API (served by the API gateway node, which translates between HTTP and the internal ROS2 service):

```
POST /robots/spawn
{
  "x": 0.2,
  "y": 0.3,
  "theta": 1.57,
  "radius": 0.028
}
```

If `radius` is omitted, the default Plato Pod radius (0.028m) is used.

**Spawn validation:**
- The requested pose (accounting for the specified radius) must be fully inside the arena boundary (Issue #1).
- The requested pose must not overlap with any existing robot (physical or virtual). Minimum clearance is the sum of both robots' radii (centre-to-centre distance ≥ radius_A + radius_B).
- If validation fails, the spawn request is rejected with an error indicating the reason (`out_of_bounds` / `collision` with the conflicting `robot_id`).
- On success, a registry entry is created with `type=virtual` and a newly assigned `robot_id`.

### Virtual robot removal

```
DELETE /robots/{robot_id}
```

Only virtual robots can be removed via API. Attempting to delete a physical robot returns an error. Physical robots are removed from the registry only when they disconnect.

An instructor can force-reset a physical robot's registry entry (e.g. when the robot is physically present but firmware has crashed and is unresponsive):

```
POST /robots/{robot_id}/reset
```

This marks the entry as `inactive` and releases the tag ID for re-matching when the robot reconnects.

### Listing available robots

Clients query the registry to see all active robots:

```
GET /robots
```

Returns a list of all registered robots with their ID, type, pose, radius, and status. The response does not distinguish between physical and virtual robots in terms of controllability — both are equally available for control.

### Architecture note

The HTTP endpoints (`GET /robots`, `POST /robots/spawn`, etc.) are the student-facing API served by the API gateway node. Internally, the robot registry is a ROS2 node exposing ROS2 services. The API gateway translates between HTTP requests and ROS2 service calls. Physical robot registration bypasses the HTTP layer entirely — it uses micro-ROS service calls directly between the ESP32 firmware and the registry node.

### Firmware requirement

Each ESP32 robot must store its AprilTag ID in non-volatile storage (NVS). This is configured once during initial setup:

```c
// During first-time setup or via serial command
nvs_set_u16(nvs_handle, "tag_id", 3);
nvs_set_u16(nvs_handle, "radius_mm", 28);
```

The firmware sends a registration service call to the server upon connecting to the micro-ROS agent. If the server responds with "deferred" (tag not yet visible), the firmware retries every 2 seconds.

### Acceptance criteria

- [ ] Server maintains a robot registry with ID, type, pose, radius, and status
- [ ] Physical robots matched by correlating micro-ROS service call tag ID with camera-detected AprilTags
- [ ] Registration uses micro-ROS service call (request/response) for reliable delivery
- [ ] Unmatched connections handled gracefully (deferred with firmware retry, warning logged)
- [ ] Duplicate tag ID connections rejected with clear error
- [ ] Virtual robots spawned via HTTP API with boundary and collision validation
- [ ] Spawn collision check uses sum of both robots' radii
- [ ] Virtual robots removed via HTTP API; physical robot deletion rejected
- [ ] Instructor can force-reset a physical robot entry
- [ ] Robot list queryable by clients, returning all active robots with radius
- [ ] Physical robot disconnection updates registry status to inactive
- [ ] Physical robot reconnection re-matches and restores existing robot_id
- [ ] ESP32 firmware stores tag ID and radius in NVS as u16
