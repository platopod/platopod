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

**Robot connection:** When an ESP32 robot boots, connects to the WiFi AP, and obtains an IP address, it sends a UDP registration message to the server's `robot_bridge_node`:

```
REG <tag_id> <radius_mm>
```

The server receives the message, attempts to match the tag ID against the list of visible tags, and responds:

- `OK <robot_id>` — registration successful, robot_id assigned
- `DEFERRED` — tag not yet visible to camera, retry later
- `ERR DUPLICATE` — another robot already registered with this tag ID

The UDP registration provides a simple request/response handshake. The firmware retries every 2 seconds until it receives `OK`.

The server also stores the robot's UDP source address (IP:port) so it can send commands (`M`, `S`, `L1`, etc.) back to the specific robot. The `robot_bridge_node` maintains this address mapping and translates ROS2 `cmd_vel` topics into UDP motor commands.

**Note:** micro-ROS integration is planned as future work once ESP-IDF compatibility issues are resolved. The current UDP protocol provides equivalent functionality with simpler implementation.

**Matching flow:**

1. Server launches, camera starts broadcasting, vision node detects AprilTags.
2. Detected robot-range tags are listed as *discovered, unmatched*.
3. A physical robot powers on, connects to the WiFi AP, and sends a UDP registration message `REG <tag_id> <radius_mm>` to the server.
4. The `robot_bridge_node` receives the message and attempts to match the tag ID to a camera-detected tag.
5. On successful match: a robot entry is created in the registry with `type=physical`, the tag's current pose, the robot's radius, and a newly assigned `robot_id`. The server responds `OK <robot_id>` and stores the robot's UDP address for future commands.
6. If the tag ID is not visible to the camera: the server responds `DEFERRED`. The firmware retries every 2 seconds until the tag becomes visible and matching succeeds.
7. If another robot has already registered with the same tag ID: the server responds `ERR DUPLICATE`. This indicates a firmware configuration mistake.
8. If the robot stops responding to heartbeat pings (`H` → no `OK` within timeout): the registry entry is marked `inactive`. If the robot reconnects and re-sends `REG`, it is re-matched and its existing `robot_id` is restored.

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

The HTTP endpoints (`GET /robots`, `POST /robots/spawn`, etc.) are the student-facing API served by the API gateway node. Internally, the robot registry is a ROS2 node exposing ROS2 services. The API gateway translates between HTTP requests and ROS2 service calls.

Physical robot communication uses a separate `robot_bridge_node` that:
- Listens for UDP `REG` messages from ESP32 robots and forwards registration to the registry node.
- Subscribes to ROS2 `/robot_{id}/cmd_vel` topics and translates them to UDP `M <linear> <angular>` commands sent to the robot's stored UDP address.
- Sends periodic heartbeat pings (`H`) to detect disconnected robots.

```
Student (WebSocket) → API Gateway → ROS2 /robot_{id}/cmd_vel → robot_bridge_node → UDP → ESP32
ESP32 → UDP "REG" → robot_bridge_node → registry_node
```

### Firmware requirement

Each ESP32 robot must store its AprilTag ID and radius in non-volatile storage (NVS). This is configured once during initial setup:

```c
// During first-time setup or via serial command
nvs_set_u16(nvs_handle, "tag_id", 3);
nvs_set_u16(nvs_handle, "radius_mm", 28);
```

On boot, the firmware connects to the WiFi AP and sends a UDP registration message to the server:

```c
// Registration message: "REG <tag_id> <radius_mm>"
char msg[32];
snprintf(msg, sizeof(msg), "REG %d %d", tag_id, radius_mm);
sendto(sock, msg, strlen(msg), 0, &server_addr, sizeof(server_addr));
// Wait for "OK <robot_id>" or "DEFERRED", retry every 2 seconds
```

The firmware also implements the UDP command protocol for receiving motor commands (`M`), stop (`S`), LED control (`L1`/`L0`, `C`), display (`D`), and heartbeat (`H`). See `firmware/main/main.c` for the reference implementation.

**Future work:** micro-ROS integration is planned once ESP-IDF compatibility issues are resolved, enabling native ROS2 topic subscription on the ESP32.

### Acceptance criteria

- [ ] Server maintains a robot registry with ID, type, pose, radius, and status
- [ ] Physical robots matched by correlating UDP registration tag ID with camera-detected AprilTags
- [ ] Registration uses UDP request/response (`REG` → `OK`/`DEFERRED`/`ERR`)
- [ ] `robot_bridge_node` stores robot UDP addresses and translates ROS2 cmd_vel to UDP motor commands
- [ ] Unmatched connections handled gracefully (DEFERRED with firmware retry, warning logged)
- [ ] Duplicate tag ID connections rejected with `ERR DUPLICATE`
- [ ] Virtual robots spawned via HTTP API with boundary and collision validation
- [ ] Spawn collision check uses sum of both robots' radii
- [ ] Virtual robots removed via HTTP API; physical robot deletion rejected
- [ ] Instructor can force-reset a physical robot entry
- [ ] Robot list queryable by clients, returning all active robots with radius
- [ ] Heartbeat pings detect disconnected robots, registry status updated to inactive
- [ ] Physical robot reconnection re-matches and restores existing robot_id
- [ ] ESP32 firmware stores tag ID and radius in NVS as u16
- [ ] Future work: micro-ROS integration documented as upgrade path
