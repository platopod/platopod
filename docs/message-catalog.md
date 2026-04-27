# Plato Pod Message Catalog — the polyglot contract

Plato Pod's public contract is its set of **ROS2 messages and services**, not any particular language's API. Write a node in **Python (`rclpy`), C++ (`rclcpp`), MATLAB (ROS Toolbox), Rust (`r2r`), or Java (`rcljava`)** — speak the message types defined here and the platform doesn't care which language you chose.

If you can:
1. Build the `plato_pod_msgs` package in your language's ROS2 binding,
2. Subscribe to the topics relevant to your role,
3. Publish/respond on the topics/services relevant to your role,

…you have a complete platform integration. No Python required.

---

## Build the message package

```bash
cd plato_pod_msgs
colcon build --packages-select plato_pod_msgs
source install/setup.bash
# Generated bindings now in:
#   install/plato_pod_msgs/lib/python3.*/site-packages/plato_pod_msgs/   (Python)
#   install/plato_pod_msgs/include/plato_pod_msgs/                       (C++)
#   <your-language-binding>/plato_pod_msgs/                              (others)
```

---

## Topics (publish/subscribe)

### Robot lifecycle

| Topic | Direction | Type | Purpose |
|---|---|---|---|
| `/robots/status` | published by `registry_node` | `plato_pod_msgs/RobotStatusList` | Snapshot of every registered robot — pose, status, team, vehicle_role, health, weapons, thermal_signature. Anyone can subscribe. |
| `/robot_<id>/cmd_vel` | published by any node, consumed by `virtual_sim_node` / `robot_bridge_node` | `geometry_msgs/Twist` | Velocity command for a robot. Pre-pipeline if from outside; post-pipeline if from `api_gateway_node`. |
| `/robot_<id>/pose` | published by `virtual_sim_node` (virtual) or pose providers (physical) | `geometry_msgs/Pose2D` | Current 2D pose. |

### Tactical engagement

| Topic | Direction | Type | Purpose |
|---|---|---|---|
| `/fire_weapon` | published by `opfor_node`, `api_gateway_node`, *or any other actor* | `plato_pod_msgs/FireIntent` | An intent to engage. Fields: `actor_id`, `target_id`, `weapon`, optional `target_x`/`target_y`, `source_seq`. |
| `/report_observation` | published by `api_gateway_node` (or any analyst tool) | `plato_pod_msgs/Observation` | Clears a target for ROE under WEAPONS TIGHT. |
| `/engagement_events` | published by `engagement_node` | `plato_pod_msgs/EngagementOutcome` | Resolved engagement: hit/blocked/damage/distance/PoK/rationale/violations/suppressed. |

### World state (latched — last sample delivered to late subscribers)

All published by `world_state_node`. Subscribe with `DurabilityPolicy.TRANSIENT_LOCAL` to receive the latest sample on connect.

| Topic | Type | Contents |
|---|---|---|
| `/world/cover` | `plato_pod_msgs/WorldCover` | All cover polygons (vertices, cover_value 0..1, label) |
| `/world/civilians` | `plato_pod_msgs/CivilianList` | Civilian groups (position, movement, count, label) |
| `/world/ied_zones` | `plato_pod_msgs/IedZoneList` | IED hazards (position, detectability_radius, label) |
| `/world/ew_emitters` | `plato_pod_msgs/EwEmitterList` | RF emitters (position, frequency, signal strength) |
| `/world/jamming` | `plato_pod_msgs/JammingZoneList` | Circular jamming zones (position, radius, strength) |
| `/world/dead_zones` | `plato_pod_msgs/DeadZoneList` | Comms dead-zone polygons |
| `/world/weather` | `plato_pod_msgs/WeatherState` | Visibility, wind, fog, time of day |
| `/world/roe` | `plato_pod_msgs/RoeRules` | Active rules of engagement |
| `/world/weapons` | `plato_pod_msgs/WeaponCatalog` | All weapon specs (name, range, PoK, damage, suppress radius) |

### Sensor readings

| Topic | Direction | Type | Purpose |
|---|---|---|---|
| `/robot_<id>/sensors/<name>` | published by `sensor_engine_node` | `plato_pod_msgs/SensorReading` | Typed envelope: `robot_id`, `sensor_name`, `timestamp`, `schema_version`, `payload_format`, `payload`. Decode `payload` only when you care about that sensor's schema. |

### Arena geometry

| Topic | Type |
|---|---|
| `/arena/model` | `plato_pod_msgs/ArenaModel` |
| `/tags/detections` | `plato_pod_msgs/TagDetections` |

---

## Services (request/response)

| Service | Provider | Type | Purpose |
|---|---|---|---|
| `/registry_node/spawn_virtual` | `registry_node` | `plato_pod_msgs/SpawnVirtual` | Spawn a virtual robot. Optional `team`, `vehicle_role`, `health`, `weapons[]`. |
| `/registry_node/remove_robot` | `registry_node` | `plato_pod_msgs/RemoveRobot` | Remove a virtual robot. |
| `/registry_node/reset_robot` | `registry_node` | `plato_pod_msgs/ResetRobot` | Reset a physical robot's status. |
| `/registry_node/list_robots` | `registry_node` | `plato_pod_msgs/ListRobots` | Get a snapshot. |
| `/registry_node/register_physical` | `registry_node` | `plato_pod_msgs/RegisterPhysical` | Register a newly-discovered physical robot. |
| `/registry_node/apply_damage` | `registry_node` | `plato_pod_msgs/ApplyDamage` | Subtract damage from a robot's health and update status. |
| `/los_*_node/evaluate_los` | `los_python_node` *or* `los_gazebo_node` | `plato_pod_msgs/EvaluateLos` | Line-of-sight query. Same contract, two backends — pick by launching the corresponding node. |

---

## Message reference (field-level)

Schemas live in `server/plato_pod_msgs/msg/` and `server/plato_pod_msgs/srv/`. The most-used contracts:

### `FireIntent.msg`
```
int32 actor_id              # robot_id of the firing unit; -1 for indirect fire
int32 target_id             # robot_id of the target; -1 if position-only
string weapon               # weapon name; must exist in the weapon catalog
bool has_target_position
float64 target_x
float64 target_y
uint64 source_seq           # opaque correlation id; echoed in EngagementOutcome
```

### `EngagementOutcome.msg`
```
int32 actor_id
int32 target_id
string weapon
uint64 source_seq

bool fired
bool hit
bool blocked
float64 damage
float64 distance_m
float64 effective_pok

string rationale
bool civilian_violation
string[] civilians_at_risk
int32[] suppressed_targets
string[] roe_violations
```

### `Observation.msg`
```
int32 actor_id
int32 target_id
string classification       # "hostile" | "friendly" | "civilian" | "unknown"
float64 confidence          # 0..1
```

### `SensorReading.msg`
```
int32 robot_id
string sensor_name          # gas, thermal, lidar_2d, rangefinder, …
float64 timestamp           # unix epoch seconds
string schema_version       # sensor-specific schema version
string payload_format       # "json" | "cbor" | "msgpack"
string payload              # serialized payload in the declared format
bool typed_published        # true ⇒ a typed sub-msg was published in parallel
```

### `WeaponSpec.msg`
```
string name
float64 max_range_m
float64 base_pok_at_100m
float64 falloff_per_100m
float64 damage
int32 ammo_capacity
float64 suppress_radius_m   # 0 = no area effect
```

For everything else, the `.msg` files are the source of truth.

---

## Sensor payload schemas (v1.0)

The `SensorReading.payload` JSON for each sensor type. C++/MATLAB nodes can use any JSON parser; payload size is small so the per-call cost is negligible.

| sensor_name | payload schema (v1.0) |
|---|---|
| `gas` | `{ "concentration": float, "raw_resistance": float, "equilibrium_resistance": float }` |
| `gps` | `{ "lat": float, "lon": float, "accuracy": float, "fix": str }` |
| `lidar_2d` | `{ "ranges": [float], "segment_angle_deg": float, "range_max": float }` |
| `sonar` | `{ "ranges": [float], "segment_angle_deg": float, "range_max": float }` |
| `rangefinder` | `{ "range_m": float|null, "target_id": int|null, "accuracy_m": float, "reason": str }` |
| `thermal` | `{ "detections": [{robot_id, bearing_rad, range_m, intensity}], "max_range_m": float, "fov_deg": float }` |
| `ied_detector` | `{ "detections": [{position, true_position, distance_m, confidence, label}], "default_detection_radius_m": float }` |
| `df_receiver` | `{ "bearings": [{bearing_deg, true_bearing_deg, signal_strength, frequency_mhz, label}], "max_range_m": float }` |
| `uav_camera` | `{ "detections": [{robot_id, team, vehicle_role, position, local_position, confidence}], "footprint_width_m": float, "footprint_height_m": float, "uav_position": [x,y], "uav_heading_rad": float }` |
| `imu` | `{ "linear_acceleration": {x,y,z}, "angular_velocity": {x,y,z}, "orientation": {x,y,z,w} }` |

When a schema changes incompatibly, bump `schema_version` (e.g. `"2.0"`) and add a fallback path in your consumer.

---

## Writing a new node — minimum viable polyglot example

### C++ (rclcpp) — receive engagement events and log them

```cpp
#include <rclcpp/rclcpp.hpp>
#include <plato_pod_msgs/msg/engagement_outcome.hpp>

class EngagementLogger : public rclcpp::Node {
 public:
  EngagementLogger() : Node("engagement_logger") {
    sub_ = create_subscription<plato_pod_msgs::msg::EngagementOutcome>(
      "/engagement_events", 10,
      [this](plato_pod_msgs::msg::EngagementOutcome::SharedPtr msg) {
        RCLCPP_INFO(get_logger(),
          "actor=%d target=%d weapon=%s hit=%d damage=%.2f rationale=%s",
          msg->actor_id, msg->target_id, msg->weapon.c_str(),
          msg->hit, msg->damage, msg->rationale.c_str());
      });
  }
 private:
  rclcpp::Subscription<plato_pod_msgs::msg::EngagementOutcome>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EngagementLogger>());
  rclcpp::shutdown();
}
```

### MATLAB — fire a weapon from a tactical-doctrine model

```matlab
node = ros2node('matlab_doctrine');
pub  = ros2publisher(node, '/fire_weapon', 'plato_pod_msgs/FireIntent');

intent = ros2message(pub);
intent.actor_id   = int32(7);
intent.target_id  = int32(102);
intent.weapon     = 'M4_carbine';
intent.has_target_position = false;
intent.source_seq = uint64(now * 1e6);
send(pub, intent);
```

### Rust (r2r) — replace the OPFOR brain

```rust
let node = r2r::Node::create(ctx, "rust_opfor", "")?;
let mut pub = node.create_publisher::<plato_pod_msgs::msg::FireIntent>("/fire_weapon", QosProfile::default())?;

// behaviour tree decides to fire …
let intent = plato_pod_msgs::msg::FireIntent {
    actor_id: 100, target_id: 1, weapon: "AK74".into(),
    has_target_position: false,
    target_x: 0.0, target_y: 0.0,
    source_seq: seq,
};
pub.publish(&intent)?;
```

In all three cases the platform — registry, world state, engagement evaluator, CoT bridge, dashboard — sees the message, evaluates it, and reacts identically. The Python `opfor_node` is one implementation of "OPFOR brain"; replace it with any of the above and the rest of the system doesn't notice.

---

## Where the existing Python implementations live

For reference (or to read as a worked example before re-implementing in another language):

| Role | Python module | Pattern |
|---|---|---|
| Engagement evaluator | `engagement_node.py` (shell) + `engagement.py` (math) | Subscribe `FireIntent` → call `evaluate_fire` → publish `EngagementOutcome` + RPC `apply_damage` |
| OPFOR brain | `opfor_node.py` + `behavior.py` | Subscribe `RobotStatusList`, `WorldCover`, `WeatherState`, `WeaponCatalog` → tick FSM → publish `Twist` and `FireIntent` |
| World state owner | `world_state_node.py` + `world_state.py` | Load YAML → publish all `/world/*` topics (latched) |
| LoS service | `los_python_node.py` (Python 2D) **or** `los_gazebo_node.py` (Gazebo lidar) | Same `EvaluateLos.srv` contract |
| CoT/ATAK bridge | `cot_bridge_node.py` + `cot_protocol.py` | Subscribe everything → emit CoT XML |
| Sensor compute | `sensor_engine_node.py` + `sensor_plugins/*` | Per-robot tick → publish `SensorReading` |

---

## Versioning policy

- **Adding** a field to a `.msg`: backwards-compatible. Old consumers ignore the new field.
- **Removing** or **renaming** a field: incompatible. Bump the message name (e.g. `FireIntentV2.msg`) and run both for one release before retiring the old one.
- **Sensor payload schema** changes: bump `schema_version` and have producers tag accordingly.
- ROS2 message generation is fast and non-breaking when fields are added at the end.

The catalog is small enough today that everything is on `v1`. Treat that as the stable baseline.
