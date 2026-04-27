# Tactical Capabilities

Plato Pod's tactical layer turns the platform from a "robots on a tabletop" sandbox into something that can drive a recognisable military exercise — engagement outcomes, casualty effects, communications failures, logistics, civilians, and rules of engagement, all computed by the platform.

The contracts between tactical modules are typed ROS2 messages and services, defined in [`message-catalog.md`](message-catalog.md). The Python implementations described below are the reference implementation; any ROS2-supported language (C++, MATLAB, Rust, Java) can replace any of these nodes.

## Module map

### Algorithm modules (Layer 3 — pure Python today, language-neutral via ROS2 contracts)

| Module | Responsibility |
|---|---|
| `engagement.py` | `evaluate_fire(actor, target, weapon, ...)` — PoK with range falloff, LoS, weather, civilians, suppression. Short-circuits with `rationale=target_already_<status>` if the target is non-operational (destroyed/incapacitated/frozen). |
| `health.py` | `mobility_factor`, `fire_capability`, `apply_damage`, status thresholds (wounded/destroyed/incapacitated) |
| `line_of_sight.py` | `has_line_of_sight(observer, target, terrain, cover_polygons, weather)` — terrain, cover, weather composition |
| `weather.py` | `WeatherState`, `visibility_factor(weather, distance)` — drives sensor and engagement attenuation |
| `comms.py` | `evaluate_comms(unit, team, ...)` — range, LoS, jamming zones, dead zones |
| `logistics.py` | `Logistics` (fuel/ammo/water), `consume_fuel`, `consume_ammo`, `is_immobile`, `can_fire`, `resupply` |
| `roe.py` | `check_fire_roe` — weapons hold/tight/free, civilian proximity, friendly-fire, target identification |
| `behavior.py` | `BehaviorTree` FSM (PATROL → OBSERVE → ENGAGE → RETREAT, INCAPACITATED) for OPFOR units |
| `world_state.py` | `WorldState` dataclass + `world_state_from_config(yaml)` — single source of truth for the world |

### Sensor digital twins (`sensor_plugins/`)

| Module | Responsibility |
|---|---|
| `rangefinder.py` | Laser rangefinder twin (range + bearing to nearest visible target in FOV) |
| `thermal.py` | Thermal imager (FOV sweep, attenuated by weather and cover) |
| `ied_detector.py` | Counter-IED proximity sensor reading `point_sources["ied"]` |
| `df_receiver.py` | RDF receiver returning bearings to RF emitters with distance-dependent noise |
| `uav_camera.py` | Overhead camera footprint sweep for a virtual UAV |
| `gas.py` | MOX sensor twin with ODE response on a `GaussianPlumeField` or GADEN-bridged field |
| `gps.py`, `lidar.py`, `sonar.py`, `fof.py` | Existing pre-tactical sensors — still supported |

The `Robot` dataclass was extended with `health`, `weapons`, `vehicle_role`, `thermal_signature`, and `logistics`. Defaults preserve backwards compatibility — pre-tactical exercises keep working.

## Command pipeline

The pipeline now has seven stages instead of four. The new stages all short-circuit if their condition triggers:

```
state_filter           ← rejects cmd_vel for status in {destroyed, incapacitated, frozen, …}
mobility_filter        ← scales velocity by mobility_factor(health)
fuel_filter            ← rejects movement when logistics tracked and fuel ≤ 0
speed_limit            (existing)
terrain_speed_modifier (existing)
boundary_filter        (existing)
collision_filter       (existing)
```

Each new filter emits a `PipelineEvent` (`command_rejected`, `mobility_reduced`, `out_of_fuel`) so the operator's WebSocket sees what happened.

## ROS2 nodes

The tactical layer is composed of five new nodes plus extensions to two existing ones. Every node is a thin shell over a Layer 3 module — all of them speak the typed message contracts in [`message-catalog.md`](message-catalog.md).

| Node | Responsibility | Subscribes | Publishes / Services |
|---|---|---|---|
| `world_state_node` | Loads exercise YAML once, publishes the world. | (parameter `exercise_file`) | latched: `/world/cover`, `/world/civilians`, `/world/ied_zones`, `/world/ew_emitters`, `/world/jamming`, `/world/dead_zones`, `/world/weather`, `/world/roe`, `/world/weapons` |
| `opfor_node` | Spawns OPFOR units, runs `BehaviorTree.tick()` per unit at 10 Hz. | `/robots/status`, `/arena/model`, `/world/cover`, `/world/weather`, `/world/weapons` | `/robot_<id>/cmd_vel`, `/fire_weapon` (`FireIntent.msg`); spawn RPC |
| `engagement_node` | Central fire evaluator: ROE → `evaluate_fire` → `apply_damage` RPC → emit outcome. | `/robots/status`, `/fire_weapon` (`FireIntent`), `/report_observation` (`Observation`), `/world/cover`, `/world/civilians`, `/world/weather`, `/world/roe`, `/world/weapons` | `/engagement_events` (`EngagementOutcome.msg`) |
| `los_python_node` | Serves `EvaluateLos.srv` via the 2D Python ray model + cover polygons. | `/world/cover` | service `~/evaluate_los` |
| `los_gazebo_node` | Serves `EvaluateLos.srv` via a Gazebo lidar query. | configurable LaserScan topic | service `~/evaluate_los` |
| `registry_node` (extended) | Adds `~/apply_damage` service backed by `health.apply_damage`; `RobotStatus.msg` extended with the new tactical fields. | (existing) | `/robots/status` (extended), `~/apply_damage` |
| `cot_bridge_node` (extended) | Subscribes to `/engagement_events` (typed `EngagementOutcome`); emits CoT engagement alerts and casualty markers. | adds `/engagement_events` | (existing CoT outputs) |
| `api_gateway_node` (extended) | Accepts `fire_weapon` and `report_observation` WebSocket messages, forwards them as typed `FireIntent` / `Observation` topics. | (existing) | adds `/fire_weapon`, `/report_observation` publishers |

## Sensor digital-twin pattern

All new sensor plugins follow the same shape as the existing `gas` plugin:

1. Implement the `SensorPlugin` protocol from `sensor_plugins/base.py`.
2. Read robot pose from `compute(robot, …)`.
3. Pull world data — other robots, `environment.fields[…]` for spatial fields, `environment.point_sources[…]` for discrete entities (IEDs, EW emitters, civilians).
4. Apply a sensor model (range, FOV, accuracy, noise).
5. Return a reading dict.

Each plugin is ~80 lines, has its own test file under `tests/unit/`, and registers itself in `sensor_plugins/__init__.py:ALL_PLUGINS`. The reading dict is wrapped in a typed `SensorReading.msg` envelope by `sensor_engine_node`; consumers in any language read the envelope's typed fields (`robot_id`, `sensor_name`, `timestamp`, `schema_version`) directly and decode the `payload` only when they care about that sensor's specifics. Per-sensor payload schemas (v1.0) are documented in [`message-catalog.md`](message-catalog.md).

## WebSocket protocol additions

The WebSocket API is the operator/dashboard surface of the platform. Internally these messages are translated to typed ROS2 messages on the topics listed above.

| Direction | Message | Internal topic |
|---|---|---|
| Client → Server | `fire_weapon {robot_id, weapon, target_id?, target_position?}` | `/fire_weapon` (`FireIntent.msg`) |
| Client → Server | `report_observation {robot_id, target_id, classification}` | `/report_observation` (`Observation.msg`) |
| Server → Client | `engagement_event {actor, target, weapon, outcome}` | sourced from `/engagement_events` |
| Server → Client | `casualty_update {robot_id, status, health}` | from `/robots/status` deltas |
| Server → Client | `comms_status {robot_id, linked, quality, relay_id}` | (currently a helper format; producer node TBD) |
| Server → Client | `logistics_update {robot_id, fuel, ammo, water}` | (currently a helper format; producer node TBD) |
| Server → Client | `roe_violation {robot_id, rule, severity, description}` | derived from `EngagementOutcome.roe_violations` |

New `inject_event` types (admin-gated): `indirect_fire`, `resupply`, `casevac`, `weather`, `apply_damage`. Validators exist; some handlers still pending — see "Wired-but-inert features" below.

## CoT/ATAK enrichment

`cot_protocol.py` ships these helpers, all called by `cot_bridge_node` when the corresponding event fires:

- `make_engagement_event(actor_uid, target_uid, weapon, outcome, lat, lon, rationale)` → short-lived `b-r-f-h-c` alert at the contact point.
- `make_casualty_event(robot_uid, callsign, lat, lon, status, health)` → `a-h-G-X` (destroyed) or `a-f-G-X` (wounded) marker.
- `make_ied_marker(uid, lat, lon, confidence, label)` → CBRN drawing type (`u-d-c-c`) for IED hazards.
- `make_civilian_marker(uid, lat, lon, label)` → neutral civilian (`a-n-G`).
- `make_remarks_detail(text)` → XML-escaped tap-to-view text.

**Identity derivation.** `cot_bridge_node` resolves the CoT type and callsign
for both the live unit symbol and the casualty marker from the **live**
`team` and `vehicle_role` fields on `RobotStatus.msg` (published by the
registry). When `team` is set, the callsign becomes `BLUE-<id>` /
`RED-<id>` / `GREEN-<id>` and the CoT affiliation flips to friendly
(`a-f-…`) or hostile (`a-h-…`) accordingly. Older YAML overrides keyed
by `tag_id` are still honoured as a fallback for legacy configs that
don't set the live fields per-spawn.

**Casualty visibility.** Once a robot transitions to a non-operational
status, its live symbol stops being redrawn but its record stays in the
bridge's cache so the casualty marker can resolve the correct callsign
and remain visible at the body's last known position.

**Civilians and IED hazards.** `cot_bridge_node` subscribes to
`/world/civilians` and `/world/ied_zones` (latched) and emits an
`a-n-G` marker per civilian (`platopod-civ-<label>`) and a `u-d-c-c`
CBRN-drawing marker per IED hazard (`platopod-ied-<label>`). Stable
UIDs mean ATAK/iTAK update the existing markers rather than spawning
duplicates. Markers republish on the same cadence as the arena
boundary, plus an immediate flush whenever the corresponding world
topic delivers a new sample. The cadet's ied_detector *sensor reading*
remains a separate concern; this is the declarative hazard layer.

**Plume contours (CBRN contamination zones).** `cot_bridge_node` loads
spatial fields from the exercise YAML (gas plumes, etc.) and on a 5 s
timer samples the configured field over the arena bounding box,
extracts polygon contours at each threshold via marching-square cell
unions in `plato_pod.plume_contour`, and emits one CoT shape per
threshold per disconnected component. Default colour ramp follows NATO
CBRN doctrine: red (≥1000) acute / IDLH, orange (≥500) cross-
contamination, yellow (≥100) caution.

Three rendering modes are available — choose by target ATAK client:

| `plume_render` | CoT type | iTAK iOS | ATAK Android | Note |
|---|---|---|---|---|
| `polygon` (default) | `u-d-r` drawing region | ✅ as bbox rectangle | ✅ as polygon outline | Always renders something visible |
| `circle` | `u-d-c-c` CBRN drawing | ⚠️ stored but not drawn | ✅ filled circle | Doctrinally correct (NATO CBRN-1 nested rings) |
| `ellipse` | `u-d-c` with major/minor/angle | ⚠️ stored but not drawn | ✅ filled ellipse | Best fit for wind-blown plume shape |

Per-exercise overrides:

```yaml
exercise:
  virtual_layers:
    gas_sources:
      - { name: "gas", x: 0.6, y: 0.6, release_rate: 200,
          wind_speed: 2.0, wind_direction: 0.0, diffusion_coeff: 0.05 }
  cot_bridge:
    plume_field: "gas"
    plume_render: "polygon"             # polygon | circle | ellipse
    plume_thresholds: [50.0, 300.0, 1500.0]
    plume_colors:                        # optional ARGB per threshold
      - "ffffff00"                       # yellow
      - "ffff8000"                       # orange
      - "ffff0000"                       # red
    plume_grid_size: 80                 # samples on the longer bbox axis
    plume_smooth_m: 0.0                 # auto = grid_resolution × 1.0
    plume_simplify_m: 0.0               # auto = grid_resolution × 0.5
    plume_max_polygons: 3               # cap islands per threshold
```

Both Python `GaussianPlumeField` (lightweight mode) and GADEN-bridged
fields (Gazebo terrain mode) feed the same pipeline, because both
implement the `SpatialField` protocol. The cadet's gas sensor reading
the field at the robot's position is a separate concern; this is the
operator-side visualisation.

**Lifecycle hygiene.** Plume contours are emitted with `<archive>` so
ATAK/iTAK actually renders them — but `cot_bridge_node` tracks every
UID it publishes per category (civilians, IEDs, plume contours) and
sends explicit `t-x-d-d` tombstone events when:
  * a UID drops out of the active set (scenario change, contour shrinks
    below threshold, fragments merge),
  * the node shuts down cleanly (Ctrl-C, ros2 launch SIGTERM).
This means demo iterations don't accumulate stale archived markers in
the operator's TAK app. For one-off cleanup of markers from before this
mechanism was added, see `tools/atak_clear.py`.

## Exercise YAML — what's optional, what's new

Existing exercises keep working unchanged. New optional sections all live in the `exercise:` block:

```yaml
exercise:
  weapons: { … weapon name → spec … }
  weather:  { visibility_m, wind_speed, fog_density, time_of_day, … }
  roe:      { fire_permission, civilian_proximity_m, require_target_id, friendly_fire }
  time_compression: 4.0      # 4× real-time

  robots:
    physical:
      - tag_id: 1
        team: "blue"
        vehicle_role: "soldier"
        weapons: ["M4_carbine"]
        logistics: { fuel: 1.0, ammo: { M4_carbine: 30 } }

  opfor:
    - id: 100
      vehicle_role: "hostile"
      position: [0.6, 0.4]
      weapons: ["AK74"]
      behavior: "patrol_then_defend"
      patrol_route: [...]
      retreat_health_threshold: 0.4

  virtual_layers:
    cover_polygons:       [ { vertices, cover_value, label } ]
    comms_dead_zones:     [ { vertices, label } ]
    jamming_zones:        [ { position, radius_m, strength, label } ]
    civilian_population:  [ { position, movement, count, label } ]
    ied_zones:            [ { position, detectability_radius_m, label } ]
    ew_emitters:          [ { position, frequency_mhz, signal_strength, label } ]

  sensors_available:
    - rangefinder
    - thermal
    - ied_detector
    - df_receiver
```

`world_state_node` parses every section above and publishes them on latched `/world/*` topics. The corresponding YAML loaders (`world_state.world_state_from_config`, `weapons_from_dict`, `roe_from_dict`, `weather_from_dict`, `jamming_zone_from_dict`, `dead_zone_from_dict`) live in `world_state.py` and the individual tactical modules.

## Canonical scenarios

Four scenarios under `config/exercises/` exercise the full pipeline end-to-end:

| Scenario | Capabilities exercised |
|---|---|
| `cbrn-recon-patrol.yaml` | Gas plume + MOX twin, thermal detection, weapons + LoS, casualty under gas exposure, weapons-tight ROE |
| `defensive-position.yaml` | Multi-wave OPFOR, suppress radius, ammo logistics, comms dead zones, indirect-fire injection |
| `cordon-and-search.yaml` | Cover polygons, civilian populations, IED detector + CoT marker, weapons-tight ROE with civilian proximity |
| `joint-coordination.yaml` | UAV camera with virtual aircraft, DF receiver vs EW emitter, GPS jamming zones, multi-platoon coordination |

See [`scenario-authoring.md`](scenario-authoring.md) for how to write your own.

## End-to-end engagement flow

```
Player fires weapon (blue cadet, via dashboard or ATAK):
  WebSocket fire_weapon
    → api_gateway_node._publish_fire_intent
    → /fire_weapon  (typed FireIntent.msg)
    → engagement_node._fire_cb
        ├─ check_fire_roe (civ proximity, friendly, weapons-tight, target ID)
        ├─ evaluate_fire  (PoK, LoS, weather, suppression)
        ├─ /registry_node/apply_damage  → Robot.health, status
        └─ /engagement_events  (typed EngagementOutcome.msg)
            ├─ api_gateway broadcasts → WebSocket engagement_event
            └─ cot_bridge_node → make_engagement_event + make_casualty_event
                → ATAK alert + casualty marker

OPFOR engages cadet (autonomous):
  opfor_node tick:  BehaviorTree.tick(unit, world)
    → Action(cmd_vel, fire_weapon=(target_id, "AK74"))
    → /robot_<id>/cmd_vel  AND  /fire_weapon  (same engagement pipeline as above)

Cadet hit; movement degraded:
  registry_node sets status="wounded", health=0.4
    → next /robots/status broadcast
    → command_pipeline: mobility_filter scales next cmd_vel by 0.4
    → wounded cadet creeps; PipelineEvent("mobility_reduced") to operator WS
```

## Wired-but-inert features

Helpers exist, contracts exist, but no node consumes them yet. Treat these as "API-stable but no producer/consumer wired today":

- `comms.evaluate_comms` — no node calls it on a tick. Suggested next-step home: a 1 Hz timer in `cot_bridge_node` calling per blue unit, gating CoT updates by `linked` and broadcasting `comms_status` over WS.
- `logistics.consume_fuel` — pipeline blocks at `fuel == 0`, but no node decrements fuel based on distance travelled. Suggested home: `command_pipeline.run_pipeline` post-stage.
- `logistics.consume_ammo` / `can_fire` — `engagement_node` doesn't yet check or decrement ammo on fire. Two-line addition to `_fire_cb`.
- `roe_violation` WebSocket broadcast — outcomes carry `roe_violations`, but no separate WS message is emitted; consumers parse it from `engagement_event.outcome`.
- `inject_event` handlers — `resupply`, `indirect_fire`, `weather`, `casevac` validators are in `ws_protocol.py`, but `api_gateway_node._handle_inject_event` doesn't implement them yet. Adding them is the simplest path to runtime world mutation.

## Test coverage

The Layer 3 tactical modules are fully testable without ROS2:

```bash
cd server
PYTHONPATH=src python3 -m pytest tests/unit/ -v
```

Tactical-layer test files: `test_engagement.py`, `test_health.py`, `test_line_of_sight.py`, `test_weather.py`, `test_behavior.py`, `test_comms.py`, `test_logistics.py`, `test_roe.py`, `test_world_state.py`, `test_new_sensor_plugins.py`, plus extensions to `test_command_pipeline.py`, `test_cot_protocol.py`, and `test_virtual_layer_loader.py`.

**Total: 838 unit tests passing, 1 skipped.**
