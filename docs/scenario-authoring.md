# Authoring a Scenario

This walks through writing a Plato Pod scenario YAML from scratch. It assumes you've read `tactical-capabilities.md` for what each module does.

## Skeleton

The minimum viable exercise is just an arena and one robot:

```yaml
exercise:
  id: "my-scenario"
  name: "My Scenario"
  arena:
    boundary:
      - [0.0, 0.0]
      - [0.852, 0.0]
      - [0.852, 0.723]
      - [0.0, 0.723]
  robots:
    physical:
      - tag_id: 1
        team: "blue"
```

Drop this in `config/exercises/my-scenario.yaml` and it loads.

## Mapping desk-arena to real terrain

Set the `geo` block to anchor your arena to the real world. The `scale_factor` decides how large the arena appears on a map — `150.0` means 1m on the desk = 150m on Dowsett Field.

```yaml
exercise:
  geo:
    origin_lat: -35.293935       # SW corner of Dowsett Field
    origin_lon: 149.166421
    origin_alt: 580.0
    rotation_deg: 0.0
    scale_factor: 150.0
```

All other coordinates in the YAML are in **arena metres** (origin = AprilTag 101 by default). The CoT bridge converts them to lat/lon at publish time.

## Adding the tactical layer

Each block below is independent — add only the ones you need.

### Weapons

```yaml
weapons:
  M4_carbine:
    max_range_m: 400
    base_pok_at_100m: 0.6
    falloff_per_100m: 0.15
    damage: 1.0
    ammo_capacity: 30
  M249_SAW:
    max_range_m: 600
    base_pok_at_100m: 0.5
    falloff_per_100m: 0.10
    damage: 1.0
    ammo_capacity: 200
    suppress_radius_m: 8.0   # area effect on nearby units
```

`base_pok_at_100m` is probability of a kill at 100m; `falloff_per_100m` is subtracted from the PoK every additional 100m. PoK is also scaled by the actor's `fire_capability(health)` and any cover attenuation between actor and target.

### Rules of engagement

```yaml
roe:
  fire_permission: "weapons_tight"   # weapons_hold | weapons_tight | weapons_free
  civilian_proximity_m: 10
  require_target_id: true
  friendly_fire: "critical"          # warning | violation | critical
```

`weapons_hold` blocks all fire. `weapons_tight` requires the target to be in `cleared_targets` (set by a `report_observation` WebSocket call). `weapons_free` allows engagement of any non-friendly target.

### Weather

```yaml
weather:
  visibility_m: 800
  wind_speed: 2.0
  wind_direction: 0.0    # radians, 0 = blowing toward +x
  fog_density: 0.0
  time_of_day: 14.0
```

Visibility caps the range at which sensors and engagements have any chance of resolving. Fog further attenuates intensity.

### Robot loadouts

```yaml
robots:
  physical:
    - tag_id: 1
      team: "blue"
      vehicle_role: "soldier"
      weapons: ["M4_carbine"]
      sensor_preset: "infantry"
      logistics:
        fuel: 1.0
        ammo: { M4_carbine: 60 }
```

Setting `logistics:` enables fuel/ammo tracking for that unit. Without it, the robot has unlimited consumables (the platform's behavior pre-tactical-extensions). The `fuel_filter` pipeline stage only blocks units that have logistics tracking.

### OPFOR (autonomous virtual hostiles)

```yaml
opfor:
  - id: 100
    vehicle_role: "hostile"
    position: [0.6, 0.4]
    weapons: ["AK74"]
    behavior: "patrol_then_defend"
    patrol_route:
      - [0.55, 0.40]
      - [0.70, 0.50]
    detection_range_m: 0.4       # arena metres
    engagement_range_m: 0.3
    retreat_health_threshold: 0.4
    enemy_teams: ["blue"]        # default: anything not on this unit's team
```

The OPFOR FSM patrols a route, transitions to OBSERVE on enemy sighting, ENGAGE within `engagement_range_m`, and RETREAT when `health < retreat_health_threshold`. To make a unit static (e.g. a sniper), set `patrol_route: []`.

### Cover polygons

```yaml
virtual_layers:
  cover_polygons:
    - vertices:
        - [0.40, 0.30]
        - [0.55, 0.30]
        - [0.55, 0.42]
        - [0.40, 0.42]
      cover_value: 1.0       # 0=transparent, 1=opaque
      label: "building_alpha"
```

Cover polygons block line-of-sight (engagements, thermal, rangefinder) and comms when configured to do so. `cover_value` scales the attenuation: `0.6` is a wood line, `1.0` is solid masonry.

### IED zones, EW emitters, civilians

```yaml
virtual_layers:
  ied_zones:
    - position: [0.48, 0.38]
      detectability_radius_m: 0.05
      label: "device_alpha"

  ew_emitters:
    - position: [0.78, 0.40]
      frequency_mhz: 144.0
      signal_strength: 1.0
      label: "comms_emitter"

  civilian_population:
    - position: [0.42, 0.40]
      movement: "stationary"
      count: 5
      label: "market"
```

These populate `EnvironmentContext.point_sources["ied" | "ew_emitters" | "civilian"]`, consumed by the corresponding sensor plugins and by `engagement_node` for civilian-proximity ROE checks.

### Comms degradation

```yaml
virtual_layers:
  comms_dead_zones:
    - vertices: [...]
      label: "ridge_shadow"

  jamming_zones:
    - position: [0.65, 0.45]
      radius_m: 0.20
      strength: 0.8
      label: "GPS_jammer_alpha"
```

A unit inside a dead zone has no comms. Jamming zones degrade quality multiplicatively (`strength=1.0` is total denial inside the radius).

### Available sensors

```yaml
sensors_available:
  - thermal
  - rangefinder
  - ied_detector
  - df_receiver
  - uav_camera
  - gps
  - gas
```

This whitelists which sensor plugins the sensor engine will instantiate. Per-robot `sensor_preset` (defined in `config/sensor_presets.yaml`) further restricts what each unit carries.

## Testing your scenario

1. **Schema-only check** — load the YAML and parse it without launching the platform:
   ```bash
   python3 -c "import yaml; print(yaml.safe_load(open('config/exercises/my-scenario.yaml')))"
   ```
2. **WorldState check** — load the tactical world definition and inspect what `world_state_node` will publish:
   ```bash
   PYTHONPATH=server/src python3 -c "
   from plato_pod.world_state import load_world_state
   ws = load_world_state('config/exercises/my-scenario.yaml')
   print(f'cover={len(ws.cover)} civilians={len(ws.civilians)} '
         f'ied={len(ws.ied_zones)} ew={len(ws.ew_emitters)} '
         f'jamming={len(ws.jamming_zones)} dead={len(ws.dead_zones)} '
         f'weapons={len(ws.weapons)} roe={ws.roe.fire_permission}')
   "
   ```
3. **Spatial-fields check** — load gas plumes and elevation grids:
   ```bash
   PYTHONPATH=server/src python3 -c "
   import yaml
   from plato_pod.virtual_layer_loader import load_virtual_layers
   cfg = yaml.safe_load(open('config/exercises/my-scenario.yaml'))
   env = load_virtual_layers(cfg)
   print('fields:', list(env.fields))
   print('point_sources:', {k: len(v) for k, v in env.point_sources.items()})
   "
   ```
4. **Full launch** — easiest path is the helper script (mounted into the container as `/ros2_ws/tools/run_demo.sh`):
   ```bash
   /ros2_ws/tools/run_demo.sh /ros2_ws/config/exercises/my-scenario.yaml 192.168.1.233
   ```
   Spins up the full stack: arena_model, registry, world_state, virtual_sim, los_python, engagement, cot_bridge. Override geo origin / scale / ports via env vars (`GEO_LAT`, `SCALE`, `COT_PORT`, etc.).

   Stop with: `/ros2_ws/tools/stop_demo.sh`.

   For finer control, launch nodes individually. `world_state_node` should start first so its latched topics are available when consumers connect:
   ```bash
   EX=config/exercises/my-scenario.yaml

   ros2 launch plato_pod world_state.launch.py exercise_file:=$EX &
   ros2 launch plato_pod simulation.launch.py mode:=lightweight \
     exercise_file:=$EX &
   ros2 launch plato_pod los.launch.py backend:=python &
   ros2 launch plato_pod engagement.launch.py exercise_file:=$EX &
   ros2 launch plato_pod opfor.launch.py exercise_file:=$EX &
   ros2 launch plato_pod cot_bridge.launch.py exercise_file:=$EX
   ```
   Use `los.launch.py backend:=gazebo` instead when running with terrain mode and a Gazebo lidar bridge.

5. **Inspect the live world** — verify `world_state_node` is publishing what you expect:
   ```bash
   ros2 topic echo /world/cover --once
   ros2 topic echo /world/roe --once
   ros2 topic echo /world/weapons --once
   ros2 topic echo /world/civilians --once
   ```
   These are latched topics — they'll deliver the latest sample to any new subscriber, so it's safe to query them at any point during the exercise.

## Common patterns

**Unscripted OPFOR ambush** — set `patrol_route: []` and put the unit at the ambush location. It will OBSERVE, then ENGAGE when a target enters `engagement_range_m`.

**Scripted UAV** — give a virtual unit `vehicle_role: "uav"` and a `patrol_route` that traces a racetrack. The `uav_camera` sensor plugin treats the unit's pose as the camera footprint centre.

**Indirect fire** — call `inject_event` with `event_type: "indirect_fire"` carrying a target position and weapon spec. `engagement_node` evaluates with `actor=None`, so friendly-fire and target-id checks skip.

**Resupply convoy** — call `inject_event` with `event_type: "resupply"` carrying `{robot_id, items: {fuel: 1.0, ammo: {M4_carbine: 60}}}`. `logistics.resupply` adds the items.

**Wounded but mobile** — `mobility_factor` halves speed at health=0.5 and drops to 0.1 below health=0.5. Test by placing an OPFOR with low health near the player and watching how slowly it can flee.
