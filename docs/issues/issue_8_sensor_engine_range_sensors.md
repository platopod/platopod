## Sensor Engine and Range Sensors (Lidar + Sonar)

Implement the sensor engine node — the runtime infrastructure for all simulated sensors — along with the ray caster utility and the first two sensor plugins: 2D Lidar and Sonar. The sensor engine is a single ROS2 node that manages shared state (arena model, robot poses, team assignments), schedules sensor computations, applies noise, and publishes results. Individual sensors are implemented as lightweight plugins that receive a context and return readings.

### Architecture

```
Arena Model (Issue #1)  ──┐
                          │
Robot Poses               ├──► Sensor Engine Node
  Physical (vision node)  │     ├── Shared State
  Virtual (Issue #4)      │     │     arena_model (Shapely polygons)
                          │     │     robot_states (pose, radius, team)
Team Registry (Issue #7) ─┘     │
                                ├── Ray Caster (Shapely)
                                ├── Noise Injector
                                ├── Scheduler
                                │
                                ├── Plugins:
                                │     ├── LidarPlugin
                                │     └── SonarPlugin
                                │
                                ▼
                        /robot_{id}/sensors/lidar_2d
                        /robot_{id}/sensors/sonar
```

### Sensor plugin interface

Every sensor plugin implements a single interface:

```python
class SensorPlugin:
    """Base class for all sensor plugins."""

    name: str               # unique sensor name, e.g. "lidar_2d"
    sensor_type: str         # "continuous" or "event"
    default_rate_hz: float   # default update rate
    default_config: dict     # default parameters

    def compute(self, context: SensorContext) -> dict:
        """
        Compute one sensor reading.
        Returns a data dict matching the sensor's JSON schema (Issue #6).
        The engine applies noise AFTER this returns.
        """
        raise NotImplementedError
```

The context provided to every plugin:

```python
@dataclass
class SensorContext:
    robot_pose: Pose2D           # this robot's (x, y, theta)
    robot_radius: float          # this robot's radius
    robot_team: str | None       # this robot's team
    arena_boundary: Polygon      # Shapely Polygon of arena perimeter
    obstacles: list[Polygon]     # Shapely Polygons of static obstacles
    other_robots: list[RobotState]  # all other robots
    config: dict                 # this sensor's per-robot configuration
    ray_cast: Callable           # reference to engine's ray_cast function

@dataclass
class RobotState:
    robot_id: int
    pose: Pose2D
    radius: float
    team: str | None
```

Plugins are stateless — they receive everything they need via the context and return a result. No ROS2 subscriptions, no state management, no side effects.

### Ray caster

The ray caster is a shared utility provided by the engine. It uses Shapely for geometry operations:

```python
@dataclass
class RayHit:
    distance: float          # distance from origin to hit point
    hit_type: str            # "boundary", "obstacle", "robot"
    label: str | None        # obstacle label or robot_id
    point: tuple[float, float]  # hit point coordinates

def ray_cast(
    origin: tuple[float, float],
    angle: float,
    max_range: float,
    arena_boundary: Polygon,
    obstacles: list[Polygon],
    other_robots: list[RobotState],
    ignore_robot_id: int | None = None
) -> RayHit | None:
    """
    Cast a ray from origin at angle (radians).
    Returns the nearest intersection, or None if no hit within max_range.
    
    Tests against:
    1. Arena boundary perimeter (exterior ring)
    2. All obstacle polygons
    3. All other robot circles (approximated as Shapely Points with buffer)
    
    ignore_robot_id excludes the casting robot from self-intersection.
    """
```

Implementation uses Shapely's `LineString.intersection()` against each geometry element, collecting all hit points and returning the nearest:

```python
from shapely.geometry import LineString, Point
import math

def ray_cast(origin, angle, max_range, arena_boundary, obstacles, other_robots, ignore_robot_id=None):
    ox, oy = origin
    end_x = ox + max_range * math.cos(angle)
    end_y = oy + max_range * math.sin(angle)
    ray = LineString([(ox, oy), (end_x, end_y)])

    nearest = None
    nearest_dist = max_range

    # Test arena boundary
    hit = ray.intersection(arena_boundary.exterior)
    if not hit.is_empty:
        d = Point(ox, oy).distance(hit)
        if d < nearest_dist:
            nearest_dist = d
            nearest = RayHit(d, "boundary", None, _nearest_point(hit, origin))

    # Test obstacles
    for obs in obstacles:
        hit = ray.intersection(obs.exterior)
        if not hit.is_empty:
            d = Point(ox, oy).distance(hit)
            if d < nearest_dist:
                nearest_dist = d
                nearest = RayHit(d, "obstacle", obs.label, _nearest_point(hit, origin))

    # Test other robots (circles)
    for robot in other_robots:
        if robot.robot_id == ignore_robot_id:
            continue
        circle = Point(robot.pose.x, robot.pose.y).buffer(robot.radius)
        hit = ray.intersection(circle.exterior)
        if not hit.is_empty:
            d = Point(ox, oy).distance(hit)
            if d < nearest_dist:
                nearest_dist = d
                nearest = RayHit(d, "robot", str(robot.robot_id), _nearest_point(hit, origin))

    return nearest
```

### Performance considerations

Ray casting is the most computationally expensive operation. For N robots each with 360-degree Lidar at 10 Hz, the engine performs N × 360 × 10 = 36,000 ray casts per second per robot. With 8 robots: 288,000 ray casts/second.

Optimisations:

- **Shapely's prepared geometries:** `prepared.prep(boundary)` pre-computes spatial indices for faster intersection tests.
- **Skip distant objects:** Before ray casting against a robot or obstacle, check if it's within `max_range` of the casting robot. If not, skip it entirely.
- **Batch rays per robot:** Construct all 360 rays as a single MultiLineString and intersect once per geometry element, rather than 360 individual intersection calls.
- **Reduce resolution:** Sonar at 10° segments needs only 36 rays, not 360. Most educational scenarios don't need 1° Lidar resolution — 5° (72 rays) is often sufficient.
- **NumPy vectorisation:** Compute ray endpoints as NumPy arrays, convert to Shapely geometries in batch.

For the expected scale (≤20 robots on a Raspberry Pi 5), even the naive approach should achieve 10 Hz update rates. Optimise only if profiling shows a bottleneck.

### Noise injector

The engine applies noise after the plugin returns ideal readings. This is consistent across all sensor types:

```python
def inject_noise(reading: float, config: dict, rng: np.random.Generator) -> float | None:
    """
    Apply noise to a single range reading.
    Returns None if dropout occurs (simulates sensor failure).
    """
    # Dropout check
    if rng.random() < config.get("dropout_rate", 0.0):
        return None  # invalid reading

    # Gaussian noise
    stddev = config.get("noise_stddev", 0.0)
    if stddev > 0:
        reading += rng.normal(0, stddev)

    # Clamp to valid range
    reading = max(0.0, min(reading, config.get("range_max", float('inf'))))

    return reading
```

The random number generator is seeded per-robot for reproducibility in training scenarios.

### Scheduler

The scheduler determines which sensor computations to run each engine tick:

```python
class SensorScheduler:
    """Manages per-robot, per-sensor update timing."""

    def __init__(self, engine_tick_hz: float = 50.0):
        self.tick_period = 1.0 / engine_tick_hz
        self.schedules = {}  # (robot_id, sensor_name) → next_tick_time

    def register(self, robot_id: int, sensor_name: str, rate_hz: float):
        """Register a sensor for scheduled computation."""
        self.schedules[(robot_id, sensor_name)] = {
            "period": 1.0 / rate_hz,
            "next_time": 0.0
        }

    def get_due(self, current_time: float) -> list[tuple[int, str]]:
        """Return list of (robot_id, sensor_name) due for computation."""
        due = []
        for key, sched in self.schedules.items():
            if current_time >= sched["next_time"]:
                due.append(key)
                sched["next_time"] = current_time + sched["period"]
        return due
```

The engine tick rate (default 50 Hz) should be at least 2× the highest sensor rate to avoid aliasing. Individual sensors can run at any rate up to the engine tick rate.

### 2D Lidar plugin

Simulates a planar laser scanner. Casts rays in a full or partial arc around the robot, returning distance to the nearest object per angular step.

```python
class LidarPlugin(SensorPlugin):
    name = "lidar_2d"
    sensor_type = "continuous"
    default_rate_hz = 10.0
    default_config = {
        "angle_min": 0.0,           # start angle relative to robot heading (rad)
        "angle_max": 6.283,         # end angle (2π = full 360°)
        "angular_resolution_deg": 1.0,  # degrees per ray
        "range_min": 0.01,          # minimum detectable range (m)
        "range_max": 2.0,           # maximum range (m)
        "noise_stddev": 0.005,      # ±5mm noise
        "dropout_rate": 0.0
    }

    def compute(self, ctx: SensorContext) -> dict:
        config = ctx.config
        angle_min = config["angle_min"]
        angle_max = config["angle_max"]
        resolution = math.radians(config["angular_resolution_deg"])
        range_max = config["range_max"]
        range_min = config["range_min"]

        angles = np.arange(angle_min, angle_max, resolution)
        ranges = []

        for angle in angles:
            world_angle = ctx.robot_pose.theta + angle
            hit = ctx.ray_cast(
                origin=(ctx.robot_pose.x, ctx.robot_pose.y),
                angle=world_angle,
                max_range=range_max,
                ignore_robot_id=None  # engine passes the casting robot's ID
            )
            if hit and hit.distance >= range_min:
                ranges.append(hit.distance)
            else:
                ranges.append(range_max)  # no hit = max range

        return {
            "ranges": ranges,
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_increment": resolution,
            "range_min": range_min,
            "range_max": range_max
        }
```

**ROS2 topic:** `/robot_{id}/sensors/lidar_2d` — publishes `sensor_msgs/LaserScan`

**WebSocket message (Issue #6):**
```json
{
  "type": "sensor",
  "robot_id": 1,
  "sensor": "lidar_2d",
  "timestamp": 1709312445.123,
  "data": {
    "ranges": [0.45, 0.44, 0.46, "..."],
    "angle_min": 0.0,
    "angle_max": 6.283,
    "angle_increment": 0.01745,
    "range_min": 0.01,
    "range_max": 2.0
  }
}
```

**Configurable parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `angle_min` | 0.0 rad | Start angle relative to robot heading |
| `angle_max` | 2π rad | End angle (2π = full circle) |
| `angular_resolution_deg` | 1.0° | Angular step per ray |
| `range_min` | 0.01 m | Minimum detectable distance |
| `range_max` | 2.0 m | Maximum detectable distance |
| `noise_stddev` | 0.005 m | Gaussian noise standard deviation |
| `dropout_rate` | 0.0 | Probability of invalid reading per ray |

### Sonar plugin

Simulates an ultrasonic range sensor. Similar to Lidar but with wider beam segments and lower angular resolution. Each segment returns the **minimum** range within its beam width, simulating the broad cone of an ultrasonic transducer.

```python
class SonarPlugin(SensorPlugin):
    name = "sonar"
    sensor_type = "continuous"
    default_rate_hz = 10.0
    default_config = {
        "angle_min": 0.0,
        "angle_max": 6.283,
        "segment_angle_deg": 10.0,    # beam width per segment
        "rays_per_segment": 5,        # rays cast within each segment for accuracy
        "range_min": 0.02,
        "range_max": 1.5,
        "noise_stddev": 0.02,         # ±20mm noise (less precise than Lidar)
        "dropout_rate": 0.0
    }

    def compute(self, ctx: SensorContext) -> dict:
        config = ctx.config
        segment_angle = math.radians(config["segment_angle_deg"])
        rays_per_seg = config["rays_per_segment"]
        range_max = config["range_max"]
        range_min = config["range_min"]

        num_segments = int((config["angle_max"] - config["angle_min"]) / segment_angle)
        ranges = []

        for i in range(num_segments):
            seg_start = config["angle_min"] + i * segment_angle
            seg_end = seg_start + segment_angle
            ray_angles = np.linspace(seg_start, seg_end, rays_per_seg, endpoint=False)

            min_range = range_max
            for angle in ray_angles:
                world_angle = ctx.robot_pose.theta + angle
                hit = ctx.ray_cast(
                    origin=(ctx.robot_pose.x, ctx.robot_pose.y),
                    angle=world_angle,
                    max_range=range_max,
                    ignore_robot_id=None
                )
                if hit and hit.distance >= range_min:
                    min_range = min(min_range, hit.distance)

            ranges.append(min_range)

        return {
            "ranges": ranges,
            "segment_angle_deg": config["segment_angle_deg"],
            "range_min": range_min,
            "range_max": range_max,
            "num_segments": num_segments
        }
```

**ROS2 topic:** `/robot_{id}/sensors/sonar` — publishes custom `MultiRange` message

**WebSocket message (Issue #6):**
```json
{
  "type": "sensor",
  "robot_id": 1,
  "sensor": "sonar",
  "timestamp": 1709312445.123,
  "data": {
    "ranges": [0.45, 0.50, 0.38, "...36 values"],
    "segment_angle_deg": 10.0,
    "range_min": 0.02,
    "range_max": 1.5,
    "num_segments": 36
  }
}
```

**Configurable parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `angle_min` | 0.0 rad | Start angle |
| `angle_max` | 2π rad | End angle |
| `segment_angle_deg` | 10.0° | Beam width per segment |
| `rays_per_segment` | 5 | Rays cast per segment (accuracy vs performance) |
| `range_min` | 0.02 m | Minimum detectable distance |
| `range_max` | 1.5 m | Maximum detectable distance |
| `noise_stddev` | 0.02 m | Gaussian noise (higher than Lidar) |
| `dropout_rate` | 0.0 | Probability of invalid reading per segment |

### Sensor presets using Lidar and Sonar

The presets defined in Issue #6 map to specific configurations:

| Preset | Lidar config | Sonar config |
|--------|-------------|--------------|
| `minimal` | — | — |
| `basic_scout` | — | Front 90° only (`angle_min: -0.785, angle_max: 0.785`), 15° segments |
| `full_suite` | 360°, 2° resolution, 1.5m range | 360°, 10° segments, 1.0m range |
| `ideal` | 360°, 1° resolution, 2.0m range, no noise | — |
| `competitive` | — | 360°, 15° segments, 1.0m range, high noise (0.05m) |

### Testing

**Unit tests for ray caster:**
- Ray against simple rectangle: verify distance and hit type
- Ray parallel to wall: verify no false intersection
- Ray through multiple obstacles: verify nearest hit returned
- Ray against robot circle: verify distance to circle edge, not centre
- Ray with no intersection within max_range: verify None returned

**Integration tests:**
- Spawn a robot in a known arena, verify Lidar scan matches expected distances
- Place robot next to a wall, verify Sonar segment returns correct range
- Add obstacle mid-session, verify sensors immediately detect it
- Configure noise, verify readings vary around true value with expected standard deviation
- Verify sensor data appears on WebSocket via Issue #6 subscription

**RViz2 visualisation:**
- Lidar scan visible as a `LaserScan` display in RViz2
- Useful for visual verification during development

### Dependencies

- **Shapely** (≥2.0) — computational geometry for ray casting
- **NumPy** — noise generation, trigonometry, array operations
- **Issue #1** — arena model (boundary, obstacles)
- **Issue #2** — robot registry (poses, radii)
- **Issue #4** — virtual robot poses
- **Issue #6** — sensor API protocol (subscription, configuration, WebSocket messages)

### Acceptance criteria

- [ ] Sensor engine node subscribes to arena model, robot poses, and exercise config
- [ ] Plugin interface defined: `SensorPlugin` base class with `compute()` method
- [ ] `SensorContext` provides pose, arena, obstacles, other robots, config, and ray_cast reference
- [ ] Ray caster implemented using Shapely, tests against boundary, obstacles, and robot circles
- [ ] Ray caster returns nearest hit with distance, type, and label
- [ ] Noise injector applies Gaussian noise and dropout after plugin computation
- [ ] Scheduler runs per-robot per-sensor computations at configured rates
- [ ] Lidar plugin: configurable arc, resolution, range, produces `LaserScan`-compatible output
- [ ] Sonar plugin: configurable segments, beam width, range, produces multi-range output
- [ ] Sensor data published to per-robot ROS2 topics
- [ ] Sensor data bridged to WebSocket via API gateway (Issue #6)
- [ ] New plugins can be added by implementing `SensorPlugin` without engine changes
- [ ] Performance adequate for ≤20 robots with Lidar at 10 Hz on Raspberry Pi 5
- [ ] Sensor presets (Issue #6) correctly configure Lidar and Sonar parameters
- [ ] RViz2 can display Lidar scans for development verification
