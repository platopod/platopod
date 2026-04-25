"""Gazebo SDF world generation from exercise YAML configuration.

Generates a complete SDF world file containing arena boundary walls,
obstacles, scoring zones (visual-only), robot models, and physics
configuration. Pure Python, no external dependencies.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET


# Vehicle role → model directory mapping
ROLE_TO_MODEL: dict[str, str] = {
    "default": "platopod",
    "recon": "recon",
    "cbrn_recon": "recon",
    "tank": "tank",
    "apc": "apc",
    "artillery": "tank",
    "sensor": "platopod",
}


def build_world_sdf(
    arena_boundary: list[tuple[float, float]],
    obstacles: list[dict] | None = None,
    zones: list[dict] | None = None,
    robots: list[dict] | None = None,
    terrain: dict | None = None,
    physics_step_size: float = 0.001,
    real_time_factor: float = 1.0,
    wall_height: float = 0.3,
    model_path: str = "models",
) -> str:
    """Generate a complete SDF world XML string.

    Args:
        arena_boundary: Polygon vertices in metres (arena coordinates).
        obstacles: Obstacle dicts from exercise YAML.
        zones: Scoring zone dicts from exercise YAML.
        robots: Robot dicts with id, x, y, theta, vehicle_role.
        terrain: Optional heightmap config.
        physics_step_size: Gazebo physics step in seconds.
        real_time_factor: Simulation speed multiplier.
        wall_height: Height of arena boundary walls in metres.
        model_path: Path to SDF model directories.

    Returns:
        Valid SDF world XML string.
    """
    obstacles = obstacles or []
    zones = zones or []
    robots = robots or []

    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name="arena")

    # Physics
    physics = ET.SubElement(world, "physics", name="default", type="dart")
    step = ET.SubElement(physics, "max_step_size")
    step.text = str(physics_step_size)
    rtf = ET.SubElement(physics, "real_time_factor")
    rtf.text = str(real_time_factor)

    # Sun light
    light = ET.SubElement(world, "light", name="sun", type="directional")
    cast = ET.SubElement(light, "cast_shadows")
    cast.text = "true"
    direction = ET.SubElement(light, "direction")
    direction.text = "-0.5 0.1 -0.9"
    diffuse = ET.SubElement(light, "diffuse")
    diffuse.text = "0.8 0.8 0.8 1"

    # Ground plane or heightmap
    if terrain:
        world.append(_build_heightmap(terrain))
    else:
        world.append(_build_ground_plane())

    # Arena boundary walls
    if len(arena_boundary) >= 3:
        n = len(arena_boundary)
        for i in range(n):
            p1 = arena_boundary[i]
            p2 = arena_boundary[(i + 1) % n]
            wall_xml = build_wall_segment(p1, p2, wall_height)
            wall_el = ET.fromstring(wall_xml)
            world.append(wall_el)

    # Obstacles
    for i, obs in enumerate(obstacles):
        obs_xml = build_obstacle_sdf(obs, i, wall_height)
        if obs_xml:
            world.append(ET.fromstring(obs_xml))

    # Zones (visual-only)
    for i, zone in enumerate(zones):
        zone_xml = build_zone_visual(zone, i)
        if zone_xml:
            world.append(ET.fromstring(zone_xml))

    # Robots
    for r in robots:
        robot_xml = robot_include_sdf(
            r.get("id", r.get("robot_id", 0)),
            r.get("x", 0), r.get("y", 0), r.get("theta", 0),
            r.get("vehicle_role", "default"),
            model_path,
        )
        world.append(ET.fromstring(robot_xml))

    return ET.tostring(sdf, encoding="unicode")


def build_wall_segment(
    p1: tuple[float, float],
    p2: tuple[float, float],
    height: float = 0.3,
    thickness: float = 0.02,
) -> str:
    """Generate SDF for a single wall segment between two points."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.sqrt(dx * dx + dy * dy)
    angle = math.atan2(dy, dx)
    cx = (p1[0] + p2[0]) / 2.0
    cy = (p1[1] + p2[1]) / 2.0

    return f"""<model name="wall_{cx:.3f}_{cy:.3f}">
  <static>true</static>
  <pose>{cx} {cy} {height/2} 0 0 {angle}</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>{length} {thickness} {height}</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>{length} {thickness} {height}</size></box></geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.5 0.5 0.5 1</diffuse>
      </material>
    </visual>
  </link>
</model>"""


def build_obstacle_sdf(obstacle: dict, index: int = 0,
                        default_height: float = 0.3) -> str | None:
    """Generate SDF for an obstacle (extruded to 3D)."""
    obs_type = obstacle.get("type", "rectangle")
    label = obstacle.get("label", f"obstacle_{index}")
    height = obstacle.get("height", default_height)

    if obs_type == "rectangle":
        x = obstacle.get("x", 0)
        y = obstacle.get("y", 0)
        w = obstacle.get("width", 0.1)
        h_dim = obstacle.get("height_2d", obstacle.get("height", 0.1))
        if "width" in obstacle:
            h_dim = obstacle.get("height", 0.1)
            # Disambiguate: 'height' in exercise YAML is 2D height (y-dimension)
            # We use wall_height for 3D extrusion
        return f"""<model name="{label}">
  <static>true</static>
  <pose>{x} {y} {height/2} 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>{w} {h_dim} {height}</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>{w} {h_dim} {height}</size></box></geometry>
      <material>
        <ambient>0.4 0.4 0.4 1</ambient>
        <diffuse>0.4 0.4 0.4 1</diffuse>
      </material>
    </visual>
  </link>
</model>"""

    elif obs_type == "circle":
        x = obstacle.get("x", 0)
        y = obstacle.get("y", 0)
        r = obstacle.get("radius", 0.05)
        return f"""<model name="{label}">
  <static>true</static>
  <pose>{x} {y} {height/2} 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><cylinder><radius>{r}</radius><length>{height}</length></cylinder></geometry>
    </collision>
    <visual name="visual">
      <geometry><cylinder><radius>{r}</radius><length>{height}</length></cylinder></geometry>
      <material>
        <ambient>0.4 0.4 0.4 1</ambient>
        <diffuse>0.4 0.4 0.4 1</diffuse>
      </material>
    </visual>
  </link>
</model>"""

    return None


def build_zone_visual(zone: dict, index: int = 0) -> str | None:
    """Generate SDF visual-only overlay for a scoring zone."""
    name = zone.get("name", f"zone_{index}")
    zone_type = zone.get("type", "circle")
    team = zone.get("team", "")

    # Team colors
    if team == "blue":
        color = "0.2 0.3 0.9 0.5"
    elif team == "red":
        color = "0.9 0.2 0.2 0.5"
    else:
        color = "0.5 0.2 0.8 0.5"

    if zone_type == "circle":
        x = zone.get("x", 0)
        y = zone.get("y", 0)
        r = zone.get("radius", 0.05)
        return f"""<model name="zone_{name}">
  <static>true</static>
  <pose>{x} {y} 0.001 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry><cylinder><radius>{r}</radius><length>0.002</length></cylinder></geometry>
      <material>
        <ambient>{color}</ambient>
        <diffuse>{color}</diffuse>
      </material>
    </visual>
  </link>
</model>"""

    return None


def robot_include_sdf(
    robot_id: int, x: float, y: float, theta: float,
    vehicle_role: str = "default", model_path: str = "models",
) -> str:
    """Generate SDF <include> element for a robot model."""
    model_dir = ROLE_TO_MODEL.get(vehicle_role, "platopod")
    model_name = f"robot_{robot_id}"

    return f"""<include>
  <uri>{model_path}/{model_dir}</uri>
  <name>{model_name}</name>
  <pose>{x} {y} 0.02 0 0 {theta}</pose>
</include>"""


def _build_ground_plane() -> ET.Element:
    """Build a flat ground plane model."""
    model = ET.fromstring("""<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
    </collision>
    <visual name="visual">
      <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
      <material>
        <ambient>0.3 0.4 0.3 1</ambient>
        <diffuse>0.3 0.4 0.3 1</diffuse>
      </material>
    </visual>
  </link>
</model>""")
    return model


def _build_heightmap(terrain: dict) -> ET.Element:
    """Build a heightmap model from terrain config.

    Accepts either a raw dict (legacy) or a dict with 'heightmap_path',
    'size_x', 'size_y', 'size_z' from TerrainResult.
    """
    # Support TerrainResult-style dict
    if "heightmap_path" in terrain:
        uri = str(terrain["heightmap_path"])
        sx = terrain.get("size_x", 100)
        sy = terrain.get("size_y", 100)
        sz = terrain.get("size_z", 20)
        ox = terrain.get("origin_x", 0)
        oy = terrain.get("origin_y", 0)
        oz = terrain.get("origin_z", 0)
    else:
        # Legacy format
        uri = terrain.get("heightmap", "terrain.png")
        size = terrain.get("size", [100, 100, 20])
        origin = terrain.get("origin", [0, 0, 0])
        sx, sy, sz = size[0], size[1], size[2]
        ox, oy, oz = origin[0], origin[1], origin[2]

    return ET.fromstring(f"""<model name="terrain">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>{uri}</uri>
          <size>{sx} {sy} {sz}</size>
          <pos>{ox} {oy} {oz}</pos>
        </heightmap>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <heightmap>
          <uri>{uri}</uri>
          <size>{sx} {sy} {sz}</size>
          <pos>{ox} {oy} {oz}</pos>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>""")
