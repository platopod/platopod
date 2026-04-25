"""Arena model data structures, exercise loading, and validation.

The ArenaModel is the single authoritative representation of the environment:
boundary polygon, obstacles, and scoring zones. It is published as a ROS2
topic and served via REST, consumed by all server components.

No ROS2 dependency.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from plato_pod.geometry import (
    circle_to_polygon,
    convex_hull,
    polygon_contains_polygon,
    rectangle_to_polygon,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class Obstacle:
    """A static obstacle within the arena, stored as a polygon."""
    label: str
    vertices: tuple[tuple[float, float], ...]
    original_type: str  # "rectangle", "polygon", "circle"


@dataclass(frozen=True, slots=True)
class Zone:
    """A scoring zone within the arena, stored as a polygon."""
    name: str
    team: str | None
    vertices: tuple[tuple[float, float], ...]
    hold_time_seconds: float
    original_type: str  # "rectangle", "polygon", "circle"


@dataclass(frozen=True, slots=True)
class ValidationResult:
    """Result of validating obstacles/zones against the boundary."""
    valid: bool
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)


@dataclass(frozen=True, slots=True)
class ArenaModel:
    """Complete arena environment model."""
    origin_tag: int
    boundary: tuple[tuple[float, float], ...]
    boundary_valid: bool
    obstacles: tuple[Obstacle, ...]
    zones: tuple[Zone, ...]
    timestamp: float

    def to_dict(self) -> dict:
        """Serialise to a JSON-compatible dictionary."""
        return {
            "origin_tag": self.origin_tag,
            "boundary": [list(v) for v in self.boundary],
            "boundary_valid": self.boundary_valid,
            "obstacles": [
                {
                    "label": o.label,
                    "vertices": [list(v) for v in o.vertices],
                    "type": o.original_type,
                }
                for o in self.obstacles
            ],
            "zones": [
                {
                    "name": z.name,
                    "team": z.team,
                    "vertices": [list(v) for v in z.vertices],
                    "type": z.original_type,
                    "hold_time_seconds": z.hold_time_seconds,
                }
                for z in self.zones
            ],
            "timestamp": self.timestamp,
        }


def parse_obstacle(
    raw: dict, circle_vertices: int = 16
) -> Obstacle:
    """Parse a single obstacle from an exercise YAML dictionary.

    Args:
        raw: Dictionary with 'type', shape parameters, and 'label'.
        circle_vertices: Number of vertices for circle approximation.

    Returns:
        Obstacle dataclass with polygon vertices.

    Raises:
        ValueError: If the obstacle type is unknown or required fields are missing.
    """
    obs_type = raw["type"]
    label = raw.get("label", "unnamed")

    if obs_type == "rectangle":
        verts = rectangle_to_polygon(
            raw["x"], raw["y"], raw["width"], raw["height"],
            rotation=raw.get("rotation", 0.0),
        )
    elif obs_type == "polygon":
        verts = [tuple(v) for v in raw["vertices"]]
    elif obs_type == "circle":
        verts = circle_to_polygon(
            raw["x"], raw["y"], raw["radius"],
            n_vertices=circle_vertices,
        )
    else:
        raise ValueError(f"Unknown obstacle type: {obs_type}")

    return Obstacle(
        label=label,
        vertices=tuple(tuple(v) for v in verts),
        original_type=obs_type,
    )


def parse_zone(
    raw: dict, circle_vertices: int = 16
) -> Zone:
    """Parse a single scoring zone from an exercise YAML dictionary.

    Args:
        raw: Dictionary with 'type', shape parameters, 'name', 'team', 'hold_time_seconds'.
        circle_vertices: Number of vertices for circle approximation.

    Returns:
        Zone dataclass with polygon vertices.

    Raises:
        ValueError: If the zone type is unknown or required fields are missing.
    """
    zone_type = raw["type"]
    name = raw["name"]
    team = raw.get("team")
    hold_time = raw.get("hold_time_seconds", 0.0)

    if zone_type == "rectangle":
        verts = rectangle_to_polygon(
            raw["x"], raw["y"], raw["width"], raw["height"],
            rotation=raw.get("rotation", 0.0),
        )
    elif zone_type == "polygon":
        verts = [tuple(v) for v in raw["vertices"]]
    elif zone_type == "circle":
        verts = circle_to_polygon(
            raw["x"], raw["y"], raw["radius"],
            n_vertices=circle_vertices,
        )
    else:
        raise ValueError(f"Unknown zone type: {zone_type}")

    return Zone(
        name=name,
        team=team,
        vertices=tuple(tuple(v) for v in verts),
        hold_time_seconds=hold_time,
        original_type=zone_type,
    )


def load_exercise_arena(
    path: str | Path, circle_vertices: int = 16
) -> tuple[list[tuple[float, float]] | None, list[Obstacle], list[Zone]]:
    """Load boundary, obstacles, and zones from an exercise YAML file.

    Args:
        path: Path to the exercise YAML file.
        circle_vertices: Number of vertices for circle approximation.

    Returns:
        Tuple of (boundary, obstacles, zones). boundary is None if not
        defined in the YAML (falls back to tag-based boundary).

    Raises:
        FileNotFoundError: If the file does not exist.
    """
    path = Path(path)
    with open(path) as f:
        raw = yaml.safe_load(f)

    exercise = raw.get("exercise", {})
    arena = exercise.get("arena", {})
    scoring = exercise.get("scoring", {})

    # Parse explicit boundary polygon if defined
    boundary = None
    boundary_raw = arena.get("boundary")
    if boundary_raw and isinstance(boundary_raw, list) and len(boundary_raw) >= 3:
        boundary = [(float(p[0]), float(p[1])) for p in boundary_raw]

    obstacles = []
    for obs_raw in arena.get("obstacles", []):
        try:
            obstacles.append(parse_obstacle(obs_raw, circle_vertices))
        except (KeyError, ValueError) as e:
            logger.warning("Skipping invalid obstacle: %s", e)

    zones = []
    for zone_raw in scoring.get("zones", []):
        try:
            zones.append(parse_zone(zone_raw, circle_vertices))
        except (KeyError, ValueError) as e:
            logger.warning("Skipping invalid zone: %s", e)

    return boundary, obstacles, zones


def validate_element(
    label: str,
    vertices: tuple[tuple[float, float], ...],
    boundary: tuple[tuple[float, float], ...],
) -> ValidationResult:
    """Validate a single element (obstacle or zone) against the boundary.

    Args:
        label: Element label for error messages.
        vertices: Element polygon vertices.
        boundary: Arena boundary polygon vertices.

    Returns:
        ValidationResult with accept/warn/error status.
    """
    if len(boundary) < 3:
        return ValidationResult(
            valid=False,
            errors=[f"'{label}': boundary not yet established"],
        )

    all_inside, outside_indices = polygon_contains_polygon(
        list(boundary), list(vertices)
    )

    if all_inside:
        return ValidationResult(valid=True)

    if len(outside_indices) == len(vertices):
        return ValidationResult(
            valid=False,
            errors=[f"'{label}': all vertices outside boundary — rejected"],
        )

    return ValidationResult(
        valid=True,
        warnings=[
            f"'{label}': {len(outside_indices)} of {len(vertices)} "
            f"vertices outside boundary (indices: {outside_indices})"
        ],
    )


def validate_all(
    obstacles: list[Obstacle],
    zones: list[Zone],
    boundary: tuple[tuple[float, float], ...],
) -> ValidationResult:
    """Validate all obstacles and zones against the boundary.

    Args:
        obstacles: List of obstacles to validate.
        zones: List of zones to validate.
        boundary: Arena boundary polygon.

    Returns:
        Aggregated ValidationResult.
    """
    all_warnings: list[str] = []
    all_errors: list[str] = []

    for obs in obstacles:
        result = validate_element(obs.label, obs.vertices, boundary)
        all_warnings.extend(result.warnings)
        all_errors.extend(result.errors)

    for zone in zones:
        result = validate_element(zone.name, zone.vertices, boundary)
        all_warnings.extend(result.warnings)
        all_errors.extend(result.errors)

    return ValidationResult(
        valid=len(all_errors) == 0,
        warnings=all_warnings,
        errors=all_errors,
    )


def build_arena_model(
    boundary_points: list[tuple[float, float]],
    obstacles: list[Obstacle] | None = None,
    zones: list[Zone] | None = None,
    origin_tag: int = 101,
    timestamp: float = 0.0,
    use_convex_hull: bool = False,
) -> ArenaModel:
    """Build an ArenaModel from boundary tag positions, obstacles, and zones.

    By default, uses all boundary points in the order given (sorted by tag ID
    by the caller). Set use_convex_hull=True to compute the convex hull instead.

    Args:
        boundary_points: (x, y) positions of detected boundary tags (sorted by tag ID).
        obstacles: Static obstacles (default: none).
        zones: Scoring zones (default: none).
        origin_tag: Origin tag ID.
        timestamp: Model timestamp.
        use_convex_hull: If True, compute convex hull (may drop interior points).

    Returns:
        Immutable ArenaModel.
    """
    obstacles = obstacles or []
    zones = zones or []

    if len(boundary_points) >= 3:
        if use_convex_hull:
            hull = convex_hull(boundary_points)
        else:
            hull = list(boundary_points)
        boundary_valid = True
    else:
        hull = list(boundary_points)
        boundary_valid = False

    boundary_tuple = tuple(tuple(p) for p in hull)

    if boundary_valid:
        result = validate_all(obstacles, zones, boundary_tuple)
        for w in result.warnings:
            logger.warning(w)
        for e in result.errors:
            logger.error(e)

    return ArenaModel(
        origin_tag=origin_tag,
        boundary=boundary_tuple,
        boundary_valid=boundary_valid,
        obstacles=tuple(obstacles),
        zones=tuple(zones),
        timestamp=timestamp,
    )
