"""Tests for plato_pod.arena_model — data model, YAML loading, and validation."""

from __future__ import annotations

from pathlib import Path

import pytest

from plato_pod.arena_model import (
    ArenaModel,
    Obstacle,
    Zone,
    build_arena_model,
    load_exercise_arena,
    parse_obstacle,
    parse_zone,
    validate_all,
    validate_element,
)

EXERCISE_DIR = Path(__file__).parent.parent.parent.parent / "config" / "exercises"


# --- parse_obstacle ---

class TestParseObstacle:
    def test_rectangle(self) -> None:
        raw = {"type": "rectangle", "x": 0.5, "y": 0.3, "width": 0.1,
               "height": 0.05, "label": "wall"}
        obs = parse_obstacle(raw)
        assert obs.label == "wall"
        assert obs.original_type == "rectangle"
        assert len(obs.vertices) == 4

    def test_rectangle_with_rotation(self) -> None:
        raw = {"type": "rectangle", "x": 0.5, "y": 0.3, "width": 0.1,
               "height": 0.05, "rotation": 0.5, "label": "rotated"}
        obs = parse_obstacle(raw)
        assert len(obs.vertices) == 4

    def test_polygon(self) -> None:
        raw = {"type": "polygon",
               "vertices": [[0.1, 0.1], [0.2, 0.1], [0.15, 0.2]],
               "label": "tri"}
        obs = parse_obstacle(raw)
        assert obs.original_type == "polygon"
        assert len(obs.vertices) == 3

    def test_circle(self) -> None:
        raw = {"type": "circle", "x": 0.5, "y": 0.3, "radius": 0.03,
               "label": "pillar"}
        obs = parse_obstacle(raw, circle_vertices=8)
        assert obs.original_type == "circle"
        assert len(obs.vertices) == 8

    def test_unknown_type_raises(self) -> None:
        with pytest.raises(ValueError, match="Unknown obstacle type"):
            parse_obstacle({"type": "hexagon", "label": "bad"})

    def test_default_label(self) -> None:
        raw = {"type": "rectangle", "x": 0, "y": 0, "width": 1, "height": 1}
        obs = parse_obstacle(raw)
        assert obs.label == "unnamed"


# --- parse_zone ---

class TestParseZone:
    def test_circle_zone(self) -> None:
        raw = {"type": "circle", "x": 0.1, "y": 0.3, "radius": 0.05,
               "name": "flag", "team": "red", "hold_time_seconds": 10}
        zone = parse_zone(raw)
        assert zone.name == "flag"
        assert zone.team == "red"
        assert zone.hold_time_seconds == 10
        assert zone.original_type == "circle"

    def test_rectangle_zone(self) -> None:
        raw = {"type": "rectangle", "x": 0.5, "y": 0.5, "width": 0.1,
               "height": 0.1, "name": "base", "team": "blue"}
        zone = parse_zone(raw)
        assert zone.original_type == "rectangle"
        assert len(zone.vertices) == 4

    def test_null_team(self) -> None:
        raw = {"type": "circle", "x": 0, "y": 0, "radius": 0.1,
               "name": "neutral"}
        zone = parse_zone(raw)
        assert zone.team is None

    def test_unknown_type_raises(self) -> None:
        with pytest.raises(ValueError):
            parse_zone({"type": "star", "name": "bad"})


# --- load_exercise_arena ---

class TestLoadExerciseArena:
    def test_loads_capture_the_flag(self) -> None:
        path = EXERCISE_DIR / "capture-the-flag.yaml"
        if not path.exists():
            pytest.skip("Exercise config not found")
        boundary, obstacles, zones = load_exercise_arena(path)
        assert boundary is not None
        assert len(boundary) == 4
        assert boundary[0] == (0.0, 0.0)
        assert boundary[2] == (0.84, 0.59)
        assert len(obstacles) == 2
        assert obstacles[0].label == "wall_north"
        assert len(zones) == 2
        assert zones[0].name == "blue_flag"

    def test_obstacle_types_are_rectangle(self) -> None:
        path = EXERCISE_DIR / "capture-the-flag.yaml"
        if not path.exists():
            pytest.skip("Exercise config not found")
        _, obstacles, _ = load_exercise_arena(path)
        for obs in obstacles:
            assert obs.original_type == "rectangle"
            assert len(obs.vertices) == 4

    def test_zone_types_are_circle(self) -> None:
        path = EXERCISE_DIR / "capture-the-flag.yaml"
        if not path.exists():
            pytest.skip("Exercise config not found")
        _, _, zones = load_exercise_arena(path)
        for zone in zones:
            assert zone.original_type == "circle"
            assert len(zone.vertices) == 16

    def test_nonexistent_file_raises(self, tmp_path: Path) -> None:
        with pytest.raises(FileNotFoundError):
            load_exercise_arena(tmp_path / "nonexistent.yaml")

    def test_empty_exercise_no_boundary(self, tmp_path: Path) -> None:
        path = tmp_path / "empty.yaml"
        path.write_text("exercise:\n  arena: {}\n  scoring: {}\n")
        boundary, obstacles, zones = load_exercise_arena(path)
        assert boundary is None
        assert obstacles == []
        assert zones == []

    def test_boundary_from_yaml(self, tmp_path: Path) -> None:
        path = tmp_path / "with_boundary.yaml"
        path.write_text(
            "exercise:\n"
            "  arena:\n"
            "    boundary:\n"
            "      - [0, 0]\n"
            "      - [1, 0]\n"
            "      - [1, 1]\n"
            "      - [0, 1]\n"
            "  scoring: {}\n"
        )
        boundary, _, _ = load_exercise_arena(path)
        assert boundary is not None
        assert len(boundary) == 4
        assert boundary == [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]


# --- validate_element ---

class TestValidateElement:
    def test_inside_boundary_accepted(self) -> None:
        boundary = ((0, 0), (1, 0), (1, 1), (0, 1))
        verts = ((0.2, 0.2), (0.4, 0.2), (0.4, 0.4), (0.2, 0.4))
        result = validate_element("box", verts, boundary)
        assert result.valid is True
        assert result.warnings == []
        assert result.errors == []

    def test_partially_outside_warns(self) -> None:
        boundary = ((0, 0), (1, 0), (1, 1), (0, 1))
        verts = ((0.8, 0.8), (1.2, 0.8), (1.2, 1.2), (0.8, 1.2))
        result = validate_element("overlap", verts, boundary)
        assert result.valid is True
        assert len(result.warnings) == 1
        assert "overlap" in result.warnings[0]

    def test_fully_outside_errors(self) -> None:
        boundary = ((0, 0), (1, 0), (1, 1), (0, 1))
        verts = ((5, 5), (6, 5), (6, 6), (5, 6))
        result = validate_element("far", verts, boundary)
        assert result.valid is False
        assert len(result.errors) == 1
        assert "rejected" in result.errors[0]

    def test_no_boundary_errors(self) -> None:
        result = validate_element("box", ((0, 0), (1, 1)), ())
        assert result.valid is False
        assert "not yet established" in result.errors[0]


# --- validate_all ---

class TestValidateAll:
    def test_all_valid(self) -> None:
        boundary = ((0, 0), (10, 0), (10, 10), (0, 10))
        obs = [Obstacle("w", ((1, 1), (2, 1), (2, 2), (1, 2)), "rectangle")]
        zones = [Zone("z", None, ((3, 3), (4, 3), (4, 4), (3, 4)), 5.0, "rectangle")]
        result = validate_all(obs, zones, boundary)
        assert result.valid is True
        assert result.warnings == []

    def test_mixed_results(self) -> None:
        boundary = ((0, 0), (1, 0), (1, 1), (0, 1))
        obs = [
            Obstacle("inside", ((0.1, 0.1), (0.2, 0.1), (0.2, 0.2)), "polygon"),
            Obstacle("outside", ((5, 5), (6, 5), (6, 6), (5, 6)), "rectangle"),
        ]
        result = validate_all(obs, [], boundary)
        assert result.valid is False
        assert len(result.errors) == 1


# --- build_arena_model ---

class TestBuildArenaModel:
    def test_basic_boundary(self) -> None:
        points = [(0, 0), (1, 0), (1, 1), (0, 1)]
        model = build_arena_model(points, timestamp=42.0)
        assert model.boundary_valid is True
        assert len(model.boundary) == 4
        assert model.timestamp == 42.0
        assert model.origin_tag == 101

    def test_with_obstacles(self) -> None:
        points = [(0, 0), (10, 0), (10, 10), (0, 10)]
        obs = [Obstacle("w", ((1, 1), (2, 1), (2, 2), (1, 2)), "rectangle")]
        model = build_arena_model(points, obstacles=obs)
        assert len(model.obstacles) == 1
        assert model.obstacles[0].label == "w"

    def test_with_zones(self) -> None:
        points = [(0, 0), (10, 0), (10, 10), (0, 10)]
        zones = [Zone("z", "blue", ((3, 3), (4, 3), (4, 4), (3, 4)), 10.0, "rect")]
        model = build_arena_model(points, zones=zones)
        assert len(model.zones) == 1

    def test_insufficient_points(self) -> None:
        model = build_arena_model([(0, 0), (1, 0)])
        assert model.boundary_valid is False
        assert len(model.boundary) == 2

    def test_empty_boundary(self) -> None:
        model = build_arena_model([])
        assert model.boundary_valid is False
        assert len(model.boundary) == 0

    def test_all_points_preserved(self) -> None:
        # Default: all points used as-is (no convex hull)
        points = [(0, 0), (2, 0), (2, 2), (0, 2), (1, 1)]
        model = build_arena_model(points)
        assert model.boundary_valid is True
        assert len(model.boundary) == 5

    def test_convex_hull_mode(self) -> None:
        # With use_convex_hull=True, interior point is dropped
        points = [(0, 0), (2, 0), (2, 2), (0, 2), (1, 1)]
        model = build_arena_model(points, use_convex_hull=True)
        assert model.boundary_valid is True
        assert len(model.boundary) == 4


# --- ArenaModel.to_dict ---

class TestArenaModelToDict:
    def test_serialises_correctly(self) -> None:
        obs = [Obstacle("wall", ((0.1, 0.1), (0.2, 0.1), (0.2, 0.2), (0.1, 0.2)),
                         "rectangle")]
        zone = [Zone("flag", "red", ((0.5, 0.5), (0.6, 0.5), (0.6, 0.6)),
                      10.0, "circle")]
        model = ArenaModel(
            origin_tag=101,
            boundary=((0, 0), (1, 0), (1, 1), (0, 1)),
            boundary_valid=True,
            obstacles=tuple(obs),
            zones=tuple(zone),
            timestamp=123.456,
        )
        d = model.to_dict()
        assert d["origin_tag"] == 101
        assert d["boundary_valid"] is True
        assert len(d["boundary"]) == 4
        assert d["boundary"][0] == [0, 0]
        assert len(d["obstacles"]) == 1
        assert d["obstacles"][0]["label"] == "wall"
        assert d["obstacles"][0]["type"] == "rectangle"
        assert len(d["zones"]) == 1
        assert d["zones"][0]["name"] == "flag"
        assert d["zones"][0]["team"] == "red"
        assert d["zones"][0]["hold_time_seconds"] == 10.0

    def test_empty_model(self) -> None:
        model = ArenaModel(
            origin_tag=101,
            boundary=(),
            boundary_valid=False,
            obstacles=(),
            zones=(),
            timestamp=0.0,
        )
        d = model.to_dict()
        assert d["boundary"] == []
        assert d["obstacles"] == []
        assert d["zones"] == []
