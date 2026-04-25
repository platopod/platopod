"""Tests for plato_pod.gazebo_world_builder — SDF world generation."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from plato_pod.gazebo_world_builder import (
    build_obstacle_sdf,
    build_wall_segment,
    build_world_sdf,
    build_zone_visual,
    robot_include_sdf,
)

SQUARE_BOUNDARY = [(0, 0), (5, 0), (5, 4), (0, 4)]


class TestBuildWorldSdf:
    def test_produces_valid_xml(self) -> None:
        xml = build_world_sdf(SQUARE_BOUNDARY)
        root = ET.fromstring(xml)
        assert root.tag == "sdf"

    def test_contains_physics(self) -> None:
        xml = build_world_sdf(SQUARE_BOUNDARY, physics_step_size=0.002)
        root = ET.fromstring(xml)
        physics = root.find(".//physics")
        assert physics is not None
        step = root.find(".//max_step_size")
        assert step is not None
        assert step.text == "0.002"

    def test_wall_count_matches_boundary(self) -> None:
        xml = build_world_sdf(SQUARE_BOUNDARY)
        root = ET.fromstring(xml)
        walls = [m for m in root.findall(".//model") if m.get("name", "").startswith("wall_")]
        assert len(walls) == 4  # 4-sided boundary

    def test_triangle_boundary(self) -> None:
        triangle = [(0, 0), (3, 0), (1.5, 2)]
        xml = build_world_sdf(triangle)
        root = ET.fromstring(xml)
        walls = [m for m in root.findall(".//model") if m.get("name", "").startswith("wall_")]
        assert len(walls) == 3

    def test_obstacles_present(self) -> None:
        obs = [{"type": "rectangle", "x": 2.5, "y": 2.0, "width": 0.5, "height": 0.3, "label": "wall1"}]
        xml = build_world_sdf(SQUARE_BOUNDARY, obstacles=obs)
        root = ET.fromstring(xml)
        wall1 = root.find(".//model[@name='wall1']")
        assert wall1 is not None

    def test_zones_visual_only(self) -> None:
        zones = [{"name": "flag", "type": "circle", "x": 1.0, "y": 1.0, "radius": 0.5, "team": "blue"}]
        xml = build_world_sdf(SQUARE_BOUNDARY, zones=zones)
        root = ET.fromstring(xml)
        zone = root.find(".//model[@name='zone_flag']")
        assert zone is not None
        # Zones should have visual but no collision
        collision = zone.find(".//collision")
        assert collision is None

    def test_robots_included(self) -> None:
        robots = [{"id": 1, "x": 1.0, "y": 1.0, "theta": 0, "vehicle_role": "default"}]
        xml = build_world_sdf(SQUARE_BOUNDARY, robots=robots)
        root = ET.fromstring(xml)
        include = root.find(".//include")
        assert include is not None
        name = include.find("name")
        assert name is not None
        assert name.text == "robot_1"

    def test_vehicle_role_mapping(self) -> None:
        robots = [{"id": 1, "x": 0, "y": 0, "theta": 0, "vehicle_role": "tank"}]
        xml = build_world_sdf(SQUARE_BOUNDARY, robots=robots, model_path="/models")
        root = ET.fromstring(xml)
        uri = root.find(".//include/uri")
        assert uri is not None
        assert "tank" in uri.text

    def test_empty_obstacles_zones(self) -> None:
        xml = build_world_sdf(SQUARE_BOUNDARY, obstacles=[], zones=[])
        root = ET.fromstring(xml)
        assert root.tag == "sdf"

    def test_terrain_heightmap(self) -> None:
        terrain = {"heightmap": "terrain.png", "size": [100, 100, 20]}
        xml = build_world_sdf(SQUARE_BOUNDARY, terrain=terrain)
        root = ET.fromstring(xml)
        hm = root.find(".//heightmap")
        assert hm is not None

    def test_no_terrain_flat_ground(self) -> None:
        xml = build_world_sdf(SQUARE_BOUNDARY, terrain=None)
        root = ET.fromstring(xml)
        ground = root.find(".//model[@name='ground_plane']")
        assert ground is not None


class TestBuildWallSegment:
    def test_valid_xml(self) -> None:
        xml = build_wall_segment((0, 0), (5, 0))
        root = ET.fromstring(xml)
        assert root.tag == "model"
        assert root.get("name").startswith("wall_")

    def test_wall_dimensions(self) -> None:
        xml = build_wall_segment((0, 0), (5, 0), height=0.5, thickness=0.03)
        root = ET.fromstring(xml)
        size = root.find(".//size")
        assert size is not None
        parts = size.text.split()
        assert float(parts[0]) == pytest.approx(5.0)  # length
        assert float(parts[1]) == pytest.approx(0.03)  # thickness
        assert float(parts[2]) == pytest.approx(0.5)   # height


class TestBuildObstacleSdf:
    def test_rectangle(self) -> None:
        obs = {"type": "rectangle", "x": 2.5, "y": 2.0, "width": 0.5, "height": 0.3}
        xml = build_obstacle_sdf(obs)
        assert xml is not None
        root = ET.fromstring(xml)
        assert root.find(".//collision") is not None

    def test_circle(self) -> None:
        obs = {"type": "circle", "x": 1.0, "y": 1.0, "radius": 0.3}
        xml = build_obstacle_sdf(obs)
        assert xml is not None
        root = ET.fromstring(xml)
        cylinder = root.find(".//cylinder")
        assert cylinder is not None


class TestRobotIncludeSdf:
    def test_valid_xml(self) -> None:
        xml = robot_include_sdf(1, 2.0, 3.0, 1.57)
        root = ET.fromstring(xml)
        assert root.tag == "include"
        name = root.find("name")
        assert name.text == "robot_1"

    def test_default_model(self) -> None:
        xml = robot_include_sdf(1, 0, 0, 0, "default", "/models")
        root = ET.fromstring(xml)
        uri = root.find("uri")
        assert "platopod" in uri.text

    def test_tank_model(self) -> None:
        xml = robot_include_sdf(1, 0, 0, 0, "tank", "/models")
        root = ET.fromstring(xml)
        uri = root.find("uri")
        assert "tank" in uri.text
