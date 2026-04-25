"""Tests for plato_pod.replay — GPS track loading and interpolation."""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

import pytest

from plato_pod.geo_reference import GeoReference
from plato_pod.replay import (
    ReplayTrack,
    ReplayWaypoint,
    interpolate_arena_position,
    interpolate_position,
    load_gpx,
    load_replay_yaml,
)


def _track() -> ReplayTrack:
    return ReplayTrack(
        track_id="alpha-1",
        robot_id=1,
        team="blue",
        waypoints=[
            ReplayWaypoint(timestamp=0.0, lat=-35.2975, lon=149.1012, heading_deg=0.0),
            ReplayWaypoint(timestamp=10.0, lat=-35.2965, lon=149.1012, heading_deg=0.0),
            ReplayWaypoint(timestamp=20.0, lat=-35.2965, lon=149.1022, heading_deg=90.0),
        ],
    )


class TestInterpolatePosition:
    def test_at_waypoint(self) -> None:
        t = _track()
        result = interpolate_position(t, 0.0)
        assert result is not None
        lat, lon, h = result
        assert lat == pytest.approx(-35.2975)
        assert lon == pytest.approx(149.1012)

    def test_midpoint(self) -> None:
        t = _track()
        result = interpolate_position(t, 5.0)
        assert result is not None
        lat, lon, h = result
        assert lat == pytest.approx(-35.2970)  # halfway between -35.2975 and -35.2965

    def test_before_start_clamps(self) -> None:
        t = _track()
        result = interpolate_position(t, -5.0)
        assert result is not None
        lat, _, _ = result
        assert lat == pytest.approx(-35.2975)

    def test_after_end_clamps(self) -> None:
        t = _track()
        result = interpolate_position(t, 100.0)
        assert result is not None
        lat, lon, _ = result
        assert lat == pytest.approx(-35.2965)
        assert lon == pytest.approx(149.1022)

    def test_empty_track(self) -> None:
        t = ReplayTrack(track_id="empty", waypoints=[])
        assert interpolate_position(t, 0.0) is None

    def test_heading_interpolation(self) -> None:
        t = _track()
        result = interpolate_position(t, 15.0)  # between wp[1] (0°) and wp[2] (90°)
        assert result is not None
        _, _, h = result
        assert h == pytest.approx(45.0)


class TestInterpolateArenaPosition:
    def test_converts_to_arena(self) -> None:
        t = _track()
        geo = GeoReference(origin_lat=-35.2975, origin_lon=149.1012)
        result = interpolate_arena_position(t, 0.0, geo)
        assert result is not None
        x, y, theta = result
        assert x == pytest.approx(0.0, abs=0.1)
        assert y == pytest.approx(0.0, abs=0.1)

    def test_moved_position(self) -> None:
        t = _track()
        geo = GeoReference(origin_lat=-35.2975, origin_lon=149.1012)
        result = interpolate_arena_position(t, 10.0, geo)
        assert result is not None
        x, y, theta = result
        # Should be ~111m north
        assert x > 50.0


class TestReplayTrack:
    def test_duration(self) -> None:
        t = _track()
        assert t.duration == pytest.approx(20.0)

    def test_empty_duration(self) -> None:
        t = ReplayTrack(track_id="empty", waypoints=[])
        assert t.duration == 0.0


class TestLoadGpx:
    def test_load_minimal_gpx(self) -> None:
        gpx = """<?xml version="1.0"?>
<gpx version="1.1" xmlns="http://www.topografix.com/GPX/1/1">
  <trk>
    <name>patrol-alpha</name>
    <trkseg>
      <trkpt lat="-35.2975" lon="149.1012">
        <ele>580</ele>
        <time>2024-03-15T10:00:00Z</time>
      </trkpt>
      <trkpt lat="-35.2965" lon="149.1012">
        <ele>585</ele>
        <time>2024-03-15T10:00:10Z</time>
      </trkpt>
      <trkpt lat="-35.2965" lon="149.1022">
        <ele>582</ele>
        <time>2024-03-15T10:00:20Z</time>
      </trkpt>
    </trkseg>
  </trk>
</gpx>"""
        with tempfile.NamedTemporaryFile(suffix=".gpx", mode="w", delete=False) as f:
            f.write(gpx)
            f.flush()
            tracks = load_gpx(Path(f.name))

        assert len(tracks) == 1
        assert tracks[0].track_id == "patrol-alpha"
        assert len(tracks[0].waypoints) == 3
        assert tracks[0].waypoints[0].timestamp == pytest.approx(0.0)
        assert tracks[0].waypoints[1].timestamp == pytest.approx(10.0)
        assert tracks[0].waypoints[0].lat == pytest.approx(-35.2975)
        assert tracks[0].waypoints[0].alt == pytest.approx(580.0)

    def test_load_multi_track_gpx(self) -> None:
        gpx = """<?xml version="1.0"?>
<gpx version="1.1" xmlns="http://www.topografix.com/GPX/1/1">
  <trk>
    <name>alpha</name>
    <trkseg>
      <trkpt lat="-35.2975" lon="149.1012"><time>2024-01-01T00:00:00Z</time></trkpt>
    </trkseg>
  </trk>
  <trk>
    <name>bravo</name>
    <trkseg>
      <trkpt lat="-35.2980" lon="149.1020"><time>2024-01-01T00:00:00Z</time></trkpt>
    </trkseg>
  </trk>
</gpx>"""
        with tempfile.NamedTemporaryFile(suffix=".gpx", mode="w", delete=False) as f:
            f.write(gpx)
            f.flush()
            tracks = load_gpx(Path(f.name))

        assert len(tracks) == 2
        assert tracks[0].track_id == "alpha"
        assert tracks[1].track_id == "bravo"

    def test_heading_computed(self) -> None:
        gpx = """<?xml version="1.0"?>
<gpx version="1.1" xmlns="http://www.topografix.com/GPX/1/1">
  <trk>
    <name>test</name>
    <trkseg>
      <trkpt lat="-35.2975" lon="149.1012"><time>2024-01-01T00:00:00Z</time></trkpt>
      <trkpt lat="-35.2965" lon="149.1012"><time>2024-01-01T00:00:10Z</time></trkpt>
    </trkseg>
  </trk>
</gpx>"""
        with tempfile.NamedTemporaryFile(suffix=".gpx", mode="w", delete=False) as f:
            f.write(gpx)
            f.flush()
            tracks = load_gpx(Path(f.name))

        # Moving north → heading ≈ 0°
        assert tracks[0].waypoints[0].heading_deg == pytest.approx(0.0, abs=1.0)


class TestLoadReplayYaml:
    def test_load_yaml(self) -> None:
        yaml_content = """
tracks:
  - track_id: "alpha-1"
    team: "blue"
    vehicle_role: "recon"
    waypoints:
      - {t: 0, lat: -35.2975, lon: 149.1012, heading: 0}
      - {t: 10, lat: -35.2965, lon: 149.1012, heading: 0}
  - track_id: "bravo-1"
    team: "red"
    vehicle_role: "tank"
    waypoints:
      - {t: 0, lat: -35.2980, lon: 149.1020, heading: 90}
"""
        with tempfile.NamedTemporaryFile(suffix=".yaml", mode="w", delete=False) as f:
            f.write(yaml_content)
            f.flush()
            tracks = load_replay_yaml(Path(f.name))

        assert len(tracks) == 2
        assert tracks[0].track_id == "alpha-1"
        assert tracks[0].team == "blue"
        assert tracks[0].vehicle_role == "recon"
        assert len(tracks[0].waypoints) == 2
        assert tracks[1].track_id == "bravo-1"
        assert tracks[1].vehicle_role == "tank"
