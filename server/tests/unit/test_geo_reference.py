"""Tests for plato_pod.geo_reference — arena ↔ lat/lon coordinate transforms."""

from __future__ import annotations

import math

import pytest

from plato_pod.geo_reference import GeoReference

# UNSW Canberra campus
_CANBERRA = GeoReference(origin_lat=-35.2975, origin_lon=149.1012, origin_alt=580.0)


class TestArenaToLatLon:
    def test_origin_maps_exactly(self) -> None:
        lat, lon = _CANBERRA.arena_to_latlon(0.0, 0.0)
        assert lat == pytest.approx(-35.2975, abs=1e-10)
        assert lon == pytest.approx(149.1012, abs=1e-10)

    def test_100m_north(self) -> None:
        """100m north should increase lat by ~0.0009 degrees."""
        lat, lon = _CANBERRA.arena_to_latlon(100.0, 0.0)
        delta_lat = lat - (-35.2975)
        assert delta_lat == pytest.approx(0.0009, abs=0.0001)
        # Longitude should barely change
        delta_lon = lon - 149.1012
        assert abs(delta_lon) < 1e-8

    def test_100m_east(self) -> None:
        """100m east should increase lon by ~0.0012 degrees at Canberra."""
        lat, lon = _CANBERRA.arena_to_latlon(0.0, 100.0)
        delta_lon = lon - 149.1012
        assert delta_lon == pytest.approx(0.0012, abs=0.0002)

    def test_negative_coordinates(self) -> None:
        lat, lon = _CANBERRA.arena_to_latlon(-50.0, -50.0)
        assert lat < -35.2975  # south of origin
        assert lon < 149.1012  # west of origin


class TestLatLonToArena:
    def test_origin_maps_exactly(self) -> None:
        x, y = _CANBERRA.latlon_to_arena(-35.2975, 149.1012)
        assert x == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)


class TestRoundTrip:
    def test_round_trip_small(self) -> None:
        """arena → latlon → arena should be identity within 1mm."""
        for ax, ay in [(0.5, 0.3), (1.0, -0.5), (-2.0, 3.0), (100.0, 50.0)]:
            lat, lon = _CANBERRA.arena_to_latlon(ax, ay)
            rx, ry = _CANBERRA.latlon_to_arena(lat, lon)
            assert rx == pytest.approx(ax, abs=0.001)
            assert ry == pytest.approx(ay, abs=0.001)

    def test_round_trip_latlon(self) -> None:
        """latlon → arena → latlon should be identity within 1e-8 degrees."""
        for lat, lon in [(-35.298, 149.102), (-35.296, 149.100)]:
            x, y = _CANBERRA.latlon_to_arena(lat, lon)
            rlat, rlon = _CANBERRA.arena_to_latlon(x, y)
            assert rlat == pytest.approx(lat, abs=1e-8)
            assert rlon == pytest.approx(lon, abs=1e-8)


class TestRotation:
    def test_90_degree_rotation(self) -> None:
        """With 90° CW rotation, arena +x maps to geographic east."""
        geo = GeoReference(origin_lat=-35.2975, origin_lon=149.1012, rotation_deg=90.0)
        lat, lon = geo.arena_to_latlon(100.0, 0.0)
        # +x with 90° rotation → east → lon increases, lat unchanged
        assert lon > 149.1012
        assert lat == pytest.approx(-35.2975, abs=0.0001)

    def test_0_rotation_x_is_north(self) -> None:
        """With 0° rotation (default), arena +x maps to north."""
        geo = GeoReference(origin_lat=-35.2975, origin_lon=149.1012, rotation_deg=0.0)
        lat, lon = geo.arena_to_latlon(100.0, 0.0)
        assert lat > -35.2975  # north
        assert lon == pytest.approx(149.1012, abs=1e-6)

    def test_rotation_round_trip(self) -> None:
        """Round trip with rotation should be identity within 1mm."""
        geo = GeoReference(origin_lat=0.0, origin_lon=0.0, rotation_deg=45.0)
        lat, lon = geo.arena_to_latlon(10.0, 5.0)
        rx, ry = geo.latlon_to_arena(lat, lon)
        assert rx == pytest.approx(10.0, abs=0.001)
        assert ry == pytest.approx(5.0, abs=0.001)


class TestScaleFactor:
    def test_scale_1_unchanged(self) -> None:
        """scale_factor=1.0 should produce identical results to no scaling."""
        geo = GeoReference(origin_lat=-35.2975, origin_lon=149.1012, scale_factor=1.0)
        lat, lon = geo.arena_to_latlon(100.0, 50.0)
        lat2, lon2 = _CANBERRA.arena_to_latlon(100.0, 50.0)
        assert lat == pytest.approx(lat2, abs=1e-12)
        assert lon == pytest.approx(lon2, abs=1e-12)

    def test_scale_1000_equivalent(self) -> None:
        """arena (0.3, 0.2) at scale 1000 should equal arena (300, 200) at scale 1."""
        geo_scaled = GeoReference(origin_lat=-35.2975, origin_lon=149.1012, scale_factor=1000.0)
        geo_real = GeoReference(origin_lat=-35.2975, origin_lon=149.1012, scale_factor=1.0)

        lat_s, lon_s = geo_scaled.arena_to_latlon(0.3, 0.2)
        lat_r, lon_r = geo_real.arena_to_latlon(300.0, 200.0)
        assert lat_s == pytest.approx(lat_r, abs=1e-10)
        assert lon_s == pytest.approx(lon_r, abs=1e-10)

    def test_round_trip_with_scale(self) -> None:
        """arena → latlon → arena round trip preserves desktop-scale coords."""
        geo = GeoReference(origin_lat=-35.2975, origin_lon=149.1012, scale_factor=500.0)
        for ax, ay in [(0.5, 0.3), (0.1, 0.8), (0.84, 0.59)]:
            lat, lon = geo.arena_to_latlon(ax, ay)
            rx, ry = geo.latlon_to_arena(lat, lon)
            assert rx == pytest.approx(ax, abs=0.001)
            assert ry == pytest.approx(ay, abs=0.001)

    def test_arena_to_world(self) -> None:
        geo = GeoReference(origin_lat=0, origin_lon=0, scale_factor=1000.0)
        wx, wy = geo.arena_to_world(0.3, 0.2)
        assert wx == pytest.approx(300.0)
        assert wy == pytest.approx(200.0)

    def test_world_to_arena(self) -> None:
        geo = GeoReference(origin_lat=0, origin_lon=0, scale_factor=1000.0)
        ax, ay = geo.world_to_arena(300.0, 200.0)
        assert ax == pytest.approx(0.3)
        assert ay == pytest.approx(0.2)

    def test_arena_world_round_trip(self) -> None:
        geo = GeoReference(origin_lat=0, origin_lon=0, scale_factor=750.0)
        for x, y in [(0.5, 0.3), (0.0, 0.0), (0.84, 0.59)]:
            wx, wy = geo.arena_to_world(x, y)
            rx, ry = geo.world_to_arena(wx, wy)
            assert rx == pytest.approx(x, abs=1e-12)
            assert ry == pytest.approx(y, abs=1e-12)

    def test_scale_with_rotation(self) -> None:
        """Scale + rotation should compose correctly."""
        geo_s = GeoReference(origin_lat=-35.2975, origin_lon=149.1012,
                             rotation_deg=45.0, scale_factor=100.0)
        geo_r = GeoReference(origin_lat=-35.2975, origin_lon=149.1012,
                             rotation_deg=45.0, scale_factor=1.0)
        lat_s, lon_s = geo_s.arena_to_latlon(1.0, 0.5)
        lat_r, lon_r = geo_r.arena_to_latlon(100.0, 50.0)
        assert lat_s == pytest.approx(lat_r, abs=1e-10)
        assert lon_s == pytest.approx(lon_r, abs=1e-10)


class TestDifferentLatitudes:
    def test_equator(self) -> None:
        geo = GeoReference(origin_lat=0.0, origin_lon=0.0)
        lat, lon = geo.arena_to_latlon(100.0, 100.0)
        rx, ry = geo.latlon_to_arena(lat, lon)
        assert rx == pytest.approx(100.0, abs=0.001)
        assert ry == pytest.approx(100.0, abs=0.001)

    def test_high_latitude(self) -> None:
        geo = GeoReference(origin_lat=60.0, origin_lon=10.0)
        lat, lon = geo.arena_to_latlon(50.0, 50.0)
        rx, ry = geo.latlon_to_arena(lat, lon)
        assert rx == pytest.approx(50.0, abs=0.01)
        assert ry == pytest.approx(50.0, abs=0.01)

    def test_southern_hemisphere(self) -> None:
        geo = GeoReference(origin_lat=-60.0, origin_lon=70.0)
        lat, lon = geo.arena_to_latlon(200.0, -100.0)
        rx, ry = geo.latlon_to_arena(lat, lon)
        assert rx == pytest.approx(200.0, abs=0.01)
        assert ry == pytest.approx(-100.0, abs=0.01)
