"""Geographic coordinate reference for arena ↔ lat/lon conversion.

Maps between arena-local (x, y) in metres and geographic (lat, lon) in degrees
using WGS84 flat-earth approximation. Supports a scale_factor for classroom
exercises where a small desktop arena represents a larger real-world area.

No external dependencies — uses only math module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# WGS84 constants
_WGS84_A = 6378137.0                # equatorial radius in metres
_WGS84_F = 1.0 / 298.257223563      # flattening
_WGS84_E2 = 2 * _WGS84_F - _WGS84_F ** 2  # eccentricity squared


def _metres_per_deg_lat(lat_rad: float) -> float:
    """Metres per degree of latitude at the given latitude."""
    sin_lat = math.sin(lat_rad)
    denom = (1.0 - _WGS84_E2 * sin_lat * sin_lat) ** 1.5
    return math.pi / 180.0 * _WGS84_A * (1.0 - _WGS84_E2) / denom


def _metres_per_deg_lon(lat_rad: float) -> float:
    """Metres per degree of longitude at the given latitude."""
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    return math.pi / 180.0 * _WGS84_A * math.cos(lat_rad) / denom


@dataclass
class GeoReference:
    """Maps between arena-local (x, y) in metres and geographic (lat, lon).

    The arena coordinate frame has:
    - origin at (origin_lat, origin_lon)
    - x-axis rotated `rotation_deg` clockwise from true north

    When scale_factor > 1.0 (classroom mode), arena coordinates are scaled
    up before geographic conversion. A desktop arena of 0.84m with
    scale_factor=1000 represents 840m of real terrain.

    Uses flat-earth approximation (valid for effective areas up to ~2 km).
    """
    origin_lat: float           # degrees, arena (0,0) maps to this latitude
    origin_lon: float           # degrees, arena (0,0) maps to this longitude
    origin_alt: float = 0.0     # metres above WGS84 ellipsoid (HAE)
    rotation_deg: float = 0.0   # arena x-axis clockwise from north, degrees
    scale_factor: float = 1.0   # 1.0 = outdoor (1:1), >1 = classroom scaling

    def arena_to_world(self, x: float, y: float) -> tuple[float, float]:
        """Convert arena (x, y) to world-scale (x, y) by applying scale_factor.

        Args:
            x: Arena X position in metres (desktop scale).
            y: Arena Y position in metres (desktop scale).

        Returns:
            Tuple of (world_x, world_y) in real-world metres.
        """
        return x * self.scale_factor, y * self.scale_factor

    def world_to_arena(self, wx: float, wy: float) -> tuple[float, float]:
        """Convert world-scale (x, y) back to arena (x, y).

        Args:
            wx: World X position in real-world metres.
            wy: World Y position in real-world metres.

        Returns:
            Tuple of (arena_x, arena_y) in desktop-scale metres.
        """
        return wx / self.scale_factor, wy / self.scale_factor

    def arena_to_latlon(self, x: float, y: float) -> tuple[float, float]:
        """Convert arena (x, y) in metres to (lat, lon) in degrees.

        Applies scale_factor before geographic conversion, so a desktop
        robot at (0.3, 0.2) with scale_factor=1000 maps to the same
        lat/lon as a real-world point at (300, 200) metres from origin.

        Args:
            x: Arena X position in metres.
            y: Arena Y position in metres.

        Returns:
            Tuple of (latitude, longitude) in degrees.
        """
        # Scale to real-world metres
        wx, wy = self.arena_to_world(x, y)

        rot_rad = math.radians(self.rotation_deg)
        cos_r = math.cos(rot_rad)
        sin_r = math.sin(rot_rad)

        north_m = wx * cos_r - wy * sin_r
        east_m = wx * sin_r + wy * cos_r

        lat_rad = math.radians(self.origin_lat)
        m_per_deg_lat = _metres_per_deg_lat(lat_rad)
        m_per_deg_lon = _metres_per_deg_lon(lat_rad)

        lat = self.origin_lat + north_m / m_per_deg_lat
        lon = self.origin_lon + east_m / m_per_deg_lon

        return lat, lon

    def latlon_to_arena(self, lat: float, lon: float) -> tuple[float, float]:
        """Convert (lat, lon) in degrees to arena (x, y) in metres.

        Applies inverse scale_factor, so a real-world lat/lon maps back
        to desktop-scale arena coordinates.

        Args:
            lat: Latitude in degrees.
            lon: Longitude in degrees.

        Returns:
            Tuple of (x, y) in arena-local metres (desktop scale).
        """
        lat_rad = math.radians(self.origin_lat)
        m_per_deg_lat = _metres_per_deg_lat(lat_rad)
        m_per_deg_lon = _metres_per_deg_lon(lat_rad)

        north_m = (lat - self.origin_lat) * m_per_deg_lat
        east_m = (lon - self.origin_lon) * m_per_deg_lon

        rot_rad = math.radians(self.rotation_deg)
        cos_r = math.cos(rot_rad)
        sin_r = math.sin(rot_rad)

        wx = north_m * cos_r + east_m * sin_r
        wy = -north_m * sin_r + east_m * cos_r

        # Scale back to arena metres
        return self.world_to_arena(wx, wy)
