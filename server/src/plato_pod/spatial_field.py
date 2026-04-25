"""Spatial field abstractions for virtual data layers.

A SpatialField represents a value that varies over space and time — gas
concentration, terrain elevation, RF signal strength, temperature, etc.
Fields are evaluated at robot positions to produce virtual sensor readings.

All fields are pure Python (Layer 3) with no ROS2 dependency.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Protocol


class SpatialField(Protocol):
    """A spatial field that can be evaluated at any (x, y, t).

    Implementations must be thread-safe for concurrent evaluation.
    """

    name: str

    def evaluate(self, x: float, y: float, t: float) -> float:
        """Evaluate the field at position (x, y) at time t.

        Args:
            x: X coordinate in arena frame (metres).
            y: Y coordinate in arena frame (metres).
            t: Time in seconds (exercise clock).

        Returns:
            Field value at the given point. Units depend on the field type.
        """
        ...

    def get_metadata(self) -> dict:
        """Return metadata describing the field for clients.

        Returns:
            Dict with at minimum 'name' and 'units' keys.
        """
        ...


@dataclass
class GaussianPlumeField:
    """Steady-state Gaussian plume dispersion model.

    Models gas concentration downwind of a point source using the standard
    Gaussian plume equation. Assumes flat terrain and constant wind.

    Concentration at (x, y) relative to source:
        C = Q / (2*pi*u*sigma_y*sigma_z) * exp(-0.5*(y'/sigma_y)^2)

    where x', y' are rotated into wind-aligned coordinates,
    sigma_y = sigma_z = sqrt(2*D*x'/u), and D is the diffusion coefficient.

    Units: concentration in arbitrary units (ppm, mg/m3, etc.) depending on
    release_rate units.
    """

    source_x: float             # Source position X (metres)
    source_y: float             # Source position Y (metres)
    release_rate: float         # Q: emission rate (mass/time)
    wind_speed: float           # u: wind speed (m/s)
    wind_direction: float       # Wind direction in radians (direction wind blows TO)
    diffusion_coeff: float      # D: diffusion coefficient (m^2/s)
    name: str = "gaussian_plume"

    def evaluate(self, x: float, y: float, t: float) -> float:
        """Evaluate gas concentration at (x, y, t).

        Returns 0.0 if the point is upwind of the source or wind is calm.
        """
        if self.wind_speed < 1e-6 or self.release_rate <= 0:
            return 0.0

        # Translate to source-relative coordinates
        dx = x - self.source_x
        dy = y - self.source_y

        # Rotate into wind-aligned frame (x' = downwind, y' = crosswind)
        cos_w = math.cos(self.wind_direction)
        sin_w = math.sin(self.wind_direction)
        x_wind = dx * cos_w + dy * sin_w    # downwind distance
        y_wind = -dx * sin_w + dy * cos_w   # crosswind distance

        # No concentration upwind of source
        if x_wind <= 0:
            return 0.0

        # Dispersion parameters (Pasquill-Gifford simplified)
        sigma = math.sqrt(2.0 * self.diffusion_coeff * x_wind / self.wind_speed)
        if sigma < 1e-9:
            return 0.0

        # Gaussian plume equation (ground-level, no reflection)
        exp_term = math.exp(-0.5 * (y_wind / sigma) ** 2)
        concentration = (
            self.release_rate / (2.0 * math.pi * self.wind_speed * sigma * sigma)
        ) * exp_term

        return max(0.0, concentration)

    def get_metadata(self) -> dict:
        return {
            "name": self.name,
            "units": "ppm",
            "type": "gaussian_plume",
            "source_x": self.source_x,
            "source_y": self.source_y,
            "release_rate": self.release_rate,
            "wind_speed": self.wind_speed,
            "wind_direction": self.wind_direction,
        }


@dataclass
class ElevationField:
    """Virtual terrain elevation from a regular grid.

    Interpolates elevation values from a 2D grid. Used for terrain speed
    penalties and line-of-sight calculations.

    Grid is stored row-major: grid_data[row][col] where row increases with Y
    and col increases with X.
    """

    grid_data: list[list[float]]   # 2D elevation grid (metres)
    origin_x: float                # Grid origin X (metres)
    origin_y: float                # Grid origin Y (metres)
    resolution: float              # Cell size (metres)
    name: str = "elevation"

    def evaluate(self, x: float, y: float, t: float) -> float:
        """Return interpolated elevation at (x, y). Time is ignored.

        Returns 0.0 for points outside the grid.
        """
        if not self.grid_data or not self.grid_data[0]:
            return 0.0

        rows = len(self.grid_data)
        cols = len(self.grid_data[0])

        # Convert to grid coordinates
        gx = (x - self.origin_x) / self.resolution
        gy = (y - self.origin_y) / self.resolution

        # Bilinear interpolation
        ix = int(math.floor(gx))
        iy = int(math.floor(gy))

        if ix < 0 or iy < 0 or ix >= cols - 1 or iy >= rows - 1:
            # Clamp to nearest edge value if partially outside
            cx = max(0, min(cols - 1, round(gx)))
            cy = max(0, min(rows - 1, round(gy)))
            if 0 <= cx < cols and 0 <= cy < rows:
                return self.grid_data[cy][cx]
            return 0.0

        fx = gx - ix
        fy = gy - iy

        v00 = self.grid_data[iy][ix]
        v10 = self.grid_data[iy][ix + 1]
        v01 = self.grid_data[iy + 1][ix]
        v11 = self.grid_data[iy + 1][ix + 1]

        return (
            v00 * (1 - fx) * (1 - fy)
            + v10 * fx * (1 - fy)
            + v01 * (1 - fx) * fy
            + v11 * fx * fy
        )

    def get_metadata(self) -> dict:
        rows = len(self.grid_data) if self.grid_data else 0
        cols = len(self.grid_data[0]) if self.grid_data and self.grid_data[0] else 0
        return {
            "name": self.name,
            "units": "metres",
            "type": "elevation_grid",
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "resolution": self.resolution,
            "rows": rows,
            "cols": cols,
        }


@dataclass
class UniformField:
    """Constant value everywhere. Useful for background levels or testing."""

    value: float
    name: str = "uniform"

    def evaluate(self, x: float, y: float, t: float) -> float:
        return self.value

    def get_metadata(self) -> dict:
        return {
            "name": self.name,
            "units": "arbitrary",
            "type": "uniform",
            "value": self.value,
        }


@dataclass
class CompositeField:
    """Sum of multiple spatial fields. Allows layering effects."""

    fields: list[SpatialField] = field(default_factory=list)
    name: str = "composite"

    def evaluate(self, x: float, y: float, t: float) -> float:
        return sum(f.evaluate(x, y, t) for f in self.fields)

    def get_metadata(self) -> dict:
        return {
            "name": self.name,
            "units": "mixed",
            "type": "composite",
            "num_fields": len(self.fields),
            "fields": [f.get_metadata() for f in self.fields],
        }


def compute_iso_contour(
    field: SpatialField,
    threshold: float,
    bounds: tuple[float, float, float, float],
    resolution: float = 0.02,
    t: float = 0.0,
) -> list[tuple[float, float]]:
    """Compute a simple iso-concentration contour as a convex boundary.

    Samples the field on a grid and returns the convex hull of all points
    where the field value exceeds the threshold. This is a fast approximation
    suitable for generating ATAK overlays, not a precise marching-squares
    contour.

    Args:
        field: Spatial field to sample.
        threshold: Iso-value threshold.
        bounds: (x_min, y_min, x_max, y_max) sampling region.
        resolution: Grid cell size in metres.
        t: Time parameter.

    Returns:
        List of (x, y) points forming the contour boundary. Empty if no
        points exceed the threshold.
    """
    x_min, y_min, x_max, y_max = bounds
    above: list[tuple[float, float]] = []

    x = x_min
    while x <= x_max:
        y = y_min
        while y <= y_max:
            if field.evaluate(x, y, t) >= threshold:
                above.append((x, y))
            y += resolution
        x += resolution

    if len(above) < 3:
        return above

    # Convex hull (Graham scan)
    from plato_pod.geometry import convex_hull
    return convex_hull(above)
