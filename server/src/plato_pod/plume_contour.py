"""Plume contour extraction — turn a SpatialField into polygon hazard zones.

Given a `SpatialField` (gas concentration, radiation, etc.) and a bounding
box, sample on a grid and return the boundary polygons of the regions where
the field meets or exceeds each threshold. Multiple disconnected components
become separate polygons — natural for plumes that wrap around obstacles or
have been cut by wind.

Pure Python; no ROS2 dependency. Used by `cot_bridge_node` to render the
contamination picture on iTAK/ATAK.

Implementation note: we use a square-cell-union approach rather than full
marching-squares interpolation. The boundary tracks the grid resolution
exactly, which is fine for ATAK rendering (lat/lon resolution is much
coarser than the ~1m grid). When the visualiser needs sub-cell precision
(e.g. for analytic Gaussian plumes shown at high zoom), refine the grid;
the algorithm is O(N) in cell count and Shapely's union is fast.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from plato_pod.spatial_field import SpatialField


@dataclass
class ContourLevel:
    """One threshold level extracted from a SpatialField sample.

    `polygons` is a list of polygon vertex lists — one per disconnected
    component. Each polygon is closed (first and last vertex equal).
    """
    threshold: float
    label: str
    polygons: list[list[tuple[float, float]]]


def extract_contours(
    field: "SpatialField",
    bbox: tuple[float, float, float, float],
    thresholds: list[float],
    *,
    grid_size: int = 60,
    time: float = 0.0,
    simplify_tolerance: float = 0.0,
) -> list[ContourLevel]:
    """Extract contour polygons of `field` for each value in `thresholds`.

    Args:
        field: any object with `.evaluate(x, y, t) -> float`.
        bbox: (xmin, ymin, xmax, ymax) in arena metres.
        thresholds: ascending list of threshold values. Highest threshold
            yields the innermost polygon.
        grid_size: number of samples along the longer bbox axis. Increase
            for smoother contours; decrease for cheaper compute.
        time: time argument passed to `field.evaluate`.
        simplify_tolerance: Douglas-Peucker tolerance (arena metres) for
            polygon simplification. 0 disables simplification.

    Returns:
        One ContourLevel per threshold, in input order. Empty thresholds
        (no cell met or exceeded the value) still appear with `polygons=[]`.
    """
    xmin, ymin, xmax, ymax = bbox
    span = max(xmax - xmin, ymax - ymin)
    if span <= 0:
        return [ContourLevel(t, "", []) for t in thresholds]

    # Square cells so the marching-squares-like boundary tracks correctly.
    resolution = span / max(1, grid_size)
    nx = max(1, int(math.ceil((xmax - xmin) / resolution)))
    ny = max(1, int(math.ceil((ymax - ymin) / resolution)))

    # Sample once; reuse for every threshold.
    grid: list[list[float]] = [
        [field.evaluate(xmin + ix * resolution, ymin + iy * resolution, time)
         for iy in range(ny + 1)]
        for ix in range(nx + 1)
    ]

    out: list[ContourLevel] = []
    for thr in thresholds:
        polys = _polygons_above(
            grid, xmin, ymin, resolution, nx, ny, thr,
            simplify_tolerance=simplify_tolerance,
        )
        out.append(ContourLevel(threshold=thr, label="", polygons=polys))
    return out


def _polygons_above(
    grid: list[list[float]],
    xmin: float, ymin: float, resolution: float,
    nx: int, ny: int, threshold: float,
    *, simplify_tolerance: float = 0.0,
) -> list[list[tuple[float, float]]]:
    """Return outer-boundary polygons of cells where corner samples >= threshold.

    A cell is "above" if any of its four corner samples reach the threshold.
    Strictly speaking this overestimates the contour by half a cell, but the
    overestimate is consistent with NATO doctrine practice — better to flag
    a slightly-too-large hazard zone than miss the edge.
    """
    # Mark cells that have at least one corner above the threshold
    marked: list[list[bool]] = [
        [
            any((
                grid[ix][iy] >= threshold,
                grid[ix + 1][iy] >= threshold,
                grid[ix][iy + 1] >= threshold,
                grid[ix + 1][iy + 1] >= threshold,
            ))
            for iy in range(ny)
        ]
        for ix in range(nx)
    ]

    if not any(any(row) for row in marked):
        return []

    try:
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
    except ImportError:
        # Without shapely we fall back to per-cell rectangles (no merge).
        return [
            _cell_corners(xmin, ymin, ix, iy, resolution)
            for ix in range(nx) for iy in range(ny) if marked[ix][iy]
        ]

    cells = [
        Polygon(_cell_corners(xmin, ymin, ix, iy, resolution))
        for ix in range(nx) for iy in range(ny) if marked[ix][iy]
    ]
    if not cells:
        return []

    union = unary_union(cells)

    polygons: list[list[tuple[float, float]]] = []

    def _emit(poly):
        coords = list(poly.exterior.coords)
        if simplify_tolerance > 0:
            simp = poly.simplify(simplify_tolerance, preserve_topology=True)
            coords = list(simp.exterior.coords)
        polygons.append([(float(x), float(y)) for x, y in coords])

    if union.geom_type == "Polygon":
        _emit(union)
    elif union.geom_type == "MultiPolygon":
        for p in union.geoms:
            _emit(p)
    return polygons


def _cell_corners(
    xmin: float, ymin: float, ix: int, iy: int, resolution: float,
) -> list[tuple[float, float]]:
    """Closed polygon (5 vertices) for a grid cell, CCW from bottom-left."""
    x0 = xmin + ix * resolution
    y0 = ymin + iy * resolution
    x1 = x0 + resolution
    y1 = y0 + resolution
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
