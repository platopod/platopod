"""Plume contour extraction — turn a SpatialField into smooth hazard polygons.

Given a `SpatialField` (gas concentration, radiation, etc.) and a bounding
box, sample on a grid and return the boundary polygons of regions where
the field meets or exceeds each threshold. Output is a small number of
smooth polygons per threshold — suitable for direct rendering on ATAK as
nested CBRN hazard zones.

Pipeline:
  1. Sample the field on a regular grid.
  2. Mark cells with any corner above the threshold; build their union.
  3. **Morphological close** (buffer +ε then -ε) to fill small gaps and
     round the cell-aligned boundary into a smooth blob.
  4. Drop islands smaller than a few cells (configurable).
  5. Apply Douglas-Peucker simplification (configurable).
  6. Cap the number of polygons per threshold to avoid clutter.

Pure Python; uses Shapely for geometry ops (already a project dependency).
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
    component, ordered largest-first. Each polygon is closed (first and
    last vertex equal).
    """
    threshold: float
    label: str
    polygons: list[list[tuple[float, float]]]


def extract_contours(
    field: "SpatialField",
    bbox: tuple[float, float, float, float],
    thresholds: list[float],
    *,
    grid_size: int = 80,
    time: float = 0.0,
    simplify_tolerance: float = 0.0,
    smooth_amount: float = 0.0,
    min_polygon_area: float | None = None,
    max_polygons_per_level: int = 3,
) -> list[ContourLevel]:
    """Extract contour polygons of `field` at each value in `thresholds`.

    Args:
        field: any object with `.evaluate(x, y, t) -> float`.
        bbox: (xmin, ymin, xmax, ymax) in arena metres.
        thresholds: ascending list of threshold values. Higher threshold
            = innermost / more dangerous polygon.
        grid_size: number of samples along the longer bbox axis. Default
            80; bump for smoother contours, drop for cheaper compute.
        time: time argument passed to `field.evaluate`.
        simplify_tolerance: Douglas-Peucker tolerance (arena metres).
            0 picks `grid_resolution / 2` automatically.
        smooth_amount: morphological close radius (arena metres). 0 picks
            `grid_resolution * 1.5` automatically — strong enough to
            merge cell-aligned blocks into a smooth blob.
        min_polygon_area: drop polygons smaller than this (arena m²). 0
            picks `9 × grid_resolution²` (≈ 3×3 cell cluster).
        max_polygons_per_level: keep only the N largest polygons at each
            threshold. Default 3 — prevents dozens of tiny islands from
            cluttering the operator's screen.

    Returns:
        One ContourLevel per threshold, in input order. Polygons within
        a level are sorted largest-first.
    """
    xmin, ymin, xmax, ymax = bbox
    span = max(xmax - xmin, ymax - ymin)
    if span <= 0:
        return [ContourLevel(t, "", []) for t in thresholds]

    resolution = span / max(1, grid_size)
    nx = max(1, int(math.ceil((xmax - xmin) / resolution)))
    ny = max(1, int(math.ceil((ymax - ymin) / resolution)))

    if simplify_tolerance <= 0:
        # Default 4× grid resolution. Each polygon vertex becomes a
        # draggable control-point handle on ATAK; aggressive
        # simplification drops the handle count to ~4-5 per polygon
        # while still tracking the plume shape for operator awareness.
        # YAML can lower this for higher fidelity at the cost of more
        # handles, or raise it for a cleaner demo picture.
        simplify_tolerance = resolution * 4.0
    if smooth_amount <= 0:
        smooth_amount = resolution * 2.0
    if min_polygon_area is None:
        min_polygon_area = 9.0 * resolution * resolution

    # Sample grid once; reuse for every threshold.
    grid: list[list[float]] = [
        [field.evaluate(xmin + ix * resolution, ymin + iy * resolution, time)
         for iy in range(ny + 1)]
        for ix in range(nx + 1)
    ]

    out: list[ContourLevel] = []
    for thr in thresholds:
        polys = _polygons_above(
            grid, xmin, ymin, resolution, nx, ny, thr,
            smooth_amount=smooth_amount,
            simplify_tolerance=simplify_tolerance,
            min_polygon_area=min_polygon_area,
            max_polygons=max_polygons_per_level,
        )
        out.append(ContourLevel(threshold=thr, label="", polygons=polys))
    return out


def _polygons_above(
    grid: list[list[float]],
    xmin: float, ymin: float, resolution: float,
    nx: int, ny: int, threshold: float,
    *,
    smooth_amount: float,
    simplify_tolerance: float,
    min_polygon_area: float,
    max_polygons: int,
) -> list[list[tuple[float, float]]]:
    """Cell-union → morphological close → simplify → cap → list of polygons.

    A cell is "above" if any of its four corner samples reach the threshold.
    The half-cell overestimate is consistent with NATO doctrine practice —
    flag a slightly-too-large hazard zone rather than miss the edge.
    """
    marked: list[list[bool]] = [
        [
            (grid[ix][iy] >= threshold
             or grid[ix + 1][iy] >= threshold
             or grid[ix][iy + 1] >= threshold
             or grid[ix + 1][iy + 1] >= threshold)
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
        # Without shapely: return per-cell rectangles (no merge / smoothing).
        return [
            _cell_corners(xmin, ymin, ix, iy, resolution)
            for ix in range(nx) for iy in range(ny) if marked[ix][iy]
        ][:max_polygons]

    cells = [
        Polygon(_cell_corners(xmin, ymin, ix, iy, resolution))
        for ix in range(nx) for iy in range(ny) if marked[ix][iy]
    ]
    if not cells:
        return []

    union = unary_union(cells)

    # Morphological close: dilate then erode. Fills small gaps inside the
    # blob and rounds the cell-aligned boundary into a smooth curve.
    if smooth_amount > 0:
        union = union.buffer(smooth_amount, join_style=1).buffer(
            -smooth_amount, join_style=1,
        )

    if union.is_empty:
        return []

    # Collect polygons, drop tiny islands, sort largest-first.
    candidates: list[tuple[float, "Polygon"]] = []
    if union.geom_type == "Polygon":
        if union.area >= min_polygon_area:
            candidates.append((union.area, union))
    elif union.geom_type == "MultiPolygon":
        for p in union.geoms:
            if p.area >= min_polygon_area:
                candidates.append((p.area, p))
    candidates.sort(key=lambda x: -x[0])
    candidates = candidates[:max_polygons]

    polygons: list[list[tuple[float, float]]] = []
    for _, poly in candidates:
        if simplify_tolerance > 0:
            poly = poly.simplify(simplify_tolerance, preserve_topology=True)
        if poly.is_empty or poly.geom_type != "Polygon":
            continue
        polygons.append([(float(x), float(y)) for x, y in poly.exterior.coords])
    return polygons


def fit_ellipse(
    polygon: list[tuple[float, float]],
) -> tuple[float, float, float, float, float] | None:
    """Fit a covering ellipse to a polygon's vertices via PCA.

    Returns:
        (center_x, center_y, major_axis_m, minor_axis_m, angle_rad) or
        None if the polygon is degenerate. `angle_rad` is the bearing of
        the major axis measured CCW from +x. Axes are FULL lengths
        (not radii).

    The result is the smallest axis-aligned-in-PCA-frame box that
    contains all vertices, expressed as an ellipse. It's a quick
    visual approximation, not a least-squares ellipse fit — but for
    plume contours it produces a clean, doctrine-friendly oval.
    """
    if len(polygon) < 3:
        return None

    n = len(polygon)
    cx = sum(p[0] for p in polygon) / n
    cy = sum(p[1] for p in polygon) / n

    # 2x2 covariance matrix
    sxx = sum((p[0] - cx) ** 2 for p in polygon) / n
    syy = sum((p[1] - cy) ** 2 for p in polygon) / n
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in polygon) / n

    # Eigenvalues of [[sxx, sxy], [sxy, syy]]
    trace = sxx + syy
    det = sxx * syy - sxy * sxy
    disc = max(0.0, trace * trace / 4 - det)
    sd = math.sqrt(disc)
    lam1 = trace / 2 + sd       # larger eigenvalue (major variance)
    lam2 = max(0.0, trace / 2 - sd)

    # Eigenvector for the larger eigenvalue
    if abs(sxy) > 1e-12:
        angle = math.atan2(lam1 - sxx, sxy)
    elif sxx >= syy:
        angle = 0.0
    else:
        angle = math.pi / 2

    # Project each vertex onto the principal axes and take the extent.
    # This bounds the ellipse so it covers all vertices.
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    max_u = 0.0
    max_v = 0.0
    for x, y in polygon:
        dx = x - cx
        dy = y - cy
        u = abs(dx * cos_a + dy * sin_a)
        v = abs(-dx * sin_a + dy * cos_a)
        if u > max_u:
            max_u = u
        if v > max_v:
            max_v = v

    return (cx, cy, max_u * 2.0, max_v * 2.0, angle)


def _cell_corners(
    xmin: float, ymin: float, ix: int, iy: int, resolution: float,
) -> list[tuple[float, float]]:
    """Closed polygon (5 vertices) for a grid cell, CCW from bottom-left."""
    x0 = xmin + ix * resolution
    y0 = ymin + iy * resolution
    x1 = x0 + resolution
    y1 = y0 + resolution
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
