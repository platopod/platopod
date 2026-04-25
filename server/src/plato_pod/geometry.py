"""Pure computational geometry for the Plato Pod arena.

Provides convex hull, point-in-polygon, polygon containment, and shape-to-polygon
conversions. Wraps Shapely in a clean functional API, confining the dependency
to this single module.

No ROS2 dependency.
"""

from __future__ import annotations

import math

from shapely.geometry import MultiPoint, Point, Polygon


def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Compute the convex hull of a set of 2D points.

    Args:
        points: List of (x, y) coordinates. Need at least 3 non-collinear points.

    Returns:
        Hull vertices in counter-clockwise order. Returns the input unchanged
        if fewer than 3 points are provided.
    """
    if len(points) < 3:
        return list(points)

    mp = MultiPoint(points)
    hull = mp.convex_hull

    if hull.geom_type == "Point":
        return [(hull.x, hull.y)]
    if hull.geom_type == "LineString":
        return list(hull.coords)

    # Polygon — extract exterior ring (Shapely returns CCW by default)
    coords = list(hull.exterior.coords)
    # Shapely closes the ring (first == last), remove the duplicate
    return coords[:-1]


def point_in_polygon(
    point: tuple[float, float],
    polygon: list[tuple[float, float]],
) -> bool:
    """Test if a point is inside a polygon (boundary counts as inside).

    Args:
        point: (x, y) coordinate.
        polygon: List of (x, y) vertices.

    Returns:
        True if the point is inside or on the boundary of the polygon.
    """
    if len(polygon) < 3:
        return False
    poly = Polygon(polygon)
    pt = Point(point)
    return poly.contains(pt) or poly.boundary.contains(pt)


def polygon_contains_polygon(
    outer: list[tuple[float, float]],
    inner: list[tuple[float, float]],
) -> tuple[bool, list[int]]:
    """Test if all vertices of the inner polygon are within the outer polygon.

    Args:
        outer: Vertices of the containing polygon.
        inner: Vertices of the polygon to test.

    Returns:
        Tuple of (all_inside, outside_indices) where outside_indices lists
        the indices of inner vertices that fall outside the outer polygon.
    """
    if len(outer) < 3:
        return False, list(range(len(inner)))

    outer_poly = Polygon(outer)
    outside = []
    for i, vertex in enumerate(inner):
        pt = Point(vertex)
        if not (outer_poly.contains(pt) or outer_poly.boundary.contains(pt)):
            outside.append(i)

    return len(outside) == 0, outside


def rectangle_to_polygon(
    x: float,
    y: float,
    width: float,
    height: float,
    rotation: float = 0.0,
) -> list[tuple[float, float]]:
    """Convert a rectangle (centre, dimensions, rotation) to a 4-vertex polygon.

    Args:
        x: Centre X coordinate.
        y: Centre Y coordinate.
        width: Width in metres.
        height: Height in metres.
        rotation: Rotation angle in radians (counter-clockwise).

    Returns:
        4 vertices in counter-clockwise order.
    """
    hw = width / 2.0
    hh = height / 2.0

    # Corners relative to centre (CCW order)
    corners = [
        (-hw, -hh),
        (hw, -hh),
        (hw, hh),
        (-hw, hh),
    ]

    if rotation != 0.0:
        cos_r = math.cos(rotation)
        sin_r = math.sin(rotation)
        corners = [
            (cx * cos_r - cy * sin_r, cx * sin_r + cy * cos_r)
            for cx, cy in corners
        ]

    return [(x + cx, y + cy) for cx, cy in corners]


def circle_to_polygon(
    x: float,
    y: float,
    radius: float,
    n_vertices: int = 16,
) -> list[tuple[float, float]]:
    """Approximate a circle as an N-gon.

    Args:
        x: Centre X coordinate.
        y: Centre Y coordinate.
        radius: Radius in metres.
        n_vertices: Number of polygon vertices (default 16).

    Returns:
        N vertices in counter-clockwise order.
    """
    vertices = []
    for i in range(n_vertices):
        angle = 2.0 * math.pi * i / n_vertices
        vx = x + radius * math.cos(angle)
        vy = y + radius * math.sin(angle)
        vertices.append((vx, vy))
    return vertices


def polygon_area(vertices: list[tuple[float, float]]) -> float:
    """Compute the signed area of a polygon using the shoelace formula.

    Positive area indicates counter-clockwise winding.

    Args:
        vertices: List of (x, y) coordinates.

    Returns:
        Signed area (positive for CCW, negative for CW).
    """
    n = len(vertices)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += vertices[i][0] * vertices[j][1]
        area -= vertices[j][0] * vertices[i][1]
    return area / 2.0


def to_shapely_polygon(vertices: list[tuple[float, float]]) -> Polygon:
    """Convert a vertex list to a Shapely Polygon.

    Used by downstream modules (e.g. sensor engine, command pipeline) that
    expect Shapely types for intersection tests.

    Args:
        vertices: List of (x, y) coordinates.

    Returns:
        Shapely Polygon object.
    """
    return Polygon(vertices)
