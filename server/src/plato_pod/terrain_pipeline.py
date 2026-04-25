"""DEM terrain pipeline — converts elevation data to Gazebo heightmaps.

Loads Digital Elevation Model data from GeoTIFF files or numpy arrays,
crops to a geographic bounding box, resamples to Gazebo-compatible
dimensions (2^n + 1), and writes 16-bit PNG heightmaps.

Supports SRTM (30m), ALOS (12.5m), or any GeoTIFF via rasterio.
Falls back to numpy-only loading when rasterio is unavailable.

No ROS2 dependency.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

# Try to import optional dependencies
try:
    from PIL import Image
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

try:
    import rasterio
    from rasterio.transform import from_bounds
    from rasterio.warp import reproject, Resampling
    _HAS_RASTERIO = True
except ImportError:
    _HAS_RASTERIO = False


@dataclass
class BoundingBox:
    """Geographic bounding box in WGS84 degrees."""
    north: float    # max latitude
    south: float    # min latitude
    east: float     # max longitude
    west: float     # min longitude

    @property
    def width_deg(self) -> float:
        return self.east - self.west

    @property
    def height_deg(self) -> float:
        return self.north - self.south

    def width_m(self, lat: float | None = None) -> float:
        """Approximate width in metres at the given latitude."""
        lat_rad = math.radians(lat or (self.north + self.south) / 2)
        m_per_deg = 111320.0 * math.cos(lat_rad)
        return self.width_deg * m_per_deg

    def height_m(self) -> float:
        """Approximate height in metres."""
        return self.height_deg * 110540.0


@dataclass
class TerrainConfig:
    """Configuration for DEM acquisition and heightmap generation."""
    source: str = "flat"            # "flat" | "geotiff" | "numpy"
    bounding_box: BoundingBox | None = None
    geotiff_path: str = ""          # path to local GeoTIFF file
    heightmap_size: int = 257       # must be 2^n + 1 for Gazebo
    max_elevation_m: float = 50.0   # vertical scale in Gazebo
    texture_path: str = ""          # optional satellite imagery for visual


@dataclass
class TerrainResult:
    """Output of the terrain pipeline."""
    heightmap_path: Path            # path to generated PNG
    size_x: float                   # real-world X dimension in metres
    size_y: float                   # real-world Y dimension in metres
    size_z: float                   # vertical range in metres
    min_elevation: float            # minimum elevation in source data
    max_elevation: float            # maximum elevation in source data
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0)


def validate_heightmap_size(size: int) -> int:
    """Ensure size is 2^n + 1 (Gazebo requirement). Round up if needed."""
    if size < 3:
        return 3
    # Find smallest 2^n + 1 >= size
    n = 1
    while (1 << n) + 1 < size:
        n += 1
    return (1 << n) + 1


def load_geotiff(path: str, bbox: BoundingBox | None = None) -> np.ndarray:
    """Load elevation data from a GeoTIFF file.

    Args:
        path: Path to the GeoTIFF file.
        bbox: Optional bounding box to crop to. If None, loads entire file.

    Returns:
        2D numpy array of elevation values in metres.

    Raises:
        ImportError: If rasterio is not installed.
        FileNotFoundError: If the file doesn't exist.
    """
    if not _HAS_RASTERIO:
        raise ImportError(
            "rasterio is required for GeoTIFF loading. "
            "Install with: pip install rasterio"
        )

    with rasterio.open(path) as src:
        if bbox is not None:
            # Compute pixel window from bounding box
            window = src.window(bbox.west, bbox.south, bbox.east, bbox.north)
            elevation = src.read(1, window=window)
        else:
            elevation = src.read(1)

    # Replace nodata with 0
    elevation = np.nan_to_num(elevation, nan=0.0)
    return elevation.astype(np.float64)


def load_numpy(path: str) -> np.ndarray:
    """Load elevation data from a numpy .npy file."""
    return np.load(path).astype(np.float64)


def resample_elevation(
    elevation: np.ndarray,
    target_size: int,
) -> np.ndarray:
    """Resample elevation grid to target dimensions using bilinear interpolation.

    Args:
        elevation: Source elevation array (any shape).
        target_size: Target dimensions (square output: target_size x target_size).

    Returns:
        Resampled elevation array of shape (target_size, target_size).
    """
    src_h, src_w = elevation.shape
    if src_h == target_size and src_w == target_size:
        return elevation.copy()

    # Bilinear interpolation using numpy
    y_indices = np.linspace(0, src_h - 1, target_size)
    x_indices = np.linspace(0, src_w - 1, target_size)

    # Create meshgrid
    xi, yi = np.meshgrid(x_indices, y_indices)

    # Integer parts and fractional parts
    x0 = np.floor(xi).astype(int)
    y0 = np.floor(yi).astype(int)
    x1 = np.minimum(x0 + 1, src_w - 1)
    y1 = np.minimum(y0 + 1, src_h - 1)
    fx = xi - x0
    fy = yi - y0

    # Bilinear interpolation
    result = (
        elevation[y0, x0] * (1 - fx) * (1 - fy)
        + elevation[y0, x1] * fx * (1 - fy)
        + elevation[y1, x0] * (1 - fx) * fy
        + elevation[y1, x1] * fx * fy
    )

    return result


def elevation_to_png(
    elevation: np.ndarray,
    output_path: Path,
    max_elevation: float | None = None,
) -> tuple[float, float]:
    """Write elevation array as a 16-bit grayscale PNG.

    Normalizes elevation values to 0–65535. Returns the actual
    min and max elevation values used for normalization.

    Args:
        elevation: 2D array of elevation values in metres.
        output_path: Where to write the PNG file.
        max_elevation: If given, map this elevation to 65535.
            If None, use the data's actual max.

    Returns:
        (min_elevation, max_elevation) used for normalization.
    """
    if not _HAS_PIL:
        raise ImportError(
            "Pillow is required for PNG heightmap generation. "
            "Install with: pip install Pillow"
        )

    e_min = float(np.min(elevation))
    e_max = float(np.max(elevation))

    if max_elevation is not None:
        e_range = max_elevation
    else:
        e_range = e_max - e_min

    if e_range < 0.01:
        # Flat terrain — write all zeros
        normalized = np.zeros_like(elevation, dtype=np.uint16)
    else:
        normalized = ((elevation - e_min) / e_range * 65535).clip(0, 65535).astype(np.uint16)

    img = Image.fromarray(normalized, mode="I;16")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(str(output_path))

    logger.info(
        f"Heightmap written: {output_path} "
        f"({elevation.shape[1]}x{elevation.shape[0]}, "
        f"elev {e_min:.1f}–{e_max:.1f}m)"
    )

    return e_min, e_max


def generate_terrain(
    config: TerrainConfig,
    output_dir: Path,
) -> TerrainResult | None:
    """Full terrain pipeline: load → resample → write PNG.

    Args:
        config: Terrain configuration.
        output_dir: Directory to write the heightmap PNG.

    Returns:
        TerrainResult with paths and metadata, or None for flat terrain.
    """
    if config.source == "flat":
        return None

    # Load elevation data
    if config.source == "geotiff":
        if not config.geotiff_path:
            logger.error("geotiff source requires geotiff_path")
            return None
        elevation = load_geotiff(config.geotiff_path, config.bounding_box)
    elif config.source == "numpy":
        if not config.geotiff_path:  # reuse path field for numpy files
            logger.error("numpy source requires geotiff_path (path to .npy)")
            return None
        elevation = load_numpy(config.geotiff_path)
    else:
        logger.error(f"Unknown terrain source: {config.source}")
        return None

    # Validate and fix heightmap size
    target_size = validate_heightmap_size(config.heightmap_size)
    if target_size != config.heightmap_size:
        logger.info(
            f"Adjusted heightmap size from {config.heightmap_size} to {target_size} "
            f"(must be 2^n + 1 for Gazebo)"
        )

    # Resample
    resampled = resample_elevation(elevation, target_size)

    # Write PNG
    output_path = Path(output_dir) / "heightmap.png"
    e_min, e_max = elevation_to_png(resampled, output_path, config.max_elevation_m)

    # Compute real-world dimensions
    if config.bounding_box is not None:
        size_x = config.bounding_box.height_m()   # north-south = X
        size_y = config.bounding_box.width_m()     # east-west = Y
    else:
        # Assume 1m per pixel if no bounding box
        size_x = float(target_size)
        size_y = float(target_size)

    size_z = config.max_elevation_m if config.max_elevation_m > 0 else (e_max - e_min)

    return TerrainResult(
        heightmap_path=output_path,
        size_x=size_x,
        size_y=size_y,
        size_z=size_z,
        min_elevation=e_min,
        max_elevation=e_max,
    )


def terrain_config_from_yaml(exercise_config: dict) -> TerrainConfig | None:
    """Parse terrain configuration from exercise YAML.

    Args:
        exercise_config: The exercise YAML dict (with or without 'exercise' wrapper).

    Returns:
        TerrainConfig or None if no terrain section.
    """
    exercise = exercise_config.get("exercise", exercise_config)
    terrain = exercise.get("terrain")
    if terrain is None:
        return None

    source = terrain.get("source", "flat")
    if source == "flat":
        return TerrainConfig(source="flat")

    bbox = None
    bbox_cfg = terrain.get("bounding_box")
    if bbox_cfg:
        bbox = BoundingBox(
            north=float(bbox_cfg["north"]),
            south=float(bbox_cfg["south"]),
            east=float(bbox_cfg["east"]),
            west=float(bbox_cfg["west"]),
        )

    return TerrainConfig(
        source=source,
        bounding_box=bbox,
        geotiff_path=terrain.get("geotiff_path", ""),
        heightmap_size=int(terrain.get("heightmap_size", 257)),
        max_elevation_m=float(terrain.get("max_elevation", 50.0)),
        texture_path=terrain.get("texture_path", ""),
    )
