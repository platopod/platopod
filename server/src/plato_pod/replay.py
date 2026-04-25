"""Exercise replay — load and interpolate GPS tracks from recorded exercises.

Loads GPS tracks from GPX files or YAML waypoint lists, converts to
arena coordinates via GeoReference, and interpolates positions at
arbitrary times for playback.

No ROS2 dependency.
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

from plato_pod.geo_reference import GeoReference

logger = logging.getLogger(__name__)


@dataclass
class ReplayWaypoint:
    """A single point in a recorded track."""
    timestamp: float        # seconds since exercise start
    lat: float              # WGS84 degrees
    lon: float              # WGS84 degrees
    alt: float = 0.0        # metres above ellipsoid
    heading_deg: float = 0.0
    speed_mps: float = 0.0


@dataclass
class ReplayTrack:
    """A single unit's GPS track from a recorded exercise."""
    track_id: str           # unique track identifier
    robot_id: int = -1      # mapped robot ID (set during exercise setup)
    vehicle_role: str = "recon"
    team: str = ""
    waypoints: list[ReplayWaypoint] = field(default_factory=list)

    @property
    def duration(self) -> float:
        """Total track duration in seconds."""
        if len(self.waypoints) < 2:
            return 0.0
        return self.waypoints[-1].timestamp - self.waypoints[0].timestamp

    @property
    def start_time(self) -> float:
        return self.waypoints[0].timestamp if self.waypoints else 0.0


@dataclass
class ReplayExercise:
    """A complete recorded exercise with tracks and metadata."""
    name: str
    tracks: list[ReplayTrack] = field(default_factory=list)
    duration_seconds: float = 0.0

    @property
    def num_tracks(self) -> int:
        return len(self.tracks)


def interpolate_position(
    track: ReplayTrack, t: float
) -> tuple[float, float, float] | None:
    """Get (lat, lon, heading_deg) at time t by linear interpolation.

    Args:
        track: The GPS track to interpolate.
        t: Time in seconds since exercise start.

    Returns:
        (lat, lon, heading_deg) or None if t is outside the track's range.
    """
    wps = track.waypoints
    if not wps:
        return None

    # Clamp to track bounds
    if t <= wps[0].timestamp:
        return wps[0].lat, wps[0].lon, wps[0].heading_deg
    if t >= wps[-1].timestamp:
        return wps[-1].lat, wps[-1].lon, wps[-1].heading_deg

    # Find bracketing waypoints
    for i in range(len(wps) - 1):
        if wps[i].timestamp <= t <= wps[i + 1].timestamp:
            dt = wps[i + 1].timestamp - wps[i].timestamp
            if dt < 1e-6:
                return wps[i].lat, wps[i].lon, wps[i].heading_deg
            frac = (t - wps[i].timestamp) / dt

            lat = wps[i].lat + frac * (wps[i + 1].lat - wps[i].lat)
            lon = wps[i].lon + frac * (wps[i + 1].lon - wps[i].lon)

            # Interpolate heading (handle wraparound)
            h1 = wps[i].heading_deg
            h2 = wps[i + 1].heading_deg
            dh = ((h2 - h1 + 180) % 360) - 180
            heading = (h1 + frac * dh) % 360

            return lat, lon, heading

    return None


def interpolate_arena_position(
    track: ReplayTrack, t: float, geo: GeoReference
) -> tuple[float, float, float] | None:
    """Get (arena_x, arena_y, theta_rad) at time t.

    Converts from lat/lon to arena coordinates using GeoReference.

    Returns:
        (x, y, theta) in arena frame, or None if outside track range.
    """
    result = interpolate_position(track, t)
    if result is None:
        return None

    lat, lon, heading_deg = result
    x, y = geo.latlon_to_arena(lat, lon)
    theta = math.radians(90.0 - heading_deg)  # heading CW from north → theta CCW from east
    return x, y, theta


def load_gpx(path: Path) -> list[ReplayTrack]:
    """Load GPS tracks from a GPX file.

    GPX is the standard GPS exchange format. Each <trk> element becomes
    a ReplayTrack. Timestamps are converted to seconds from the first
    point in the file.

    Args:
        path: Path to the GPX file.

    Returns:
        List of ReplayTrack objects.
    """
    tree = ET.parse(path)
    root = tree.getroot()

    # GPX namespace
    ns = ""
    if root.tag.startswith("{"):
        ns = root.tag.split("}")[0] + "}"

    tracks = []
    first_time: float | None = None

    for trk_idx, trk in enumerate(root.findall(f"{ns}trk")):
        name_el = trk.find(f"{ns}name")
        track_id = name_el.text if name_el is not None and name_el.text else f"track_{trk_idx}"

        waypoints = []
        for seg in trk.findall(f"{ns}trkseg"):
            for pt in seg.findall(f"{ns}trkpt"):
                lat = float(pt.get("lat", "0"))
                lon = float(pt.get("lon", "0"))

                ele_el = pt.find(f"{ns}ele")
                alt = float(ele_el.text) if ele_el is not None and ele_el.text else 0.0

                time_el = pt.find(f"{ns}time")
                if time_el is not None and time_el.text:
                    from datetime import datetime, timezone
                    try:
                        dt = datetime.fromisoformat(
                            time_el.text.replace("Z", "+00:00")
                        )
                        ts = dt.timestamp()
                    except ValueError:
                        ts = 0.0
                else:
                    ts = 0.0

                if first_time is None:
                    first_time = ts

                waypoints.append(ReplayWaypoint(
                    timestamp=ts - (first_time or 0.0),
                    lat=lat,
                    lon=lon,
                    alt=alt,
                ))

        # Compute headings from consecutive points
        for i in range(len(waypoints) - 1):
            dlat = waypoints[i + 1].lat - waypoints[i].lat
            dlon = waypoints[i + 1].lon - waypoints[i].lon
            heading = math.degrees(math.atan2(dlon, dlat)) % 360
            waypoints[i].heading_deg = heading
        if len(waypoints) >= 2:
            waypoints[-1].heading_deg = waypoints[-2].heading_deg

        # Compute speeds
        for i in range(len(waypoints) - 1):
            dt = waypoints[i + 1].timestamp - waypoints[i].timestamp
            if dt > 0:
                dlat = (waypoints[i + 1].lat - waypoints[i].lat) * 110540
                dlon = (waypoints[i + 1].lon - waypoints[i].lon) * 111320 * math.cos(
                    math.radians(waypoints[i].lat)
                )
                dist = math.sqrt(dlat ** 2 + dlon ** 2)
                waypoints[i].speed_mps = dist / dt

        tracks.append(ReplayTrack(
            track_id=track_id,
            waypoints=waypoints,
        ))

    return tracks


def load_replay_yaml(path: Path) -> list[ReplayTrack]:
    """Load replay tracks from a YAML file.

    YAML format:
        tracks:
          - track_id: "alpha-1"
            team: "blue"
            vehicle_role: "recon"
            waypoints:
              - {t: 0, lat: -35.2975, lon: 149.1012, heading: 0}
              - {t: 10, lat: -35.2970, lon: 149.1015, heading: 45}
    """
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f)

    tracks = []
    for t in data.get("tracks", []):
        wps = []
        for w in t.get("waypoints", []):
            wps.append(ReplayWaypoint(
                timestamp=float(w.get("t", 0)),
                lat=float(w.get("lat", 0)),
                lon=float(w.get("lon", 0)),
                alt=float(w.get("alt", 0)),
                heading_deg=float(w.get("heading", 0)),
                speed_mps=float(w.get("speed", 0)),
            ))
        tracks.append(ReplayTrack(
            track_id=t.get("track_id", ""),
            vehicle_role=t.get("vehicle_role", "recon"),
            team=t.get("team", ""),
            waypoints=wps,
        ))

    return tracks
