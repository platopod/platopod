"""Cursor on Target (CoT) XML generation and parsing.

Pure Python CoT protocol implementation using only stdlib xml.etree.
Generates CoT events for robot positions, arena boundaries, and sensor
readings. Parses inbound CoT events for ATAK waypoints.

No external dependencies.
"""

from __future__ import annotations

import datetime
import xml.etree.ElementTree as ET

# Vehicle role → MIL-STD-2525B CoT type codes
VEHICLE_ROLE_TO_COT_TYPE: dict[str, str] = {
    "default":      "a-f-G",
    "tank":         "a-f-G-U-C-A",
    "apc":          "a-f-G-U-C-I",
    "recon":        "a-f-G-U-R",
    "cbrn_recon":   "a-f-G-U-R",
    "artillery":    "a-f-G-U-C-F",
    "hostile":      "a-h-G",
    "hostile_tank": "a-h-G-U-C-A",
    "unknown":      "a-u-G",
    "sensor":       "a-f-G-E-S",
}


def _utc_now() -> datetime.datetime:
    return datetime.datetime.now(datetime.UTC)


def _iso_format(dt: datetime.datetime) -> str:
    """Format datetime as CoT ISO 8601 with Z suffix."""
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def make_cot_event(
    uid: str,
    cot_type: str,
    lat: float,
    lon: float,
    hae: float = 0.0,
    ce: float = 10.0,
    le: float = 10.0,
    how: str = "m-g",
    stale_seconds: float = 30.0,
    detail_xml: str = "",
) -> str:
    """Generate a complete CoT event XML string.

    Args:
        uid: Unique identifier for this event.
        cot_type: CoT type code (e.g. "a-f-G").
        lat: Latitude in degrees.
        lon: Longitude in degrees.
        hae: Height above WGS84 ellipsoid in metres.
        ce: Circular error in metres.
        le: Linear error in metres.
        how: How the position was determined ("m-g" = machine-generated).
        stale_seconds: Seconds until the event goes stale.
        detail_xml: Optional XML fragment for the <detail> element.

    Returns:
        CoT event as a UTF-8 XML string (no XML declaration).
    """
    now = _utc_now()
    stale = now + datetime.timedelta(seconds=stale_seconds)

    event = ET.Element("event")
    event.set("version", "2.0")
    event.set("uid", uid)
    event.set("type", cot_type)
    event.set("time", _iso_format(now))
    event.set("start", _iso_format(now))
    event.set("stale", _iso_format(stale))
    event.set("how", how)

    point = ET.SubElement(event, "point")
    point.set("lat", f"{lat:.7f}")
    point.set("lon", f"{lon:.7f}")
    point.set("hae", f"{hae:.1f}")
    point.set("ce", f"{ce:.1f}")
    point.set("le", f"{le:.1f}")

    detail = ET.SubElement(event, "detail")
    if detail_xml:
        for child in ET.fromstring(f"<d>{detail_xml}</d>"):
            detail.append(child)

    return ET.tostring(event, encoding="unicode")


def make_contact_detail(callsign: str) -> str:
    """Generate <contact callsign="..."/> XML fragment."""
    return f'<contact callsign="{callsign}"/>'


def make_track_detail(course: float, speed: float) -> str:
    """Generate <track course="..." speed="..."/> XML fragment.

    Args:
        course: Heading in degrees clockwise from north (0-360).
        speed: Ground speed in m/s.
    """
    return f'<track course="{course:.1f}" speed="{speed:.2f}"/>'


def make_sensor_detail(readings: dict[str, float], model_name: str = "") -> str:
    """Generate custom <sensor> detail XML fragment for gas sensor readings.

    Args:
        readings: e.g. {"acetone": 420.0, "ethanol": 0.0}
        model_name: e.g. "fractional-tempered"
    """
    parts = [f'<sensor model="{model_name}">']
    for name, value in readings.items():
        parts.append(f'  <reading name="{name}" value="{value:.4f}"/>')
    parts.append("</sensor>")
    return "\n".join(parts)


def make_shape_event(
    uid: str,
    points: list[tuple[float, float]],
    color: str = "ff0000ff",
    label: str = "",
    stale_seconds: float = 60.0,
) -> str:
    """Generate a CoT shape event (polygon) for arena boundaries or plume contours.

    Args:
        uid: Unique identifier.
        points: List of (lat, lon) vertices.
        color: ARGB hex colour string.
        label: Optional label text.
        stale_seconds: Seconds until stale.

    Returns:
        CoT shape event as XML string.
    """
    now = _utc_now()
    stale = now + datetime.timedelta(seconds=stale_seconds)

    event = ET.Element("event")
    event.set("version", "2.0")
    event.set("uid", uid)
    event.set("type", "u-d-f")  # drawing/shape/freeform
    event.set("time", _iso_format(now))
    event.set("start", _iso_format(now))
    event.set("stale", _iso_format(stale))
    event.set("how", "m-g")

    # First point as the event location
    if points:
        point = ET.SubElement(event, "point")
        point.set("lat", f"{points[0][0]:.7f}")
        point.set("lon", f"{points[0][1]:.7f}")
        point.set("hae", "0.0")
        point.set("ce", "10.0")
        point.set("le", "10.0")
    else:
        point = ET.SubElement(event, "point")
        point.set("lat", "0"); point.set("lon", "0")
        point.set("hae", "0"); point.set("ce", "0"); point.set("le", "0")

    detail = ET.SubElement(event, "detail")

    # Shape links
    for lat, lon in points:
        link = ET.SubElement(detail, "link")
        link.set("point", f"{lat:.7f},{lon:.7f}")

    # Stroke color
    stroke = ET.SubElement(detail, "strokeColor")
    stroke.set("value", color)

    if label:
        lbl = ET.SubElement(detail, "labels_on")
        lbl.set("value", "true")
        contact = ET.SubElement(detail, "contact")
        contact.set("callsign", label)

    return ET.tostring(event, encoding="unicode")


def parse_cot_event(xml_str: str) -> dict | None:
    """Parse an incoming CoT event XML string into a dict.

    Args:
        xml_str: Raw CoT XML string.

    Returns:
        Dict with keys: uid, type, lat, lon, hae, time, stale, detail.
        None if parsing fails.
    """
    try:
        root = ET.fromstring(xml_str)
    except ET.ParseError:
        return None

    if root.tag != "event":
        return None

    point_el = root.find("point")
    if point_el is None:
        return None

    result = {
        "uid": root.get("uid", ""),
        "type": root.get("type", ""),
        "time": root.get("time", ""),
        "stale": root.get("stale", ""),
        "how": root.get("how", ""),
        "lat": float(point_el.get("lat", "0")),
        "lon": float(point_el.get("lon", "0")),
        "hae": float(point_el.get("hae", "0")),
    }

    detail_el = root.find("detail")
    if detail_el is not None:
        result["detail"] = ET.tostring(detail_el, encoding="unicode")
    else:
        result["detail"] = ""

    return result


def parse_nav_goal(cot_dict: dict) -> tuple[float, float] | None:
    """Extract a navigation waypoint (lat, lon) from a parsed CoT event.

    Recognises CoT types: b-m-p-w (waypoint), b-m-p-s-p-i (point of interest).

    Args:
        cot_dict: Parsed CoT event from parse_cot_event().

    Returns:
        (lat, lon) or None if the event is not a navigation target.
    """
    cot_type = cot_dict.get("type", "")
    nav_types = {"b-m-p-w", "b-m-p-s-p-i"}

    if cot_type in nav_types:
        return (cot_dict["lat"], cot_dict["lon"])
    return None


UID_PREFIX = "platopod-"


def robot_id_from_uid(uid: str) -> int:
    """Extract robot_id from a Plato Pod CoT UID.

    Outbound UIDs use the format 'platopod-{robot_id}'. For inbound
    waypoints from ATAK, this parses the UID to recover the target robot.

    Args:
        uid: CoT unique identifier string.

    Returns:
        Robot ID, or -1 if the UID doesn't match the expected format.
    """
    if uid.startswith(UID_PREFIX):
        try:
            return int(uid[len(UID_PREFIX):])
        except (ValueError, IndexError):
            pass
    return -1
