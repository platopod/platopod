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


def make_tombstone_event(uid: str) -> str:
    """Emit a CoT delete event for one UID — iTAK / ATAK will drop the marker.

    iTAK and ATAK don't all recognise the same delete signal, so this
    event packs four known mechanisms together:

      1. `type="t-x-d-d"` (data delete) at the event level — ATAK Civ
         3.x+ and FreeTAKServer recognise this.
      2. `<_remove uid="..."/>` element inside `<detail>` — older ATAK
         builds and ATAK Forge fork.
      3. `<link uid=... relation="p-p" type="t-x-d-d"/>` — additional
         signal honoured by some forks for orphan-link removal.
      4. `stale < time` — generic CoT staleness, makes sure standards-
         compliant receivers expire the event regardless of mechanism.

    The `how` attribute is `h-g-i-g-o` (human-generated-internal-
    generated-operator), the canonical "how" for operator deletes.

    For events archived in ATAK's local database, sending this MAY not
    be enough — archived markers can only be reliably cleared by
    deleting the iTAK / ATAK local data (Delete App + reinstall on
    iOS, or Clear App Data on Android). The tombstone reliably
    suppresses live updates and removes non-archived markers.
    """
    now = _utc_now()
    past = now - datetime.timedelta(seconds=60)

    event = ET.Element("event")
    event.set("version", "2.0")
    event.set("uid", uid)
    event.set("type", "t-x-d-d")
    event.set("time", _iso_format(now))
    event.set("start", _iso_format(past))
    event.set("stale", _iso_format(past))
    event.set("how", "h-g-i-g-o")

    point = ET.SubElement(event, "point")
    point.set("lat", "0.0")
    point.set("lon", "0.0")
    point.set("hae", "0.0")
    point.set("ce", "9999999.0")
    point.set("le", "9999999.0")

    detail = ET.SubElement(event, "detail")
    # ATAK Civ-style remove element
    remove = ET.SubElement(detail, "_remove")
    remove.set("uid", uid)
    # Older ATAK link-relation delete signal
    link = ET.SubElement(detail, "link")
    link.set("uid", uid)
    link.set("type", "t-x-d-d")
    link.set("relation", "p-p")

    return ET.tostring(event, encoding="unicode")


def make_stale_update_event(uid: str, original_type: str) -> str:
    """Emit a same-UID, same-type event with stale far in the past.

    This is the second half of the iTAK cleanup pattern: even when iTAK
    ignores `t-x-d-d` events, it processes UPDATES to existing markers.
    By sending an update with the original type but stale-in-past, iTAK
    expires the marker on its next refresh cycle.

    Use in combination with `make_tombstone_event`: send both per UID,
    one of them will work depending on the iTAK build.
    """
    now = _utc_now()
    past = now - datetime.timedelta(seconds=600)

    event = ET.Element("event")
    event.set("version", "2.0")
    event.set("uid", uid)
    event.set("type", original_type)
    event.set("time", _iso_format(now))
    event.set("start", _iso_format(past))
    event.set("stale", _iso_format(past))
    event.set("how", "h-g-i-g-o")

    point = ET.SubElement(event, "point")
    point.set("lat", "0.0")
    point.set("lon", "0.0")
    point.set("hae", "0.0")
    point.set("ce", "9999999.0")
    point.set("le", "9999999.0")

    ET.SubElement(event, "detail")
    return ET.tostring(event, encoding="unicode")


def make_remarks_detail(text: str) -> str:
    """Generate <remarks>text</remarks> XML fragment (ATAK tap-to-view)."""
    safe = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    return f"<remarks>{safe}</remarks>"


def make_engagement_event(
    actor_uid: str,
    target_uid: str,
    weapon: str,
    outcome: str,
    lat: float,
    lon: float,
    rationale: str = "",
    stale_seconds: float = 30.0,
) -> str:
    """Generate a CoT alert marking an engagement.

    Uses a combat marker type (`b-r-f-h-c` — combat: fire) at the contact
    location. Remarks include actor → target, weapon, outcome.
    """
    uid = f"engagement-{actor_uid}-{target_uid}-{int(_utc_now().timestamp())}"
    remarks = (
        f"Engagement: {actor_uid} -> {target_uid} | weapon={weapon} | "
        f"outcome={outcome}"
    )
    if rationale:
        remarks += f" | {rationale}"
    detail = make_contact_detail(f"FIRE_{weapon}") + make_remarks_detail(remarks)
    return make_cot_event(
        uid=uid, cot_type="b-r-f-h-c",
        lat=lat, lon=lon,
        stale_seconds=stale_seconds,
        detail_xml=detail,
    )


def make_casualty_event(
    robot_uid: str,
    callsign: str,
    lat: float,
    lon: float,
    status: str,
    health: float = 0.0,
    stale_seconds: float = 600.0,
) -> str:
    """Generate a casualty marker for a wounded/destroyed unit.

    Uses MIL-STD-2525B `a-h-G-X` (hostile, ground, casualty) for destroyed
    units regardless of original team — engagement outcomes are the focus.
    """
    cot_type = "a-h-G-X" if status == "destroyed" else "a-f-G-X"
    detail = (
        make_contact_detail(callsign)
        + make_remarks_detail(
            f"Casualty: {callsign} status={status} health={health:.2f}"
        )
    )
    return make_cot_event(
        uid=robot_uid, cot_type=cot_type,
        lat=lat, lon=lon,
        stale_seconds=stale_seconds,
        detail_xml=detail,
    )


def make_ied_marker(
    uid: str,
    lat: float,
    lon: float,
    confidence: float,
    label: str = "IED",
    stale_seconds: float = 3600.0,
) -> str:
    """CBRN/IED hazard marker (`u-d-c-c` — drawing CBRN)."""
    remarks = f"{label} confidence={confidence:.2f}"
    return make_cot_event(
        uid=uid, cot_type="u-d-c-c",
        lat=lat, lon=lon,
        stale_seconds=stale_seconds,
        detail_xml=make_contact_detail(label) + make_remarks_detail(remarks),
    )


def make_civilian_marker(
    uid: str,
    lat: float,
    lon: float,
    label: str = "civilian",
    stale_seconds: float = 300.0,
) -> str:
    """Neutral civilian marker (`a-n-G`)."""
    return make_cot_event(
        uid=uid, cot_type="a-n-G",
        lat=lat, lon=lon,
        stale_seconds=stale_seconds,
        detail_xml=make_contact_detail(label),
    )


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


def make_plume_ellipse_event(
    uid: str,
    center_lat: float,
    center_lon: float,
    major_axis_m: float,
    minor_axis_m: float,
    angle_deg: float,
    threshold_value: float,
    label: str = "",
    color: str | None = None,
    stale_seconds: float = 30.0,
) -> str:
    """Emit a parametric ellipse CoT event for a plume contour.

    ATAK renders ellipses with only ONE control-point handle (the
    centre), regardless of how detailed the ellipse is. That makes
    ellipses dramatically cleaner than polygons for a CBRN plume picture
    where multiple nested contours would otherwise produce dozens of
    vertex handles cluttering the operator's screen.

    Args:
        uid: Unique identifier (stable across redraws).
        center_lat / center_lon: ellipse centre in WGS84.
        major_axis_m / minor_axis_m: full axis lengths in metres.
        angle_deg: rotation of the major axis, degrees clockwise from
            north (standard CoT convention).
        threshold_value: concentration threshold this contour represents
            (used for default colour selection).
        label: optional label.
        color: ARGB hex override; default by threshold magnitude.
        stale_seconds: TTL on ATAK before the shape is dropped.
    """
    if color is None:
        color = _plume_color_for_threshold(threshold_value)
    if not label:
        label = f"≥{threshold_value:.0f}"
    return _make_ellipse_drawing(
        uid=uid,
        center_lat=center_lat,
        center_lon=center_lon,
        major_axis_m=major_axis_m,
        minor_axis_m=minor_axis_m,
        angle_deg=angle_deg,
        color=color,
        label=label,
        stale_seconds=stale_seconds,
    )


def make_plume_contour_event(
    uid: str,
    points: list[tuple[float, float]],
    threshold_value: float,
    label: str = "",
    color: str | None = None,
    stale_seconds: float = 30.0,
) -> str:
    """Generate a CoT freehand-polygon event for a contamination contour.

    Uses CoT type `u-d-f` (User Drawing Freehand) rather than `u-d-r`
    (User Drawing Rectangle) so ATAK renders the actual polygon outline
    instead of its axis-aligned bounding box. Default colour follows
    NATO CBRN doctrine by magnitude:
        red    (≥ 1000)  acute / IDLH zone
        orange (≥  500)  cross-contamination zone
        yellow (≥  100)  caution / detect threshold
        green  ( else )  trace / clean transit

    The caller can override `color` to use a custom palette.
    """
    if color is None:
        color = _plume_color_for_threshold(threshold_value)
    if not label:
        label = f"≥{threshold_value:.0f}"
    return _make_polygon_drawing(
        uid=uid,
        points=points,
        cot_type="u-d-f",
        color=color,
        label=label,
        stale_seconds=stale_seconds,
    )


def _plume_color_for_threshold(threshold: float) -> str:
    """ARGB hex colour for a CBRN contour by concentration magnitude."""
    if threshold >= 1000:
        return "ffff0000"   # red — acute / IDLH
    if threshold >= 500:
        return "ffff8000"   # orange — cross-contamination
    if threshold >= 100:
        return "ffffff00"   # yellow — caution
    return "ff00ff00"        # green — trace


def _argb_hex_to_signed_int(argb_hex: str) -> int:
    """Convert ARGB hex string (e.g. 'ff00ff00') to signed 32-bit int for ATAK."""
    s = argb_hex.lstrip("#")
    if len(s) == 6:
        s = "ff" + s  # add full alpha
    val = int(s, 16)
    if val >= 0x80000000:
        val -= 0x100000000
    return val


def make_shape_event(
    uid: str,
    points: list[tuple[float, float]],
    color: str = "ff0000ff",
    label: str = "",
    stale_seconds: float = 60.0,
) -> str:
    """Generate a CoT shape event (polygon) for arena boundaries or plume contours.

    Uses the ATAK/WinTAK drawing-shape XML format with signed-integer
    colours and 'relation="c"' on link elements.

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

    stroke_int = _argb_hex_to_signed_int(color)
    # Fill = same colour but ~20% alpha
    fill_hex = "33" + color[2:] if len(color) >= 8 else "33" + color
    fill_int = _argb_hex_to_signed_int(fill_hex)

    event = ET.Element("event")
    event.set("version", "2.0")
    event.set("uid", uid)
    event.set("type", "u-d-r")  # drawing region (closed polygon)
    event.set("time", _iso_format(now))
    event.set("start", _iso_format(now))
    event.set("stale", _iso_format(stale))
    event.set("how", "h-e")  # human-entered (drawing)

    # Centroid as the event location
    if points:
        clat = sum(p[0] for p in points) / len(points)
        clon = sum(p[1] for p in points) / len(points)
        point = ET.SubElement(event, "point")
        point.set("lat", f"{clat:.7f}")
        point.set("lon", f"{clon:.7f}")
        point.set("hae", "0.0")
        point.set("ce", "9999999.0")
        point.set("le", "9999999.0")
    else:
        point = ET.SubElement(event, "point")
        point.set("lat", "0"); point.set("lon", "0")
        point.set("hae", "0"); point.set("ce", "0"); point.set("le", "0")

    detail = ET.SubElement(event, "detail")

    # Shape vertices — ATAK polygon format: type="b-m-p-c" marks them as
    # control points of a shape (not standalone waypoints), relation="c"
    # links them as part of this shape.
    for i, (lat, lon) in enumerate(points):
        link = ET.SubElement(detail, "link")
        link.set("uid", f"{uid}-pt-{i}")
        link.set("type", "b-m-p-c")
        link.set("relation", "c")
        link.set("point", f"{lat:.7f},{lon:.7f}")

    # Stroke and fill — ATAK expects signed-int values
    stroke = ET.SubElement(detail, "strokeColor")
    stroke.set("value", str(stroke_int))
    weight = ET.SubElement(detail, "strokeWeight")
    weight.set("value", "3.0")
    fill = ET.SubElement(detail, "fillColor")
    fill.set("value", str(fill_int))

    # link_attr: alternate styling element used by some ATAK builds
    link_attr = ET.SubElement(detail, "link_attr")
    link_attr.set("line_color", str(stroke_int))
    link_attr.set("line_thickness", "3")
    link_attr.set("fill_color", str(fill_int))

    # Persistent across stale (so it stays visible)
    ET.SubElement(detail, "archive")

    if label:
        contact = ET.SubElement(detail, "contact")
        contact.set("callsign", label)
        lbl = ET.SubElement(detail, "labels_on")
        lbl.set("value", "true")

    return ET.tostring(event, encoding="unicode")


def _make_ellipse_drawing(
    uid: str,
    center_lat: float,
    center_lon: float,
    major_axis_m: float,
    minor_axis_m: float,
    angle_deg: float,
    color: str,
    label: str = "",
    stale_seconds: float = 30.0,
) -> str:
    """Generate a parametric ellipse CoT drawing event.

    Uses ATAK's CoT type `u-d-c` (drawing circle/ellipse). The shape is
    described by axes in metres and a rotation angle, so ATAK renders it
    smoothly without per-vertex control handles.
    """
    now = _utc_now()
    stale = now + datetime.timedelta(seconds=stale_seconds)

    stroke_int = _argb_hex_to_signed_int(color)
    fill_hex = "33" + color[2:] if len(color) >= 8 else "33" + color
    fill_int = _argb_hex_to_signed_int(fill_hex)

    event = ET.Element("event")
    event.set("version", "2.0")
    event.set("uid", uid)
    event.set("type", "u-d-c")
    event.set("time", _iso_format(now))
    event.set("start", _iso_format(now))
    event.set("stale", _iso_format(stale))
    event.set("how", "h-e")

    point = ET.SubElement(event, "point")
    point.set("lat", f"{center_lat:.7f}")
    point.set("lon", f"{center_lon:.7f}")
    point.set("hae", "0.0")
    point.set("ce", "9999999.0")
    point.set("le", "9999999.0")

    detail = ET.SubElement(event, "detail")

    # ATAK ellipse / shape sub-tree. Several ATAK forks accept slightly
    # different schemas; emitting both <shape><ellipse/></shape> and a
    # flat <ellipse/> covers the common dialects.
    shape = ET.SubElement(detail, "shape")
    ellipse = ET.SubElement(shape, "ellipse")
    ellipse.set("major", f"{major_axis_m:.2f}")
    ellipse.set("minor", f"{minor_axis_m:.2f}")
    ellipse.set("angle", f"{angle_deg:.2f}")

    flat_ellipse = ET.SubElement(detail, "ellipse")
    flat_ellipse.set("major", f"{major_axis_m:.2f}")
    flat_ellipse.set("minor", f"{minor_axis_m:.2f}")
    flat_ellipse.set("angle", f"{angle_deg:.2f}")

    stroke = ET.SubElement(detail, "strokeColor")
    stroke.set("value", str(stroke_int))
    weight = ET.SubElement(detail, "strokeWeight")
    weight.set("value", "3.0")
    fill = ET.SubElement(detail, "fillColor")
    fill.set("value", str(fill_int))

    link_attr = ET.SubElement(detail, "link_attr")
    link_attr.set("line_color", str(stroke_int))
    link_attr.set("line_thickness", "3")
    link_attr.set("fill_color", str(fill_int))

    # iTAK requires <archive> for u-d-c ellipses to render reliably.
    # Cleanup of stale archived markers is handled by the diff-tracking
    # tombstone mechanism in cot_bridge_node.
    ET.SubElement(detail, "archive")

    if label:
        contact = ET.SubElement(detail, "contact")
        contact.set("callsign", label)
        lbl = ET.SubElement(detail, "labels_on")
        lbl.set("value", "true")

    return ET.tostring(event, encoding="unicode")


def _make_polygon_drawing(
    uid: str,
    points: list[tuple[float, float]],
    cot_type: str,
    color: str,
    label: str = "",
    stale_seconds: float = 30.0,
) -> str:
    """Generate a closed-polygon CoT drawing event with a custom CoT type.

    Like `make_shape_event` but lets the caller choose the CoT type
    (e.g. `u-d-f` freehand polygon vs. `u-d-r` rectangle). The mandatory
    `<point>` element gets a degenerate accuracy footprint (ce/le very
    large) so ATAK doesn't render a centroid marker on top of the shape.
    """
    now = _utc_now()
    stale = now + datetime.timedelta(seconds=stale_seconds)

    stroke_int = _argb_hex_to_signed_int(color)
    fill_hex = "33" + color[2:] if len(color) >= 8 else "33" + color
    fill_int = _argb_hex_to_signed_int(fill_hex)

    event = ET.Element("event")
    event.set("version", "2.0")
    event.set("uid", uid)
    event.set("type", cot_type)
    event.set("time", _iso_format(now))
    event.set("start", _iso_format(now))
    event.set("stale", _iso_format(stale))
    event.set("how", "h-e")

    if points:
        clat = sum(p[0] for p in points) / len(points)
        clon = sum(p[1] for p in points) / len(points)
    else:
        clat = clon = 0.0
    point = ET.SubElement(event, "point")
    point.set("lat", f"{clat:.7f}")
    point.set("lon", f"{clon:.7f}")
    point.set("hae", "0.0")
    point.set("ce", "9999999.0")
    point.set("le", "9999999.0")

    detail = ET.SubElement(event, "detail")
    for i, (lat, lon) in enumerate(points):
        link = ET.SubElement(detail, "link")
        link.set("uid", f"{uid}-pt-{i}")
        link.set("type", "b-m-p-c")
        link.set("relation", "c")
        link.set("point", f"{lat:.7f},{lon:.7f}")

    stroke = ET.SubElement(detail, "strokeColor")
    stroke.set("value", str(stroke_int))
    weight = ET.SubElement(detail, "strokeWeight")
    weight.set("value", "3.0")
    fill = ET.SubElement(detail, "fillColor")
    fill.set("value", str(fill_int))

    link_attr = ET.SubElement(detail, "link_attr")
    link_attr.set("line_color", str(stroke_int))
    link_attr.set("line_thickness", "3")
    link_attr.set("fill_color", str(fill_int))

    # iTAK requires <archive> for u-d-r / u-d-f shapes to render at all.
    # The "stuck markers" risk is now handled by cot_bridge_node's
    # diff-tracking tombstone mechanism — when a UID disappears from the
    # active set, a delete event is sent. Net effect: visible while the
    # bridge is alive, gone when it stops or the contour shrinks below
    # threshold, and not stuck across scenarios.
    ET.SubElement(detail, "archive")

    if label:
        contact = ET.SubElement(detail, "contact")
        contact.set("callsign", label)
        lbl = ET.SubElement(detail, "labels_on")
        lbl.set("value", "true")

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
