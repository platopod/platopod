#!/usr/bin/env python3
"""Lightweight CoT map viewer — receives CoT XML on UDP and serves a web map.

Listens for Cursor on Target events on a UDP port and displays them on a
Leaflet map in the browser. Simpler alternative to FreeTAKServer for
development and testing.

Usage:
    python3 tools/cot_viewer.py [--port 4242] [--web-port 8090]
    # Open http://localhost:8090 in a browser

Then launch Plato Pod with target_host:=127.0.0.1 target_port:=4242
"""

import argparse
import asyncio
import json
import socket
import threading
import xml.etree.ElementTree as ET
from http.server import HTTPServer, SimpleHTTPRequestHandler

# Shared state: latest CoT events keyed by UID
_events: dict[str, dict] = {}
_lock = threading.Lock()


def parse_cot_to_dict(xml_str: str) -> dict | None:
    """Parse CoT XML into a simple dict for the web client."""
    try:
        root = ET.fromstring(xml_str)
    except ET.ParseError:
        return None
    if root.tag != "event":
        return None

    point = root.find("point")
    if point is None:
        return None

    result = {
        "uid": root.get("uid", ""),
        "type": root.get("type", ""),
        "lat": float(point.get("lat", "0")),
        "lon": float(point.get("lon", "0")),
        "hae": float(point.get("hae", "0")),
    }

    # Extract callsign
    detail = root.find("detail")
    if detail is not None:
        contact = detail.find("contact")
        if contact is not None:
            result["callsign"] = contact.get("callsign", "")
        track = detail.find("track")
        if track is not None:
            result["course"] = float(track.get("course", "0"))
        remarks = detail.find("remarks")
        if remarks is not None and remarks.text:
            result["remarks"] = remarks.text

        # Check for shape (polygon links)
        links = detail.findall("link")
        if links:
            result["shape"] = [
                [float(x) for x in l.get("point", "0,0").split(",")]
                for l in links
            ]

    return result


def udp_listener(port: int) -> None:
    """Listen for CoT events on UDP and update shared state."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    print(f"CoT listener on UDP :{port}")

    while True:
        try:
            data, addr = sock.recvfrom(65535)
            xml_str = data.decode("utf-8", errors="replace")
            event = parse_cot_to_dict(xml_str)
            if event:
                with _lock:
                    _events[event["uid"]] = event
        except Exception as e:
            print(f"UDP error: {e}")


HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
<title>Plato Pod CoT Viewer</title>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9/dist/leaflet.js"></script>
<style>
  body { margin: 0; font-family: monospace; }
  #map { height: 100vh; }
  #hud { position: absolute; top: 10px; right: 10px; z-index: 1000;
         background: rgba(0,0,0,0.7); color: #0f0; padding: 10px;
         border-radius: 5px; font-size: 13px; }
  .unit-label .leaflet-tooltip { background: rgba(0,0,0,0.75); color: #fff;
         border: none; font-family: monospace; font-size: 11px; padding: 2px 5px; }
  .unit-label .leaflet-tooltip-top::before { border-top-color: rgba(0,0,0,0.75); }
</style>
</head>
<body>
<div id="map"></div>
<div id="hud">Plato Pod CoT Viewer<br><span id="count">0 units</span></div>
<script>
const map = L.map('map').setView([-35.2935, 149.1670], 17);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 22, attribution: 'OSM'
}).addTo(map);

const markers = {};
const shapes = {};

function getColor(type) {
  if (type.startsWith('a-f-')) return '#2563eb'; // friendly blue
  if (type.startsWith('a-h-')) return '#dc2626'; // hostile red
  if (type.startsWith('a-u-')) return '#ca8a04'; // unknown yellow
  if (type === 'u-d-f') return '#16a34a';         // shape green
  return '#888';
}

function getBorderColor(type) {
  if (type.startsWith('a-f-')) return '#1e40af'; // darker blue
  if (type.startsWith('a-h-')) return '#991b1b'; // darker red
  if (type.startsWith('a-u-')) return '#854d0e'; // darker yellow
  return '#555';
}

// MIL-STD type code → vehicle info (label, real-world radius in metres)
function getVehicleInfo(type) {
  const t = type || '';
  // Tanks (armour) — ~6m long
  if (t.includes('-U-C-A')) return {label: 'TANK', radius_m: 4.0};
  // APC/IFV (infantry carrier) — ~5m long
  if (t.includes('-U-C-I')) return {label: 'APC', radius_m: 3.0};
  // Artillery — ~6m
  if (t.includes('-U-C-F')) return {label: 'ARTY', radius_m: 3.5};
  // Recon vehicle — ~2m
  if (t.includes('-U-R'))  return {label: 'RCN', radius_m: 1.5};
  // Sensor/equipment — ~1m
  if (t.includes('-E-S'))  return {label: 'SNS', radius_m: 0.8};
  // Default ground unit (soldier/squad) — ~1m
  if (t.startsWith('a-'))  return {label: 'GND', radius_m: 1.0};
  return {label: '?', radius_m: 1.0};
}

function updateMap(events) {
  let unitCount = 0;
  for (const [uid, e] of Object.entries(events)) {
    // Shapes (polygons)
    if (e.shape) {
      if (shapes[uid]) map.removeLayer(shapes[uid]);
      const latlngs = e.shape.map(p => [p[0], p[1]]);
      if (latlngs.length >= 2) {
        shapes[uid] = L.polygon(latlngs, {
          color: getColor(e.type), weight: 2, fillOpacity: 0.15
        }).addTo(map);
        if (e.callsign) shapes[uid].bindTooltip(e.callsign);
      }
      continue;
    }

    // Unit markers
    unitCount++;
    const color = getColor(e.type);
    const veh = getVehicleInfo(e.type);
    const label = e.callsign || uid;
    const tooltip = veh.label + ' ' + label + (e.remarks ? '\\n' + e.remarks : '');

    if (markers[uid] && markers[uid].group) {
      // Update existing — move all layers in the group
      const ll = [e.lat, e.lon];
      markers[uid].group.eachLayer(l => {
        if (l.setLatLng) l.setLatLng(ll);
      });
      // Update heading line
      if (markers[uid].heading && e.course !== undefined) {
        const hRad = (90 - e.course) * Math.PI / 180;
        const headLen = veh.radius_m * 3;
        const dLat = headLen * Math.cos(hRad) / 111320;
        const dLon = headLen * Math.sin(hRad) / (111320 * Math.cos(e.lat * Math.PI / 180));
        markers[uid].heading.setLatLngs([ll, [e.lat + dLat, e.lon + dLon]]);
      }
      markers[uid].tooltip.setLatLng(ll);
      markers[uid].tooltip.setTooltipContent(tooltip);
    } else {
      // Create new unit marker group
      const group = L.layerGroup().addTo(map);
      const borderColor = getBorderColor(e.type);

      // Main body — L.circle scales with zoom (real-world metres)
      const body = L.circle([e.lat, e.lon], {
        radius: veh.radius_m,
        color: borderColor, fillColor: color, fillOpacity: 0.7, weight: 2
      }).addTo(group);

      // Heading line
      let headingLine = null;
      if (e.course !== undefined) {
        const hRad = (90 - e.course) * Math.PI / 180;
        const headLen = veh.radius_m * 3;
        const dLat = headLen * Math.cos(hRad) / 111320;
        const dLon = headLen * Math.sin(hRad) / (111320 * Math.cos(e.lat * Math.PI / 180));
        headingLine = L.polyline(
          [[e.lat, e.lon], [e.lat + dLat, e.lon + dLon]],
          {color: borderColor, weight: 2, dashArray: '4,4'}
        ).addTo(group);
      }

      // Label
      const labelMarker = L.circleMarker([e.lat, e.lon], {radius: 0, opacity: 0})
        .addTo(group)
        .bindTooltip(tooltip, {permanent: true, direction: 'top', offset: [0, -12], className: 'unit-label'});

      markers[uid] = {group, body, heading: headingLine, tooltip: labelMarker};

      // Auto-fit on first unit
      if (unitCount === 1) map.setView([e.lat, e.lon], 18);
    }
  }
  document.getElementById('count').textContent = unitCount + ' units';
}

setInterval(async () => {
  try {
    const resp = await fetch('/api/events');
    const events = await resp.json();
    updateMap(events);
  } catch (e) {}
}, 500);
</script>
</body>
</html>"""


class ViewerHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())
        elif self.path == "/api/events":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            with _lock:
                self.wfile.write(json.dumps(_events).encode())
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass  # suppress request logging


def main():
    parser = argparse.ArgumentParser(description="Plato Pod CoT Map Viewer")
    parser.add_argument("--port", type=int, default=4242, help="UDP port for CoT input")
    parser.add_argument("--web-port", type=int, default=8090, help="HTTP port for web map")
    args = parser.parse_args()

    # Start UDP listener
    t = threading.Thread(target=udp_listener, args=(args.port,), daemon=True)
    t.start()

    # Start web server
    server = HTTPServer(("0.0.0.0", args.web_port), ViewerHandler)
    print(f"Map viewer at http://localhost:{args.web_port}/")
    print(f"Launch Plato Pod with: target_host:=127.0.0.1 target_port:={args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
