# TAK Integration — Setup Guide

This guide walks through setting up a complete TAK ecosystem alongside Plato Pod for tactical visualisation: **FreeTAKServer** (the relay) plus **WinTAK** or **ATAK** (the client). Operators see the full tactical picture — physical robots, virtual units, CBRN hazards, scoring zones — as standard MIL-STD-2525B symbols on a real map.

```
Plato Pod (Docker)
    │  CoT XML over TCP
    ▼
FreeTAKServer (Docker)
    │  Relays to all connected clients
    ├──► WinTAK (Windows native or via CrossOver/Wine on Linux/Mac)
    ├──► ATAK on Android tablet
    └──► iTAK on iOS
```

## Step 1 — Run FreeTAKServer

FreeTAKServer is the relay between Plato Pod and TAK clients. Run it in Docker:

```bash
docker run -d --name fts \
  --network host \
  -e IP=127.0.0.1 \
  -e APPIP=127.0.0.1 \
  vigsecdrone/freetakserver
```

The `IP` and `APPIP` env vars are critical — without them, the UI binds to an empty host and won't load.

Verify it's running:

```bash
sudo ss -tlnp | grep -E ":(5000|8087|19023)"
# Should show:
#   :5000  — FTS web UI
#   :8087  — CoT TCP input (where Plato Pod connects)
#   :19023 — FTS API
```

Open `http://localhost:5000/` in a browser. You should see the FTS dashboard. The `CONNECTED CLIENTS` counter shows how many TAK clients (and Plato Pod) are connected.

To make FTS auto-restart on boot:
```bash
docker update --restart unless-stopped fts
```

## Step 2 — Install a TAK client

### Option A: WinTAK on Linux via CrossOver (recommended for Linux users)

CrossOver handles Wine and dependencies automatically — easier than vanilla Wine.

1. Register at https://tak.gov (basic civilian registration, no special credentials needed)
2. Download **WinTAK-CIV** (Windows installer)
3. In CrossOver:
   - **Install Unlisted Application** → name `WinTAK`, select the installer EXE, choose **Windows 10 (64-bit)** bottle
   - In the new bottle, install **.NET Framework 4.8** (via "Install Windows Software" → "Install a runtime")
   - Install **d3dcompiler_47** (DirectX shader compiler)
   - Install Visual C++ runtime if listed (often already present)
4. Run the WinTAK installer
5. Launch WinTAK — first start takes ~60 seconds while it loads vehicle models and imagery cache
6. If you see an `OverflowException` dialog about `WindowChromeWorker._HandleNCHitTest`, just dismiss it — it's a harmless WPF/Wine bug

### Option B: ATAK on Android (recommended for production demos)

- Install **ATAK-CIV** from Google Play (free)
- Phone must be on the same network as your Plato Pod server

### Option C: WinTAK on native Windows

Just run the installer normally. Skips all the Wine setup.

## Step 3 — Connect the TAK client to FreeTAKServer

In WinTAK / ATAK:

1. **Settings** → **Network Preferences** → **Manage Server Connections** → **Add**
2. Configure:
   - **Description**: `FreeTAKServer`
   - **Address**: your machine's LAN IP (find with `hostname -I | awk '{print $1}'`). Use the actual LAN IP, not `127.0.0.1` — Wine and Android emulators don't route `localhost` to the host
   - **Port**: `8087`
   - **Protocol**: TCP / Streaming (not SSL)
3. Save and enable

Verify on the FTS dashboard — `CONNECTED CLIENTS` should increment.

## Step 4 — Add a high-resolution map source (WinTAK)

WinTAK ships with low-resolution NRL tiles. For demos, add Esri satellite imagery or standard OSM:

```bash
# Linux/CrossOver — file goes in the WinTAK bottle's Imagery directory
mkdir -p ~/.cxoffice/WinTAK/drive_c/ProgramData/WinTAK/Imagery
cat > ~/.cxoffice/WinTAK/drive_c/ProgramData/WinTAK/Imagery/Esri-Satellite.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<customMapSource>
    <name>Esri World Imagery (Satellite)</name>
    <minZoom>0</minZoom>
    <maxZoom>22</maxZoom>
    <tileType>image/jpeg</tileType>
    <url>https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{$z}/{$y}/{$x}</url>
    <tileUpdate>false</tileUpdate>
    <backgroundColor>#000000</backgroundColor>
</customMapSource>
EOF
```

Restart WinTAK, then in the **map source selector** (top toolbar / layers icon) choose **Esri World Imagery**.

For native Windows, drop the same XML into `C:\ProgramData\WinTAK\Imagery\`.

## Step 5 — Launch Plato Pod with TAK output

```bash
# Inside Docker
ros2 launch plato_pod atak_test.launch.py \
  exercise_file:=/ros2_ws/config/exercises/dowsett-field-ctf.yaml \
  target_host:=172.17.0.1 \
  target_port:=8087 \
  transport:=tcp \
  geo_origin_lat:=-35.293935 \
  geo_origin_lon:=149.166421 \
  scale_factor:=150 \
  gateway_port:=8090 \
  inbound_port:=4243
```

**Why these parameters:**
- `target_host:=172.17.0.1` — Docker host bridge IP (so Plato Pod inside Docker can reach FTS on the host). Use `127.0.0.1` if Plato Pod is on the host directly.
- `target_port:=8087` — FreeTAKServer's CoT TCP input
- `transport:=tcp` — FTS expects TCP, not UDP unicast
- `geo_origin_lat/lon` — south-west corner of the area you want to map. Use the SW corner so all robot positions fall inside the field.
- `scale_factor:=150` — desktop arena (0.84m) maps to ~126m of real terrain (covers Dowsett Field)
- `gateway_port:=8090` — avoids conflict with FreeTAKServer's port 8080
- `inbound_port:=4243` — UDP port for incoming ATAK waypoints

## Step 6 — Spawn robots and watch them on TAK

```bash
# From the host, spawn virtual robots
curl -X POST http://localhost:8090/robots/spawn -H 'Content-Type: application/json' -d '{"x":0.1,"y":0.36}'
curl -X POST http://localhost:8090/robots/spawn -H 'Content-Type: application/json' -d '{"x":0.3,"y":0.36}'
curl -X POST http://localhost:8090/robots/spawn -H 'Content-Type: application/json' -d '{"x":0.5,"y":0.36}'
curl -X POST http://localhost:8090/robots/spawn -H 'Content-Type: application/json' -d '{"x":0.7,"y":0.36}'
```

In WinTAK, navigate to **Canberra, Australia** (around -35.29, 149.17) and zoom to **Dowsett Field**. You should see 4 military symbols:

| Robot | Role | Symbol on TAK |
|-------|------|---------------|
| 1 | TANK BLUE-1 | Blue rectangle, tank icon |
| 2 | RCN BLUE-2 | Blue rectangle, recon icon |
| 3 | APC RED-3 | Red rectangle, APC icon |
| 4 | RCN RED-4 | Red rectangle, recon icon |

## Step 7 — Drive a robot

```bash
pip install websocket-client --break-system-packages

cat > /tmp/drive.py << 'EOF'
import websocket, json, time
ws = websocket.create_connection('ws://localhost:8090/api/control')
for _ in range(300):
    ws.send(json.dumps({'type':'cmd_vel','robot_id':1,'linear_x':0.02,'angular_z':0.0}))
    time.sleep(0.1)
ws.close()
EOF
python3 /tmp/drive.py
```

Watch TANK BLUE-1 cross Dowsett Field at ~3 m/s scaled (0.02 m/s × 150x). The command pipeline's boundary and collision filters automatically stop the robot at the arena edge or when it bumps into another robot.

## Step 8 (optional) — Run the standalone CoT viewer

If you don't want to install WinTAK or FreeTAKServer, the platform includes a lightweight browser-based CoT viewer for development:

```bash
python3 tools/cot_viewer.py
# Open http://localhost:8090 (use a different port if Plato Pod gateway is here)
```

It shows robots as real-world-sized circles on OpenStreetMap with vehicle-specific colours and heading lines. Not as polished as ATAK but useful for quick verification without TAK clients.

## Troubleshooting

| Problem | Fix |
|---------|-----|
| FTS UI shows `Invalid URL 'http://:19023/...'` | Container started without `IP` env var. Stop and recreate with `-e IP=127.0.0.1` |
| WinTAK silently exits after splash | Wait 60s — it's loading. Check log at `~/.cxoffice/WinTAK/drive_c/users/crossover/AppData/Local/WinTAK/Logs/` |
| WinTAK shows `OverflowException` repeatedly | Harmless WPF/Wine bug. Dismiss the dialog |
| WinTAK can't connect to FTS via 127.0.0.1 | Use the host's LAN IP instead — Wine doesn't always route loopback correctly |
| Map is low resolution | Add Esri or OSM map source (see Step 4), restart WinTAK |
| Robots not visible on map | Check `CONNECTED CLIENTS = 2` on FTS dashboard. If only 1, WinTAK isn't connected |
| Robots in the wrong location | Adjust `geo_origin_lat/lon` to the SW corner of your target area |
| Polygons (boundary, zones) not rendering | Different ATAK versions parse polygon CoT differently. Tested on ATAK-CIV; WinTAK polygon rendering varies |
| `address already in use` on port 4242 | A previous CoT bridge is still running. Restart the Docker container or use `inbound_port:=4243` |
| `address already in use` on port 8080 | FreeTAKServer's data package server uses 8080. Use `gateway_port:=8090` |
