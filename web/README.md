# Web

## Dashboard

Browser-based tactical display — `web/static/index.html`

Served by the API gateway at `http://<server-ip>:8080/`. Two rendering modes:

**Map mode (default):** 2D top-down arena with obstacles, zones, and robots. No camera needed. Works immediately from exercise YAML.

**Camera mode (toggle):** Live MJPEG camera feed as background with pixel-based AR overlay for detected AprilTags. Toggle via "Camera overlay" checkbox. Requires vision node with camera.

Shows: arena boundary, obstacles, scoring zones, robot positions with direction arrows (physical = solid, virtual = dashed), real-time pose updates via WebSocket, HUD with robot count and connection status.

Single HTML file, vanilla JS, no build tools.

## Python SDK

Client library for controlling robots — `web/sdk/platopod/`

```bash
cd web/sdk && pip install -e .
```

See `web/sdk/examples/keyboard_demo.py` for a keyboard control example.
