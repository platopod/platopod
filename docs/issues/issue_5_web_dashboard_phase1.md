## Web Dashboard — Live Camera with AR Overlay (Phase 1)

Implement a browser-based dashboard that displays the live overhead camera feed with augmented reality overlays showing robot poses, arena boundaries, and system status. This is the primary visual interface for students and instructors. No software installation required — students open a URL in any modern browser.

### Purpose

The dashboard serves three audiences:

- **Students** see their robots (physical and virtual) moving on the arena in real-time, with visual feedback for boundaries and collisions.
- **Instructors** monitor the entire arena, all robots, and system health from a single view.
- **Demonstrations** provide an impressive live visualisation for ADFA leadership, grant reviewers, and conference presentations.

### Architecture

The dashboard is a static HTML/JavaScript application served by the Raspberry Pi. It connects to two data sources:

```
┌──────────────────────────────────┐
│         Web Browser              │
│  ┌────────────┐  ┌────────────┐  │
│  │ Camera Feed│  │  AR Canvas │  │
│  │  (video)   │  │  (overlay) │  │
│  └─────┬──────┘  └─────┬──────┘  │
└────────┼────────────────┼────────┘
         │                │
    MJPEG/WebRTC     WebSocket
         │                │
┌────────┴────────────────┴────────┐
│        ROS2 Server (Pi 5)        │
│  Camera node    API Gateway      │
└──────────────────────────────────┘
```

1. **Camera feed:** Raw video stream via MJPEG over HTTP or WebRTC. The browser renders this as the background layer.
2. **WebSocket (Issue #3):** Pose updates, events, and robot registry data. The browser renders overlays on a transparent canvas positioned on top of the video feed.

The dashboard is a **read-only client** of the existing APIs — it does not introduce any new server-side components. It subscribes to all robots for observation and renders what it receives.

### Camera-to-canvas calibration

The AR overlay must align with the physical arena in the camera feed. This requires a one-time calibration mapping between camera pixel coordinates and arena coordinates (metres):

1. The vision node already computes the arena boundary from corner AprilTags (Issue #1). These corner tags have known pixel positions (from camera) and known arena positions (from AprilTag pose estimation).
2. From these correspondences, compute a **homography matrix** mapping arena coordinates (metres) to camera pixel coordinates.
3. The server publishes this homography to a ROS2 topic. The API gateway makes it available via a REST endpoint:
   ```
   GET /arena/homography
   ```
4. The dashboard fetches the homography on load and uses it to project all overlay elements from arena coordinates onto the correct pixel positions in the video feed.

If corner tags are moved (dynamic arena reconfiguration, Issue #1), the homography is recomputed and pushed to the dashboard via WebSocket. The overlay adjusts in real-time.

### Overlay elements

All overlays are drawn on a transparent HTML5 canvas layered on top of the video feed:

**Arena boundary:**
- The convex hull boundary (Issue #1) drawn as a solid coloured line.
- Corner tag positions marked with small icons.
- Semi-transparent fill outside the boundary to visually indicate the out-of-bounds area.

**Static obstacles:**
- Obstacles from the arena model (Issue #1) rendered as filled polygons with a distinct colour (e.g. dark grey with border).
- Obstacle labels displayed on hover or as small text annotations.
- Circles rendered as circles, rectangles as rectangles, arbitrary polygons as polygons.

**Scoring zones:**
- Scoring zones from the arena model (Issue #1) rendered as semi-transparent coloured regions.
- Zone colour matches the team colour if team-specific, or neutral colour if open to all teams.
- Zone name displayed as a label.
- Active capture progress (e.g. hold timer) shown as an animated ring or progress bar around the zone.

**Physical robots:**
- Solid coloured circle at the robot's position, radius proportional to the robot's actual radius.
- Direction arrow indicating orientation (theta).
- Robot ID label above the circle.
- Colour indicates status: green = active, yellow = connected but tag not visible, grey = inactive.
- Team colour border when an exercise is active.

**Virtual robots:**
- Same visualisation as physical robots but with a dashed circle outline to distinguish from physical robots.
- Semi-transparent fill to convey that the robot is not physically present.
- Team colour border when an exercise is active.

**Game effects (Issue #7):**
- Frozen robots shown with a distinct visual (e.g. flashing outline, ice/freeze icon) and countdown timer.

**System status (HUD):**
- Number of active physical and virtual robots.
- Server connection status indicator.
- Camera feed FPS.
- Arena dimensions.
- Exercise name, state, and timer (when exercise is active).
- Team scores (when exercise with scoring is active).

### Camera feed delivery

The server streams the camera feed to the browser. Two options, in order of preference:

**Option A — MJPEG over HTTP (simplest):**
```
GET /camera/stream
```
Returns a multipart JPEG stream. The browser renders it in an `<img>` tag. Low latency (~100ms), works in all browsers, no JavaScript media API needed. Bandwidth: ~2–5 Mbps at 1080p 15fps.

**Option B — WebRTC (lower latency, more complex):**
Uses the browser's built-in WebRTC stack for hardware-accelerated H.264 decoding. Lower latency (~50ms) and lower bandwidth (~1–2 Mbps) but significantly more complex to implement (signalling server, STUN/TURN, ICE negotiation).

**Recommendation:** Start with MJPEG for Phase 1. It requires minimal server-side code (a few lines with OpenCV's `imencode` in a Flask/FastAPI endpoint) and works reliably. Upgrade to WebRTC in Phase 2 if latency or bandwidth becomes an issue.

### Responsive layout

The dashboard layout adapts to different screen sizes:

- **Desktop (instructor):** Full-screen arena view with status panel on the side.
- **Tablet/phone (student):** Arena view fills the screen. Status shown as a minimal overlay bar.

The camera feed and canvas overlay scale together, maintaining aspect ratio and correct alignment.

### Technology stack

- **Frontend:** Vanilla HTML5, CSS, JavaScript. No framework required for Phase 1. A single `index.html` file with inline CSS and JS.
- **Canvas:** HTML5 Canvas 2D API for overlay rendering.
- **WebSocket:** Native browser `WebSocket` API, connecting to the endpoint defined in Issue #3.
- **Serving:** Static files served by the same HTTP server that handles the REST API (e.g. FastAPI's `StaticFiles` middleware).

### Interaction (Phase 1 — minimal)

Phase 1 is primarily a visualisation tool. Minimal interaction:

- **Click on a robot** to see its details (ID, type, pose, velocity, status) in a tooltip or side panel.
- **Toggle overlays** on/off (boundary, robot labels, status HUD) via checkboxes.

Full interactive control (selecting a robot and controlling it from the dashboard, sensor visualisation, game overlays) is deferred to Phase 2 (Issue #8).

### URL and access

The dashboard is accessible at:
```
http://<server-ip>:8080/
```

No authentication for Phase 1. The dashboard connects as a read-only observer and does not acquire control of any robot.

### Example render

```
┌─────────────────────────────────────────────┐
│  ┌─ plato-arena ────────────────────────┐   │
│  │                                      │   │
│  │    ◁ 2                               │   │
│  │         ┈┈┈┈┈                        │   │
│  │         ┊ 4 ┊                        │   │
│  │         ┈┈┈┈┈                        │   │
│  │                          △ 1         │   │
│  │                                      │   │
│  │              ▷ 3                     │   │
│  │                                      │   │
│  └──────────────────────────────────────┘   │
│                                             │
│  Robots: 3 physical, 1 virtual  │ 30 FPS    │
│  Server: connected              │ 0.84×0.59m│
└─────────────────────────────────────────────┘

△ ▷ ▽ ◁ = physical robots (solid) with direction
┊ ┊     = virtual robot (dashed outline)
```

### Acceptance criteria

- [ ] Dashboard served as static HTML from the Pi, accessible via browser at `http://<server-ip>:8080/`
- [ ] Live camera feed displayed as background layer (MJPEG)
- [ ] Camera-to-canvas calibration via homography from arena corner tags
- [ ] Homography updates dynamically when arena is reconfigured
- [ ] Arena boundary polygon rendered as overlay
- [ ] Static obstacles rendered as filled polygons with labels
- [ ] Scoring zones rendered as semi-transparent coloured regions with labels
- [ ] Active capture progress visualised on scoring zones
- [ ] Physical robots rendered as solid coloured circles with direction arrow and ID label
- [ ] Virtual robots rendered as dashed circles, visually distinct from physical robots
- [ ] Robot colour reflects status (active/inactive/error) and team assignment
- [ ] Frozen robots shown with distinct visual indicator and countdown
- [ ] Pose overlays update in real-time via WebSocket subscription
- [ ] Arena model updates (obstacles/zones added/removed) reflected in real-time
- [ ] System status HUD showing robot count, connection status, FPS, arena size, exercise state, and scores
- [ ] Click-to-inspect robot details
- [ ] Toggle overlay visibility
- [ ] Responsive layout for desktop and mobile screens
- [ ] No server-side changes required beyond camera stream endpoint and arena model endpoint
- [ ] Works in Chrome, Firefox, Safari, and Edge
