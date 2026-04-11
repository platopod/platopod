## Web Dashboard — Full AR Visualisation (Phase 2)

Extend the Phase 1 dashboard (Issue #5) with sensor visualisation, inverse perspective mapping, interactive robot control, exercise game overlays, and projector output mode. Phase 2 transforms the dashboard from a passive viewer into the primary interface for students, instructors, and spectators.

### Dependencies

- Issue #5 — Phase 1 dashboard (camera feed, pose overlays, arena model rendering)
- Issue #6 — Sensor API (sensor data subscription)
- Issue #7 — Exercise management (teams, scoring, game effects)
- Issue #8 — Range sensors (Lidar, Sonar data for visualisation)
- Issue #9 — Position and identification sensors (GPS, FoF data for visualisation)

### Inverse Perspective Mapping (IPM)

The overhead camera rarely views the arena perfectly perpendicular — even a slight tilt introduces perspective distortion. IPM rectifies the camera image into a true top-down orthographic view where every pixel corresponds to a consistent real-world distance.

**Why IPM matters:**
- Without IPM, circular robots appear as ellipses at the far edge of the arena, obstacles look skewed, and the visual mismatch between rectified overlays and distorted video is jarring.
- With IPM, the video is rectified to match the arena coordinate system. Overlays and video align perfectly. Distances are visually consistent everywhere on screen.
- IPM simplifies dashboard rendering — overlays are drawn directly in arena coordinates (metres) with a simple linear scaling to pixels, no perspective math needed in the browser.

**Implementation:**

The homography matrix computed from the four corner AprilTags (Issue #1) is exactly the IPM transform. The camera node applies `cv2.warpPerspective()` before streaming:

```python
rectified = cv2.warpPerspective(frame, H, (output_width, output_height))
```

The server streams both raw and rectified feeds on separate endpoints:

```
GET /camera/stream          # raw camera feed (MJPEG)
GET /camera/stream/ipm      # IPM-rectified top-down feed (MJPEG)
```

When the arena boundary changes (corner tags moved), the homography is recomputed and the IPM transform updates automatically.

### Dashboard views

The dashboard offers three view modes, selectable via tabs or keyboard shortcut:

| View | Video layer | Overlay projection | Primary use |
|------|------------|-------------------|-------------|
| **Top-down (IPM)** | Rectified bird's-eye feed | Arena coordinates, linear scaling | Default view for students and instructors. Clean tactical display. |
| **Raw camera** | Original undistorted feed | Arena coordinates via homography | Debugging camera placement, checking physical robot state. |
| **Projector output** | No video, black background | Arena coordinates mapped to projector homography | AR projection onto physical arena surface (see projector section). |

The **top-down IPM view** is the default. All subsequent sections describe overlays as rendered in this view unless noted otherwise.

### Sensor visualisation

Students can see what their robot perceives, overlaid on the arena view. Each sensor type has a distinct visual representation. Sensor overlays are toggled per robot and per sensor type.

**Lidar (Issue #8):**
- Fan of thin rays emanating from the robot at each scan angle.
- Hit points rendered as small dots at the intersection distance.
- Rays colour-coded by distance: green (far, > 75% range_max) → yellow (mid) → red (close, < 25% range_max).
- Semi-transparent filled polygon connecting all hit points shows the "visible area."
- Toggle: rays only, hit points only, filled polygon, or all.

**Sonar (Issue #8):**
- Wide cone segments emanating from the robot, one per sonar segment.
- Each cone shaded by range: darker fill = closer obstacle.
- Cone width matches the `segment_angle_deg` configuration.
- Less detailed than Lidar visually, matching the sensor's lower resolution.

**GPS (Issue #9):**
- Uncertainty ellipse around the robot, sized proportional to `position_noise_stddev`.
- Shows students why their GPS-based controller wobbles — the true position is somewhere within the ellipse.
- When GPS is in dropout (no readings), the ellipse expands or a "GPS LOST" indicator appears.

**FoF (Issue #9):**
- Detection range shown as a semi-transparent circle (or arc for limited FOV) around the robot.
- Identification range shown as a smaller inner circle.
- Lines drawn from the detecting robot to each detected robot:
  - Blue solid line = friend (identified)
  - Red solid line = foe (identified)
  - Grey dashed line = unknown (detected but not identified)
- Detection lines fade when a robot exits detection range.

**Sensor view mode:**

A "sensor perspective" toggle restricts the dashboard to show only what a selected robot can perceive:

- Areas outside the Lidar/Sonar scan range are darkened (fog of war effect).
- Only detected robots (via FoF) are shown — undetected opponents are hidden.
- GPS position shown with uncertainty ellipse instead of true position.

This teaches students the difference between omniscient "God view" and realistic sensor-limited perception.

### Interactive control

Phase 2 adds robot control directly from the browser, using the same WebSocket API and shared control policy defined in Issue #3.

**Robot selection:**
- Click a robot on the arena view to select it. Selected robot shows a highlight ring.
- Robot details panel appears showing ID, type, team, pose, velocity, sensor readings.

**Keyboard control:**
- Arrow keys send `cmd_vel` using impulse-based steering (same as the Python `KeyboardController` in Issue #3).
- Key down starts sending commands at 10 Hz. Key up sends zero velocity.
- ↑ = forward, ↓ = backward, ← = turn left, → = turn right.
- ↑+← = forward-left arc, ↑+→ = forward-right arc.

**On-screen joystick (mobile/tablet):**
- Virtual joystick for touch devices.
- Left thumb: linear velocity (forward/backward). Right thumb: angular velocity.
- Or single joystick in arcade-drive mode.

**Control indicator:**
- When the dashboard has control of a robot, a "CONTROLLING" badge appears on the robot.
- If another client (Python SDK) has control, the robot shows "BUSY — controlled by Alice."
- Dashboard follows the shared control policy — first to send `cmd_vel` acquires control, control timeout releases after 30 seconds of inactivity.

### Exercise and game overlays

When an exercise is active (Issue #7), the dashboard displays game-specific overlays:

**Scoreboard:**
- Team scores displayed prominently at the top of the screen.
- Animated score change on update (e.g. "+1" floating animation).

**Timer:**
- Exercise countdown timer. Changes colour when < 30 seconds remain.
- Paused indicator when exercise is paused.

**Freeze indicators:**
- Frozen robots rendered with a distinct visual: pulsing blue overlay and a circular countdown timer showing seconds remaining.
- "FROZEN" label on the robot.

**Capture progress:**
- When a robot is holding a scoring zone, an animated ring around the zone fills up over the hold duration.
- Ring colour matches the capturing team's colour.
- If the robot leaves the zone, the ring resets.

**Event feed:**
- Scrolling event log at the bottom of the screen, styled like a game HUD.
- Examples: "Alice tagged Carol! (Robot 3 frozen for 5s)", "Blue team captured objective!", "Robot 2 disconnected."
- Events fade after 10 seconds.

**Team indicators:**
- Robot borders coloured by team.
- Team zones shaded with team colour.
- Team legend in the corner showing team names, colours, and member count.

### Projector output mode

The dashboard supports a projector mode designed for the UMBOLITE HY320 (or similar) projecting down onto the physical arena surface. This creates a true AR experience where virtual elements are physically visible on the arena.

**Projector calibration:**

The projector and camera must be calibrated to establish a projector homography:

1. The projector displays a known checkerboard pattern onto the arena surface.
2. The overhead camera detects the projected checkerboard using `cv2.findChessboardCorners()`.
3. The correspondence between projector pixels and camera pixels (and therefore arena coordinates) gives the projector homography matrix.
4. This matrix is stored and used to map arena coordinates to projector pixels.

```
POST /admin/calibrate/projector
```

Triggers the calibration sequence. The admin can re-run it whenever the projector or camera is moved.

**Projector rendering:**

In projector mode, the dashboard renders on a black background (no camera feed) using the projector homography:

- **Virtual robots** rendered as bright coloured circles — visible on the physical arena as moving shapes.
- **Arena boundary** projected as visible lines.
- **Obstacles** projected as solid shapes — students see virtual walls on the arena surface.
- **Scoring zones** projected as coloured regions.
- **Sensor visualisations** (Lidar rays, FoF lines) projected onto the arena surface around the physical robots.
- **Freeze effect** projected as a flashing ring around a frozen robot.

**Projector endpoint:**

The projector view is a separate URL:
```
http://<server-ip>:8080/projector
```

The browser on the projector-connected device opens this URL in full-screen mode. The Pi's HDMI output can also drive the projector directly using a lightweight renderer.

**Projector + camera co-registration:**

The projector and camera don't need to be co-located. The calibration homography handles arbitrary relative positioning. However, mounting them close together (e.g. in the integrated housing from the server hardware spec) minimises parallax for objects above the arena surface (like robot bodies).

### Replay

The server optionally records timestamped state during an exercise for post-exercise review:

**Recording:**
- Robot poses (all robots, at pose update rate)
- Sensor data (all active sensors, at sensor update rate)
- Events (boundary contacts, collisions, tags, scoring)
- Exercise state changes (start, pause, resume, end)

Stored as a compressed JSON-lines file:
```
/server/recordings/{exercise_id}_{timestamp}.jsonl.gz
```

**Playback:**
- The dashboard includes a timeline scrubber for replay mode.
- Play, pause, speed control (0.5×, 1×, 2×, 4×).
- All overlays render from recorded data instead of live WebSocket.
- Sensor visualisations replay exactly as they occurred.

**Replay endpoint:**
```
GET /recordings                              # list available recordings
GET /recordings/{filename}                   # download recording file
WS  /api/replay?file={filename}&speed=1.0    # stream replay data via WebSocket
```

The replay WebSocket sends the same message types as the live WebSocket (poses, events, sensor data) but from the recording file, time-shifted and speed-adjusted. The dashboard code doesn't distinguish between live and replay — same rendering logic, different data source.

### Spectator / broadcast view

An optimised view for projection onto a wall or screen (not the arena) for audiences:

```
http://<server-ip>:8080/spectator
```

- Full-screen layout: arena view on the left (70%), scoreboard and event feed on the right (30%).
- Large team scores with animated updates.
- Robot labels enlarged for readability from distance.
- Event feed with larger text and longer display duration.
- No interactive controls — observation only.
- Optional picture-in-picture: raw camera thumbnail in the corner showing the physical arena.

### Technology stack (Phase 2 additions)

- **Three.js or Pixi.js** — for hardware-accelerated 2D rendering of sensor visualisations (Canvas 2D may struggle with hundreds of Lidar rays at 10 Hz). Pixi.js is recommended for 2D.
- **Gamepad API** — browser API for USB/Bluetooth game controllers as an alternative to keyboard.
- **Fullscreen API** — for projector and spectator modes.
- **Web Workers** — offload sensor data processing from the main rendering thread.

### Acceptance criteria

**Inverse Perspective Mapping:**
- [ ] Server streams IPM-rectified camera feed on separate endpoint
- [ ] IPM homography updates automatically when arena boundary changes
- [ ] Dashboard defaults to IPM top-down view
- [ ] Raw camera view available as toggle
- [ ] Overlays align correctly with video in both IPM and raw views

**Sensor visualisation:**
- [ ] Lidar rendered as colour-coded ray fan with hit points and optional filled polygon
- [ ] Sonar rendered as shaded cone segments
- [ ] GPS uncertainty ellipse displayed, expands on dropout
- [ ] FoF detection/identification ranges shown, detection lines coloured by classification
- [ ] Sensor overlays toggled per robot and per sensor type
- [ ] "Sensor perspective" mode shows only what the selected robot perceives (fog of war)

**Interactive control:**
- [ ] Click-to-select robot on arena view
- [ ] Keyboard arrow keys send cmd_vel via WebSocket (impulse-based steering)
- [ ] On-screen virtual joystick for touch devices
- [ ] Control follows shared control policy (Issue #3)
- [ ] Control status indicator on each robot (controlling / busy / available)

**Exercise and game overlays:**
- [ ] Scoreboard with animated score updates
- [ ] Countdown timer with colour change in final 30 seconds
- [ ] Freeze indicator with visual effect and countdown on frozen robots
- [ ] Capture progress ring on scoring zones
- [ ] Scrolling event feed (tag, capture, disconnect events)
- [ ] Team colour indicators on robots and zones

**Projector output:**
- [ ] Projector calibration via checkerboard detection
- [ ] Projector mode renders on black background with projector homography
- [ ] Virtual robots, obstacles, zones, and sensor rays projected onto arena surface
- [ ] Projector view accessible at `/projector` URL
- [ ] Projector and camera calibration independent (arbitrary relative positioning)

**Replay:**
- [ ] Exercise state recorded to compressed JSON-lines file
- [ ] Timeline scrubber with play, pause, and speed control
- [ ] Replay streams same message types as live WebSocket
- [ ] Sensor visualisations replay correctly
- [ ] Recording list and download via REST endpoints

**Spectator view:**
- [ ] Full-screen spectator layout at `/spectator` URL
- [ ] Large scoreboard, event feed, and robot labels for audience readability
- [ ] Optional picture-in-picture raw camera thumbnail
- [ ] Observation only, no interactive controls
