## Vision Node: Camera Capture, AprilTag Detection, and Pose Estimation

Implement the vision node — the foundational component that makes the physical world visible to the server. This node captures video from the overhead camera, detects AprilTag markers, estimates their poses in metres, and publishes the results to ROS2 topics. Every other issue depends on this node: the arena boundary (Issue #1) needs tag poses, robot discovery (Issue #2) needs visible tag lists, and the dashboard (Issue #5) needs the camera stream.

### Role in the architecture

```
Physical Arena
    │
    ▼
┌──────────────┐
│   Overhead   │
│   Camera     │
│  (OV2710 /   │
│  Logitech)   │
└──────┬───────┘
       │ USB (V4L2)
       ▼
┌──────────────────────────────────────────┐
│           Vision Node                     │
│                                           │
│  Camera Capture ──► AprilTag Detector     │
│       │                    │              │
│       │              Pose Estimator       │
│       │              (camera intrinsics   │
│       │               + tag size → m)     │
│       │                    │              │
│       ▼                    ▼              │
│  MJPEG Stream      Tag Pose Publisher     │
│  /camera/stream    /tags/detections       │
│                    /tags/poses             │
└──────────────────────────────────────────┘
       │                    │
       ▼                    ▼
   Dashboard           Arena Model (Issue #1)
   (Issue #5)          Registry (Issue #2)
```

### Camera capture

The node captures frames from the overhead USB camera using OpenCV's `VideoCapture`:

```python
cap = cv2.VideoCapture(0)  # /dev/video0
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)
```

**Supported cameras:**
- OV2710 USB module (1080p, 100° FOV, no-distortion lens) — production
- Logitech C920/C922 (1080p, rolling shutter) — development
- Any UVC-compatible USB camera

The node auto-detects the camera on startup. If no camera is found, it logs an error and publishes a "no camera" status, allowing virtual-only development without hardware.

### Camera calibration

Before AprilTag pose estimation works accurately, the camera's intrinsic parameters must be calibrated. This is a one-time procedure per camera.

**Calibration procedure:**

1. Print a checkerboard pattern (9×6 inner corners, 25mm square size) on A4 paper.
2. Run the calibration tool:
   ```bash
   ros2 run plato_pod camera_calibrate --squares-x 9 --squares-y 6 --square-size 0.025
   ```
3. Hold the checkerboard in front of the camera at various angles and distances. The tool captures frames automatically when it detects the pattern.
4. After ~20 frames, the tool computes intrinsic parameters and saves them:
   ```
   server/config/camera_calibration.yaml
   ```

**Calibration file format:**

```yaml
camera_calibration:
  image_width: 1920
  image_height: 1080
  camera_matrix:
    - [fx, 0, cx]
    - [0, fy, cy]
    - [0, 0, 1]
  distortion_coefficients: [k1, k2, p1, p2, k5]
  calibration_date: "2026-04-12"
  reprojection_error: 0.25  # pixels, lower is better
```

If no calibration file exists, the node uses a default pinhole model estimated from the camera's resolution and stated FOV. This provides approximate poses but may have errors of several centimetres at the edges of the frame.

### AprilTag detection

The node uses the `pupil-apriltags` Python library (or `dt-apriltags`) to detect AprilTag markers in each frame:

```python
from pupil_apriltags import Detector

detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=2.0,    # downsample for speed (2.0 = half resolution)
    quad_sigma=0.0,       # no blur
    decode_sharpening=0.25,
    refine_edges=True
)
```

**Tag family:** `tag36h11` — 587 unique tags, robust to partial occlusion.

**Tag ID ranges (convention):**

| ID range | Purpose |
|----------|---------|
| 0–15 | Robot tags |
| 100–103 | Arena boundary corner tags |
| 101 | Arena origin (coordinate system reference) |
| 200+ | Reserved for future use (obstacles, waypoints) |

These ranges are configurable in `server/config/apriltag_settings.yaml`.

### AprilTag settings configuration

```yaml
apriltag_settings:
  family: "tag36h11"
  
  # Physical tag sizes (outer dimension including white border) in metres
  tag_sizes:
    default: 0.050          # 50mm — standard Plato Pod tag
    boundary: 0.070         # 70mm — larger for reliable corner detection
  
  # Tag ID ranges
  tag_ranges:
    robots: [0, 15]         # IDs 0–15 are robot tags
    boundary: [100, 103]    # IDs 100–103 are arena corners
    origin: 101             # coordinate system origin
  
  # Detector parameters
  detector:
    nthreads: 4
    quad_decimate: 2.0      # 1.0 = full resolution (slower), 2.0 = half (faster)
    refine_edges: true
    decode_sharpening: 0.25
  
  # Performance
  detection_fps: 30         # target detection rate
  pose_estimation: true     # enable 3D pose estimation (requires calibration)
```

### Pose estimation

For each detected tag, the node estimates its 6-DOF pose (position + orientation) relative to the camera using the camera intrinsics and known tag size:

```python
for detection in detections:
    # Determine tag size based on ID range
    if detection.tag_id in range(100, 104):
        tag_size = config["tag_sizes"]["boundary"]
    else:
        tag_size = config["tag_sizes"]["default"]
    
    # Estimate pose
    pose = detector.estimate_tag_pose(
        detection,
        camera_params=(fx, fy, cx, cy),
        tag_size=tag_size
    )
    # pose.t = [x, y, z] translation in metres from camera
    # pose.R = 3x3 rotation matrix
```

### Coordinate transform: camera frame → arena frame

The raw pose is in the camera's coordinate frame (origin at camera, Z forward). The arena model (Issue #1) needs poses in the arena frame (origin at Tag 101, XY plane on the arena surface):

**Step 1:** Detect Tag 101 (the origin tag) and compute the camera-to-arena transform.

**Step 2:** Transform all other tag poses from camera frame to arena frame using this transform.

```python
# T_camera_to_arena is computed from Tag 101's detected pose
# For each tag:
arena_pose = T_camera_to_arena @ camera_pose

# Extract 2D pose (x, y, theta) from the 3D transform
x = arena_pose[0]  # metres from Tag 101
y = arena_pose[1]  # metres from Tag 101
theta = atan2(R[1,0], R[0,0])  # heading angle from rotation matrix
```

If Tag 101 is not visible, the node uses the last known transform and logs a warning. The arena boundary (Issue #1) handles staleness logic for corner tags.

### Homography computation

In addition to per-tag poses, the node computes a **homography matrix** mapping between camera pixel coordinates and arena coordinates (metres). This is used by:

- **Issue #1** — arena model validation
- **Issue #5** — dashboard AR overlay alignment
- **Issue #10** — IPM rectification

The homography is computed from the four boundary corner tags (IDs 100–103) whenever all four are visible:

```python
# Corner tag pixel positions (from detection)
pixel_points = np.array([[det.center for det in corner_detections]])

# Corner tag arena positions (from pose estimation, in metres)
arena_points = np.array([[pose.x, pose.y] for pose in corner_poses])

# Compute homography
H, _ = cv2.findHomography(pixel_points, arena_points)
```

The homography is published to a ROS2 topic and served via REST endpoint for the dashboard.

### MJPEG camera stream

The node serves the raw camera feed as an MJPEG stream over HTTP for the dashboard (Issue #5):

```
GET /camera/stream          # raw camera feed
GET /camera/stream/ipm      # IPM-rectified feed (Issue #10, uses homography)
```

Implementation uses FastAPI with a streaming response:

```python
@app.get("/camera/stream")
async def camera_stream():
    async def generate():
        while True:
            ret, frame = cap.read()
            if ret:
                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")
```

### Debug visualisation

When running in debug mode, the node can overlay detection results on the camera feed:

- Tag outlines drawn as green quadrilaterals
- Tag IDs labelled at tag centres
- Coordinate axes drawn on each tag (RGB = XYZ)
- Arena boundary polygon drawn when corner tags are detected
- FPS counter

Debug mode is enabled via parameter:

```bash
ros2 run plato_pod vision_node --ros-args -p debug_overlay:=true
```

The debug-annotated feed is available at:

```
GET /camera/stream/debug
```

### ROS2 integration

**Published topics:**

```
/camera/image_raw          # sensor_msgs/Image — raw camera frames (for RViz2)
/tags/detections           # custom message — list of detected tags with pixel coordinates
/tags/poses                # custom message — list of detected tags with arena poses (x, y, theta)
/arena/homography          # custom message — 3x3 homography matrix (camera pixels → arena metres)
/camera/info               # sensor_msgs/CameraInfo — camera intrinsic parameters
```

**Published tag pose message format:**

```python
# Custom message: TagPose
tag_id: int
x: float          # metres from Tag 101
y: float          # metres from Tag 101
theta: float      # heading in radians [-π, π]
confidence: float # detection quality (0–1, based on decision margin)
timestamp: float  # detection timestamp
```

**Published detections message format:**

```python
# Custom message: TagDetections
tags: list[TagPose]           # all detected tags this frame
boundary_tags: list[TagPose]  # only boundary tags (IDs 100–103)
robot_tags: list[TagPose]     # only robot tags (IDs 0–15)
origin_visible: bool          # is Tag 101 currently detected?
frame_id: int                 # sequential frame counter
fps: float                    # current detection rate
```

**Parameters (configurable via ROS2 parameter server):**

```yaml
vision_node:
  ros__parameters:
    camera_device: 0
    camera_width: 1920
    camera_height: 1080
    camera_fps: 30
    calibration_file: "config/camera_calibration.yaml"
    apriltag_config: "config/apriltag_settings.yaml"
    debug_overlay: false
    mjpeg_quality: 80
    mjpeg_port: 8081
```

### REST endpoints

```
GET /camera/stream              # MJPEG raw feed
GET /camera/stream/debug        # MJPEG with detection overlay
GET /camera/stream/ipm          # MJPEG IPM-rectified (requires homography)
GET /camera/info                # camera intrinsics as JSON
GET /camera/calibrate/start     # start calibration capture
GET /camera/calibrate/status    # calibration progress
POST /camera/calibrate/compute  # compute and save calibration
GET /arena/homography           # current homography matrix as JSON
```

### Performance targets

| Metric | Target | Notes |
|--------|--------|-------|
| Detection FPS | ≥15 fps | At 1080p with `quad_decimate=2.0` |
| Detection latency | <50ms | From frame capture to pose published |
| MJPEG stream | ≥15 fps | Separate from detection rate |
| Max simultaneous tags | 20 | 4 boundary + 16 robots |
| Pose accuracy | ±5mm | At 1.5m camera height with calibrated camera |

On the Raspberry Pi 5 (Cortex-A76 @ 2.4GHz), the AprilTag library achieves ~20ms per frame at 1080p with `quad_decimate=2.0`. This leaves headroom for pose estimation and publishing.

### Virtual-only mode

If no camera is available (e.g. development on a laptop without hardware), the node starts in **virtual-only mode**:

- No camera capture or MJPEG stream
- No tag detection
- Publishes empty detections
- Arena model (Issue #1) can still function using boundary coordinates from the exercise config (Issue #7) instead of camera-detected tags
- Virtual robots (Issue #4) work normally — they don't need the camera

This allows the team to develop and test Issues #3–#10 without physical hardware.

### Dependencies

- **OpenCV** (`opencv-python-headless`) — camera capture, image processing, homography
- **pupil-apriltags** or **dt-apriltags** — AprilTag detection and pose estimation
- **NumPy** — matrix operations, coordinate transforms
- **FastAPI** — MJPEG streaming endpoints

Install:
```bash
pip3 install --break-system-packages pupil-apriltags
```

### Testing

**Unit tests (no camera needed):**
- Pose transform: given a known camera pose and tag pose, verify arena coordinate output
- Homography: given 4 known pixel-arena correspondences, verify computed matrix
- Tag range classification: verify correct categorisation of IDs into robot/boundary/origin
- Configuration loading: verify YAML parsing of apriltag_settings and camera_calibration

**Integration tests (camera needed):**
- Print a tag, hold it in front of the camera, verify detection and ID
- Print 4 boundary tags, arrange on desk, verify homography computation
- Verify MJPEG stream accessible in browser at `http://localhost:8081/camera/stream`
- Verify ROS2 topic publishing with `ros2 topic echo /tags/poses`

**Simulated camera (for CI):**
- Use a pre-recorded video file or synthetic images with rendered tags
- Set `camera_device` parameter to a video file path instead of device number
- Enables automated testing without physical hardware

### Acceptance criteria

- [ ] Camera capture from USB camera via OpenCV with configurable resolution and FPS
- [ ] AprilTag detection using tag36h11 family with configurable detector parameters
- [ ] Tag ID ranges loaded from `apriltag_settings.yaml`
- [ ] Camera calibration tool: checkerboard capture, intrinsic computation, save to YAML
- [ ] Pose estimation in metres using camera intrinsics and known tag sizes
- [ ] Coordinate transform from camera frame to arena frame using Tag 101 as origin
- [ ] Tag 101 not visible: use last known transform with warning
- [ ] Homography computed from boundary corner tags (IDs 100–103)
- [ ] Homography published to ROS2 topic and served via REST endpoint
- [ ] Tag detections published to ROS2 topics with arena poses (x, y, theta)
- [ ] Detections message includes separate lists for boundary tags and robot tags
- [ ] MJPEG raw camera stream served via HTTP
- [ ] MJPEG debug stream with detection overlay (tag outlines, IDs, axes)
- [ ] Virtual-only mode when no camera is present
- [ ] Simulated camera input from video file for automated testing
- [ ] Detection FPS ≥15 at 1080p on Raspberry Pi 5
- [ ] Pose accuracy ±5mm at 1.5m camera height with calibrated camera
- [ ] All settings configurable via ROS2 parameters and YAML config files
