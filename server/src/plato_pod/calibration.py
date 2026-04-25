"""Camera calibration tool for the Plato Pod vision pipeline.

Captures checkerboard images from the camera, computes intrinsic parameters,
and saves the calibration to a YAML file. Can be used standalone or driven
by REST endpoints.

No ROS2 dependency — the calibration math is pure OpenCV/numpy.
"""

from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path

import numpy as np
import yaml

logger = logging.getLogger(__name__)

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore[assignment]
    _HAS_CV2 = False


@dataclass
class CalibrationState:
    """Mutable state for an ongoing calibration session."""
    squares_x: int = 9
    squares_y: int = 6
    square_size: float = 0.025  # metres
    target_frames: int = 20

    # Collected data
    object_points: list[np.ndarray] = field(default_factory=list)
    image_points: list[np.ndarray] = field(default_factory=list)
    image_size: tuple[int, int] | None = None

    @property
    def frames_captured(self) -> int:
        return len(self.image_points)

    @property
    def is_complete(self) -> bool:
        return self.frames_captured >= self.target_frames


def generate_object_points(squares_x: int, squares_y: int,
                           square_size: float) -> np.ndarray:
    """Generate 3D object points for a checkerboard pattern.

    Args:
        squares_x: Number of inner corners along X.
        squares_y: Number of inner corners along Y.
        square_size: Size of each square in metres.

    Returns:
        Nx3 array of 3D points (Z=0 plane).
    """
    objp = np.zeros((squares_x * squares_y, 3), dtype=np.float32)
    indices = np.mgrid[0:squares_x, 0:squares_y].T.reshape(-1, 2)
    objp[:, :2] = indices * square_size
    return objp


def find_checkerboard(
    frame_gray: np.ndarray,
    squares_x: int,
    squares_y: int,
) -> np.ndarray | None:
    """Find checkerboard corners in a grayscale frame.

    Args:
        frame_gray: Grayscale uint8 image.
        squares_x: Number of inner corners along X.
        squares_y: Number of inner corners along Y.

    Returns:
        Nx1x2 array of sub-pixel corner positions, or None if not found.
    """
    if not _HAS_CV2:
        return None

    ret, corners = cv2.findChessboardCorners(
        frame_gray, (squares_x, squares_y),
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
    )

    if not ret:
        return None

    # Sub-pixel refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(frame_gray, corners, (11, 11), (-1, -1), criteria)

    return corners


def add_frame(state: CalibrationState, corners: np.ndarray,
              image_size: tuple[int, int]) -> None:
    """Add a detected checkerboard frame to the calibration state.

    Args:
        state: Mutable calibration state.
        corners: Detected corner positions from find_checkerboard.
        image_size: (width, height) of the image.
    """
    objp = generate_object_points(state.squares_x, state.squares_y, state.square_size)
    state.object_points.append(objp)
    state.image_points.append(corners)
    state.image_size = image_size
    logger.info(
        "Calibration frame %d/%d captured",
        state.frames_captured, state.target_frames,
    )


def compute_calibration(
    state: CalibrationState,
) -> tuple[np.ndarray, np.ndarray, float] | None:
    """Compute camera intrinsics from collected checkerboard frames.

    Args:
        state: Calibration state with collected frames.

    Returns:
        Tuple of (camera_matrix, distortion_coefficients, reprojection_error),
        or None if not enough frames or OpenCV unavailable.
    """
    if not _HAS_CV2:
        logger.error("OpenCV required for calibration computation")
        return None

    if state.frames_captured < 5:
        logger.error("Need at least 5 frames, got %d", state.frames_captured)
        return None

    if state.image_size is None:
        logger.error("No image size recorded")
        return None

    w, h = state.image_size
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        state.object_points, state.image_points, (w, h), None, None
    )

    logger.info("Calibration complete. Reprojection error: %.4f px", ret)
    return camera_matrix, dist_coeffs.flatten(), ret


def save_calibration(
    path: str | Path,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    image_width: int,
    image_height: int,
    reprojection_error: float,
) -> None:
    """Save camera calibration to a YAML file.

    Args:
        path: Output file path.
        camera_matrix: 3x3 camera intrinsic matrix.
        distortion: Distortion coefficients.
        image_width: Image width in pixels.
        image_height: Image height in pixels.
        reprojection_error: RMS reprojection error in pixels.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    data = {
        "camera_calibration": {
            "image_width": image_width,
            "image_height": image_height,
            "camera_matrix": camera_matrix.tolist(),
            "distortion_coefficients": distortion.tolist(),
            "calibration_date": date.today().isoformat(),
            "reprojection_error": round(float(reprojection_error), 4),
        }
    }

    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    logger.info("Calibration saved to %s", path)


def main(args=None) -> None:
    """CLI entry point: ros2 run plato_pod camera_calibrate."""
    if not _HAS_CV2:
        print("Error: OpenCV is required for camera calibration")
        return

    parser = argparse.ArgumentParser(description="Plato Pod Camera Calibration")
    parser.add_argument("--squares-x", type=int, default=9,
                        help="Checkerboard inner corners X (default: 9)")
    parser.add_argument("--squares-y", type=int, default=6,
                        help="Checkerboard inner corners Y (default: 6)")
    parser.add_argument("--square-size", type=float, default=0.025,
                        help="Square size in metres (default: 0.025)")
    parser.add_argument("--frames", type=int, default=20,
                        help="Number of frames to capture (default: 20)")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index (default: 0)")
    parser.add_argument("--output", type=str,
                        default="config/camera_calibration.yaml",
                        help="Output file path")

    parsed = parser.parse_args(args)

    state = CalibrationState(
        squares_x=parsed.squares_x,
        squares_y=parsed.squares_y,
        square_size=parsed.square_size,
        target_frames=parsed.frames,
    )

    cap = cv2.VideoCapture(parsed.camera)
    if not cap.isOpened():
        print(f"Error: Could not open camera {parsed.camera}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    print(f"Calibration: {parsed.squares_x}x{parsed.squares_y} checkerboard, "
          f"{parsed.square_size*1000:.0f}mm squares")
    print(f"Capture {parsed.frames} frames. Press 'q' to abort, 'c' to force capture.")
    print("Frames are captured automatically when the checkerboard is detected.\n")

    last_capture_time = 0.0

    while not state.is_complete:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = find_checkerboard(gray, state.squares_x, state.squares_y)

        display = frame.copy()
        if corners is not None:
            cv2.drawChessboardCorners(
                display, (state.squares_x, state.squares_y), corners, True
            )

            # Auto-capture with 2-second cooldown
            now = time.monotonic()
            if now - last_capture_time > 2.0:
                h, w = frame.shape[:2]
                add_frame(state, corners, (w, h))
                last_capture_time = now
                print(f"  Frame {state.frames_captured}/{state.target_frames} captured")

        status = f"Frames: {state.frames_captured}/{state.target_frames}"
        cv2.putText(display, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Calibration", display)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            print("Calibration aborted.")
            cap.release()
            cv2.destroyAllWindows()
            return

    cap.release()
    cv2.destroyAllWindows()

    print("\nComputing calibration...")
    result = compute_calibration(state)
    if result is None:
        print("Calibration failed.")
        return

    camera_matrix, distortion, error = result
    print(f"Reprojection error: {error:.4f} pixels")
    print(f"Camera matrix:\n{camera_matrix}")

    save_calibration(
        parsed.output,
        camera_matrix, distortion,
        state.image_size[0], state.image_size[1],
        error,
    )
    print(f"\nCalibration saved to {parsed.output}")


if __name__ == "__main__":
    main()
