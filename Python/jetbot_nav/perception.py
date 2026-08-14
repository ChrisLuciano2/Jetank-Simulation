"""
jetbot_nav.perception — color-based object detection from camera frames.

Works on a real RGB numpy frame (the same array shape whether it came
from jetson_utils.cudaToNumpy() in simulation or from the real Jetson
camera). No Unity-specific data is used here — this is plain OpenCV
color thresholding, which is why it runs unmodified on hardware.

Typical usage:

    import jetson_utils
    from jetbot_nav import perception

    camera = jetson_utils.videoSource("csi://0")
    img    = camera.Capture()
    frame  = jetson_utils.cudaToNumpy(img)      # (H, W, 3) RGB uint8

    blob = perception.largest_blob(frame, "red")
    if blob:
        print(f"Red block at center ({blob['cx']}, {blob['cy']})")
"""

import cv2
import numpy as np

# ─── HSV color ranges ────────────────────────────────────────────────────────
# OpenCV hue range is 0-179. Red wraps around 0, so it needs two ranges.
# These are reasonable starting points for solid-colored props under
# neutral lighting — tune them for your scene if detection looks off
# (see the tuning note at the bottom of this file).

COLOR_RANGES = {
    "red": [
        ((0,   90, 60), (10,  255, 255)),
        ((170, 90, 60), (179, 255, 255)),
    ],
    "blue":   [((90,  90, 60), (130, 255, 255))],
    "green":  [((40,  70, 60), (85,  255, 255))],
    "yellow": [((20,  90, 60), (35,  255, 255))],
}


def _mask_for_color(hsv: np.ndarray, color: str) -> np.ndarray:
    if color not in COLOR_RANGES:
        raise ValueError(
            f"Unknown color '{color}'. Available: {list(COLOR_RANGES)}"
        )

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in COLOR_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
    return mask


def detect_color_blobs(frame_rgb: np.ndarray, color: str, min_area: int = 500):
    """
    Find all blobs of the given color in an RGB frame.

    Args:
        frame_rgb: (H, W, 3) uint8 RGB array (e.g. from jetson_utils.cudaToNumpy)
        color: one of "red", "blue", "green", "yellow"
        min_area: minimum blob area in pixels to filter out noise

    Returns:
        List of dicts, each: {x1, y1, x2, y2, cx, cy, area}, largest first.
    """
    bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = _mask_for_color(hsv, color)

    # Clean up noise: small holes/specks confuse contour detection
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        blobs.append({
            "x1": float(x), "y1": float(y),
            "x2": float(x + w), "y2": float(y + h),
            "cx": float(x + w / 2), "cy": float(y + h / 2),
            "area": float(area),
        })

    blobs.sort(key=lambda b: b["area"], reverse=True)
    return blobs


def largest_blob(frame_rgb: np.ndarray, color: str, min_area: int = 500):
    """Convenience wrapper: returns the single largest blob, or None."""
    blobs = detect_color_blobs(frame_rgb, color, min_area)
    return blobs[0] if blobs else None


# ─── Tuning note ─────────────────────────────────────────────────────────────
# If detection misses objects or picks up the wrong things:
#   1. Run test_perception.py and check the printed HSV value at the
#      object's pixel center (add a debug print if needed) against the
#      ranges above.
#   2. Scene lighting matters: very bright or very dark lighting shifts
#      saturation/value readings. Prefer flat, neutral scene lighting
#      over strong directional lights for consistent detection.
#   3. Same tuning applies on the physical Jetson — lighting conditions
#      in the room will matter there too, which is realistic behavior
#      for this kind of vision pipeline (not a simulation quirk).
