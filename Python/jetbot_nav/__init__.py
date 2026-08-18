"""
jetbot_nav — student-facing navigation/perception helpers.

This package sits ON TOP OF the existing simulation shims (jetbot,
SCSCtrl, jetson_utils, pycuda, tensorrt). It contains no
simulation-only logic: every function here processes real data
(camera frames, proximity scans) with real algorithms, so it runs
identically on the Unity simulation and on the physical Jetson Nano.

Drop this folder next to your script (same place jetbot/ and
SCSCtrl/ already live) — no pip install needed, same as the rest
of the shim packages.

MODULES
    perception   — OpenCV colour-blob detection on a raw RGB frame
    visual_scan  — free-space distance scan derived from a camera frame
    heading      — relative yaw from the camera (visual gyro + course lock)
    gap_follow   — follow-the-gap obstacle avoidance from an N-ray scan
    target_seek  — gap_follow biased toward a colour-detected target

visual_scan and heading split the frame between them: visual_scan walks up
from the bottom for near-field DISTANCE, heading reads the band around the
horizon for far-field ROTATION. One camera, two non-overlapping jobs.

The sensing modules (perception, visual_scan, heading) need numpy
(+ opencv-python for perception and visual_scan); the control modules
(gap_follow, target_seek) are standard library only, so their offline
suites run with no extra install.

visual_scan is what feeds gap_follow on this robot: the JETANK has a
camera and no lidar, so the scan is computed from the image rather than
read from a range sensor. gap_follow itself is agnostic — it consumes a
distance array and does not care how the numbers were produced.
"""

from . import gap_follow
from . import target_seek

__all__ = ["gap_follow", "target_seek", "perception", "visual_scan", "heading"]

_LAZY = {"perception", "visual_scan", "heading"}


def __getattr__(name):
    # These pull in cv2/numpy. Import them lazily so that
    # `from jetbot_nav import gap_follow` — and the offline control-logic
    # suites — still work on a bare Python install with neither present.
    if name in _LAZY:
        import importlib
        return importlib.import_module("." + name, __name__)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
