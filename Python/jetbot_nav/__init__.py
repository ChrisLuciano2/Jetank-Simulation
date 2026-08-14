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
    gap_follow   — follow-the-gap obstacle avoidance from an N-ray scan
    target_seek  — gap_follow biased toward a colour-detected target

perception depends on numpy + opencv-python (see requirements.txt);
gap_follow and target_seek are standard library only, so their offline
regression suites run with no extra install.
"""

from . import gap_follow
from . import target_seek

__all__ = ["gap_follow", "target_seek", "perception"]


def __getattr__(name):
    # perception pulls in cv2/numpy. Import it lazily so that
    # `from jetbot_nav import gap_follow` — and the offline test suites —
    # still work on a bare Python install with neither package present.
    if name == "perception":
        from . import perception
        return perception
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
