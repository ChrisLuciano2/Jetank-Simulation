"""
jetbot_nav — student-facing navigation/perception helpers.

This package sits ON TOP OF the existing simulation shims (jetbot,
SCSCtrl, jetson_utils, pycuda, tensorrt). It contains no
simulation-only logic: every function here processes real data
(camera frames, joint angles) with real algorithms, so it runs
identically on the Unity simulation and on the physical Jetson Nano.

Drop this folder next to your script (same place jetbot/ and
SCSCtrl/ already live) — no pip install needed, same as the rest
of the shim packages.
"""

from . import perception

__all__ = ["perception"]
