"""
test_visual_scan.py — offline tests for jetbot_nav.visual_scan.

No Unity, no camera: the projection tests check the geometry against
distances derived independently by hand, and the scan tests run on
synthetic frames built in numpy.

The geometry half matters most. A segmentation bug is loud (the mask
looks obviously wrong the moment you render it), but a projection bug is
silent — it returns plausible numbers that are uniformly wrong, and
gap_follow will happily drive on them. So the checks below pin distances
against closed-form values computed from the setup, not against whatever
the code currently happens to return.

    py -3.8 test_visual_scan.py
"""

import math
import sys

import numpy as np

from jetbot_nav.visual_scan import (
    CameraGeometry, IMX219_STANDARD, floor_mask, free_space_scan, annotate,
)

_results = []


def check(label, ok, detail=""):
    _results.append(bool(ok))
    print(f"{'PASS' if ok else 'FAIL'}  {label}" + (f"  {detail}" if detail and not ok else ""))


def close(a, b, tol=0.05):
    return a is not None and b is not None and abs(a - b) <= tol


# ─── 1. Projection geometry, against hand-computed values ───────────────────
# Camera 0.5 m up, pitched 30 deg down, looking at a flat floor.
#
# The optical axis leaves the lens 30 deg below horizontal, so it meets the
# floor at horizontal distance  h / tan(30 deg) = 0.5 / 0.57735 = 0.8660 m.
# That ray is the image centre, so the centre pixel must report exactly
# that, at bearing 0.

geom = CameraGeometry(height_m=0.5, tilt_deg=30.0,
                      hfov_deg=62.2, vfov_deg=48.8, width=640, height=480)

centre = geom.pixel_to_ground(geom.cx, geom.cy)
check("centre pixel projects to h/tan(tilt) = 0.866 m",
      centre is not None and close(centre[1], 0.5 / math.tan(math.radians(30)), 0.01),
      f"got {centre}")
check("centre pixel has zero bearing",
      centre is not None and close(centre[0], 0.0, 1e-6), f"got {centre}")

# Bottom-centre of the frame looks further DOWN, so it must be NEARER than
# the centre. Depression angle there = tilt + vfov/2 = 30 + 24.4 = 54.4 deg,
# giving 0.5 / tan(54.4 deg) = 0.3573 m.
bottom = geom.pixel_to_ground(geom.cx, geom.height - 1)
expected_bottom = 0.5 / math.tan(math.radians(30 + 48.8 / 2))
check("bottom-centre pixel is nearer, at h/tan(tilt + vfov/2)",
      bottom is not None and close(bottom[1], expected_bottom, 0.02),
      f"got {bottom}, expected ~{expected_bottom:.3f}")

# Top-centre looks UP: depression = 30 - 24.4 = 5.6 deg, still below the
# horizon, so it hits the floor but far away: 0.5 / tan(5.6) = 5.10 m.
top = geom.pixel_to_ground(geom.cx, 0)
expected_top = 0.5 / math.tan(math.radians(30 - 48.8 / 2))
check("top-centre pixel is far but still on the floor",
      top is not None and close(top[1], expected_top, 0.15),
      f"got {top}, expected ~{expected_top:.3f}")

check("distance increases monotonically from bottom of frame to top",
      bottom[1] < centre[1] < top[1],
      f"{bottom[1]:.2f} / {centre[1]:.2f} / {top[1]:.2f}")

# ─── 2. Above the horizon returns None, not a distance ──────────────────────
# Tilt 5 deg down with a 48.8 deg vertical FOV puts the horizon inside the
# frame; rows above it must report "no information" rather than a number.

shallow = CameraGeometry(height_m=0.5, tilt_deg=5.0, width=640, height=480)
check("pixel above the horizon returns None (not a fake distance)",
      shallow.pixel_to_ground(shallow.cx, 0) is None)
check("pixel below the horizon still projects",
      shallow.pixel_to_ground(shallow.cx, shallow.height - 1) is not None)
check("horizon_row lands inside the frame for a shallow tilt",
      0 < shallow.horizon_row() < shallow.height,
      f"got {shallow.horizon_row():.1f}")
# A robot camera tilted well down sees floor to the top of the frame, so
# its horizon is off-image. That must come back as a negative row (clamped
# by callers), not as a huge positive one that would skip the whole image.
check("horizon_row goes negative when the horizon is above the frame",
      geom.horizon_row() < 0, f"got {geom.horizon_row():.1f}")

# ─── 3. Bearings are symmetric and signed left-negative ─────────────────────
# gap_follow's convention: negative = left, positive = right. Getting this
# backwards would steer the robot INTO obstacles, so it is worth pinning.

left = geom.pixel_to_ground(0, geom.height - 1)
right = geom.pixel_to_ground(geom.width - 1, geom.height - 1)
check("left edge gives a negative bearing", left is not None and left[0] < 0,
      f"got {left}")
check("right edge gives a positive bearing", right is not None and right[0] > 0,
      f"got {right}")
check("left/right edge bearings are mirror images",
      close(abs(left[0]), abs(right[0]), 0.01),
      f"{left[0]:.2f} vs {right[0]:.2f}")
# A tilted camera's bottom corners look down AND out, so their ground
# bearings exceed half the lens FOV (31.1 deg here). This is real geometry,
# not an error — and it is why the scan bins over ground_fov_deg().
check("bottom-corner ground bearing exceeds half the LENS fov",
      abs(left[0]) > geom.hfov_deg / 2.0,
      f"got {abs(left[0]):.2f}, lens half-fov {geom.hfov_deg / 2.0:.2f}")
check("ground_fov_deg matches twice the corner bearing",
      close(geom.ground_fov_deg(), 2 * abs(left[0]), 0.01),
      f"got {geom.ground_fov_deg():.2f} vs {2 * abs(left[0]):.2f}")
check("ground fov is wider than the lens fov for a tilted camera",
      geom.ground_fov_deg() > geom.hfov_deg,
      f"{geom.ground_fov_deg():.1f} vs {geom.hfov_deg:.1f}")

# ─── 3b. The near blind zone is real and reported ───────────────────────────
# An obstacle inside min_visible_range() reads as open floor, not as an
# error. Pinning it here so a future camera-mounting change that widens the
# blind zone fails loudly instead of quietly degrading obstacle avoidance.

check("min_visible_range matches the bottom-centre pixel's range",
      close(geom.min_visible_range(), bottom[1], 1e-6),
      f"{geom.min_visible_range():.3f} vs {bottom[1]:.3f}")

from jetbot_nav.visual_scan import sim_jetank
from jetbot_nav.gap_follow import EMERGENCY_FORWARD

sim_geom = sim_jetank()
check("sim camera geometry constructs and sees the floor",
      sim_geom.min_visible_range() < 2.0,
      f"nearest visible {sim_geom.min_visible_range():.2f}")
# Not an assertion that the tuning is CORRECT — it is not yet — but a
# tripwire so the gap between the two stays visible while Phase 2 retunes.
print(f"      note: sim blind zone {sim_geom.min_visible_range():.2f} vs "
      f"gap_follow EMERGENCY_FORWARD {EMERGENCY_FORWARD} "
      f"({'OK' if sim_geom.min_visible_range() < EMERGENCY_FORWARD else 'BLIND ZONE EXCEEDS EMERGENCY STOP'})")

# ─── 4. Invalid geometry is rejected loudly ─────────────────────────────────
# A tilt of 0 means no ray ever reaches the floor; silently returning
# max_range everywhere would look exactly like "the path is clear".

for bad_kwargs, why in [
    (dict(height_m=0.0, tilt_deg=30.0), "zero height"),
    (dict(height_m=-1.0, tilt_deg=30.0), "negative height"),
    (dict(height_m=0.5, tilt_deg=0.0), "zero tilt (never meets the floor)"),
    (dict(height_m=0.5, tilt_deg=90.0), "90 deg tilt"),
]:
    try:
        CameraGeometry(**bad_kwargs)
        check(f"rejects {why}", False, "no exception raised")
    except ValueError:
        check(f"rejects {why}", True)


# ─── 5. Synthetic frame: empty floor reads fully clear ──────────────────────

def make_floor_frame(geom, floor_rgb=(120, 120, 120), sky_rgb=(30, 30, 60)):
    """Uniform floor below the horizon, distinct background above it."""
    frame = np.zeros((geom.height, geom.width, 3), dtype=np.uint8)
    horizon = int(max(0, min(geom.height, geom.horizon_row())))
    frame[:horizon] = sky_rgb
    frame[horizon:] = floor_rgb
    return frame


scan_geom = CameraGeometry(height_m=0.5, tilt_deg=30.0, width=640, height=480)
empty = make_floor_frame(scan_geom)
scan = free_space_scan(empty, scan_geom, n_rays=13, max_range=12.0)

check("empty floor: scan has the requested ray count",
      len(scan["distances"]) == 13 and len(scan["angles_deg"]) == 13)
check("empty floor: every ray reads max_range (nothing to hit)",
      all(d >= 12.0 for d in scan["distances"]),
      f"min was {min(scan['distances']):.2f}")
check("angles span the GROUND fov, ascending left to right",
      scan["angles_deg"][0] < 0 < scan["angles_deg"][-1]
      and scan["angles_deg"] == sorted(scan["angles_deg"])
      and close(scan["angles_deg"][-1] - scan["angles_deg"][0],
                scan_geom.ground_fov_deg(), 0.01))
check("scan dict matches gap_follow's contract",
      set(scan) == {"angles_deg", "distances", "max_range"})


# ─── 6. Synthetic frame: a block occludes the floor at a known distance ─────
# Place an obstacle so its BASE sits on a chosen image row, then check the
# scan reports the distance that row projects to. This is the end-to-end
# path: segmentation -> column walk -> projection -> binning.

base_row = 400            # obstacle's base, below the image centre
obstacle = make_floor_frame(scan_geom)
obstacle[:base_row, 280:360] = (200, 40, 40)     # red block, centred-ish

expected = scan_geom.pixel_to_ground(320, base_row)[1]
scan2 = free_space_scan(obstacle, scan_geom, n_rays=13, max_range=12.0)
nearest = min(scan2["distances"])

check("block: some ray now reads nearer than max_range",
      nearest < 12.0, f"min was {nearest:.2f}")
check("block: reported distance matches the base row's projection",
      close(nearest, expected, 0.05),
      f"got {nearest:.3f}, expected {expected:.3f}")

# The block sits slightly right of centre (columns 280-360, centre 320 of
# 640 -> essentially dead ahead), so the nearest ray should be near 0 deg.
nearest_angle = scan2["angles_deg"][scan2["distances"].index(nearest)]
check("block: nearest ray is close to dead ahead",
      abs(nearest_angle) < 10.0, f"got {nearest_angle:.1f} deg")

# Rays well off to the sides still see open floor.
check("block: peripheral rays remain clear",
      scan2["distances"][0] >= 12.0 and scan2["distances"][-1] >= 12.0,
      f"edges: {scan2['distances'][0]:.2f}, {scan2['distances'][-1]:.2f}")


# ─── 7. Nearer obstacles win within a ray bin ───────────────────────────────
# A scan is a safety input: when two things fall in one bin, the near one
# must be reported. Averaging would hide the closer hazard.

two_blocks = make_floor_frame(scan_geom)
two_blocks[:300, 300:320] = (200, 40, 40)    # further (higher base row)
two_blocks[:430, 320:340] = (200, 40, 40)    # nearer  (lower base row)
scan3 = free_space_scan(two_blocks, scan_geom, n_rays=13, max_range=12.0)

near_expected = scan_geom.pixel_to_ground(330, 430)[1]
check("overlapping obstacles: the NEARER one is reported",
      close(min(scan3["distances"]), near_expected, 0.06),
      f"got {min(scan3['distances']):.3f}, expected {near_expected:.3f}")


# ─── 8. Left/right placement maps to the correct side ───────────────────────
# The sign error that would steer into obstacles rather than away.

left_block = make_floor_frame(scan_geom)
left_block[:400, 40:160] = (200, 40, 40)
lscan = free_space_scan(left_block, scan_geom, n_rays=13, max_range=12.0)
langle = lscan["angles_deg"][lscan["distances"].index(min(lscan["distances"]))]
check("obstacle on the image's left -> negative bearing",
      langle < 0, f"got {langle:.1f} deg")

right_block = make_floor_frame(scan_geom)
right_block[:400, 480:600] = (200, 40, 40)
rscan = free_space_scan(right_block, scan_geom, n_rays=13, max_range=12.0)
rangle = rscan["angles_deg"][rscan["distances"].index(min(rscan["distances"]))]
check("obstacle on the image's right -> positive bearing",
      rangle > 0, f"got {rangle:.1f} deg")


# ─── 9. Mismatched frame size is rejected, not silently skewed ──────────────
# Feeding a differently-sized frame would shift every bearing without any
# visible symptom, which is the worst possible failure mode here.

try:
    free_space_scan(np.zeros((240, 320, 3), np.uint8), scan_geom)
    check("rejects a frame whose size disagrees with CameraGeometry", False,
          "no exception raised")
except ValueError:
    check("rejects a frame whose size disagrees with CameraGeometry", True)


# ─── 10. The output actually drives GapFollowController ─────────────────────
# The entire point of matching get_proximity_scan()'s shape.

from jetbot_nav.gap_follow import GapFollowController

ctrl = GapFollowController(angles_deg=scan["angles_deg"],
                           max_range=scan["max_range"])
l1, r1 = ctrl.step(scan["distances"])          # open floor
check("GapFollowController accepts a visual scan unchanged",
      isinstance(l1, float) and isinstance(r1, float))
check("open floor -> drives forward, both motors positive",
      l1 > 0 and r1 > 0, f"got ({l1:.2f}, {r1:.2f})")

ctrl2 = GapFollowController(angles_deg=scan2["angles_deg"],
                            max_range=scan2["max_range"])
for _ in range(3):
    l2, r2 = ctrl2.step(scan2["distances"])    # block dead ahead
check("obstacle ahead -> reacts (differs from the open-floor command)",
      (abs(l2 - r2) > 0.01) or (l2 < l1), f"got ({l2:.2f}, {r2:.2f})")


# ─── 11. annotate() runs and preserves frame shape ──────────────────────────

vis = annotate(obstacle, scan_geom, scan2, mask=floor_mask(obstacle))
check("annotate returns a same-shape uint8 image",
      vis.shape == obstacle.shape and vis.dtype == np.uint8)


# ─── Summary ────────────────────────────────────────────────────────────────

passed = sum(_results)
print(f"\n{passed}/{len(_results)} checks passed")
sys.exit(0 if all(_results) else 1)
