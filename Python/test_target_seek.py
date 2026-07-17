"""
test_target_seek.py — offline regression suite for jetbot_nav.target_seek.

Run:  py test_target_seek.py
No Unity needed — synthetic scans and bbox streams only, same pattern as
test_gap_logic.py.
"""

import math

from jetbot_nav.gap_follow import (
    GapFollowController, DEFAULT_MAX_RANGE, FORWARD, PIVOT,
)
from jetbot_nav.target_seek import (
    SeekingGapFollowController, TargetTracker,
    bbox_to_bearing, bearing_from_blob, blob_height_fraction,
    estimate_target_distance, is_arrived,
)

FOV, N = 120.0, 13
ANGLES = [-FOV / 2 + i * (FOV / (N - 1)) for i in range(N)]
OPEN = [DEFAULT_MAX_RANGE] * N   # fully clear scan

checks = []


def check(name, cond):
    checks.append((name, bool(cond)))
    print(("PASS " if cond else "FAIL ") + name)


# ── 1. Backward compatibility: no target -> identical to base class ─────────

def scan_with_two_gaps():
    """Blocks the center rays, leaves a gap on each side (indices as built)."""
    d = [DEFAULT_MAX_RANGE] * N
    for i in range(5, 8):       # block the middle
        d[i] = 1.0
    return d


base = GapFollowController(angles_deg=ANGLES)
seek = SeekingGapFollowController(angles_deg=ANGLES)
stream = [OPEN, scan_with_two_gaps(), scan_with_two_gaps(), OPEN]
same = True
for d in stream:
    mb = base.step(d)
    ms = seek.step(d, target_bearing_deg=None)
    if mb != ms:
        same = False
check("no-target output matches base GapFollowController exactly", same)


# ── 2. Open field, target dead ahead -> drives straight ─────────────────────

c = SeekingGapFollowController(angles_deg=ANGLES)
l, r = c.step(OPEN, target_bearing_deg=0.0)
check("open field + target at 0deg -> symmetric motors (straight)",
      abs(l - r) < 1e-6 and l > 0)


# ── 3. Open field, target off to the right -> turns right, doesn't ignore it -

c = SeekingGapFollowController(angles_deg=ANGLES)
l, r = c.step(OPEN, target_bearing_deg=30.0)
check("open field + target at +30deg -> steers right (left motor faster)",
      l > r)


# ── 4. Among two safe gaps, prefers the one closer to the target bearing ────

def scan_two_gaps_left_and_right():
    """
    13 rays over 120deg (10deg spacing), indices 0..12, angle = -60+10*i.
    Block rays 4-8 (angle -20..+20) so there's a left gap (0-3, -60..-30)
    and a right gap (9-12, +30..+60), both wide enough (>=3 rays).
    """
    d = [DEFAULT_MAX_RANGE] * N
    for i in range(4, 9):
        d[i] = 3.5   # inside GAP_THRESHOLD (blocked for gap-finding) but
                      # forward clearance stays >= STOP_FORWARD so this
                      # scan reaches gap SELECTION rather than PIVOT/BACKUP
    return d


c = SeekingGapFollowController(angles_deg=ANGLES)
d = scan_two_gaps_left_and_right()
l, r = c.step(d, target_bearing_deg=45.0)   # target is well to the right
dbg = c.debug()
check("target to the right -> right-side gap chosen over left-side gap",
      dbg["target_deg"] is not None and dbg["target_deg"] > 0)

c2 = SeekingGapFollowController(angles_deg=ANGLES)
l2, r2 = c2.step(d, target_bearing_deg=-45.0)   # target is well to the left
dbg2 = c2.debug()
check("target to the left -> left-side gap chosen over right-side gap",
      dbg2["target_deg"] is not None and dbg2["target_deg"] < 0)


# ── 5. Recovery states fully suppress target bias ───────────────────────────

def scan_emergency():
    """Everything close in front -> triggers BACKUP on the very first tick."""
    d = [DEFAULT_MAX_RANGE] * N
    for i in range(4, 9):
        d[i] = 0.5   # inside EMERGENCY_FORWARD
    return d


c_target = SeekingGapFollowController(angles_deg=ANGLES)
c_plain = GapFollowController(angles_deg=ANGLES)
d = scan_emergency()
out_target = c_target.step(d, target_bearing_deg=45.0)
out_plain = c_plain.step(d)
check("emergency/BACKUP: seeking output identical to plain controller "
      "(target fully suppressed)", out_target == out_plain)
check("emergency correctly entered BACKUP", c_target.state == "BACKUP")


# ── 6. TargetTracker holds last bearing across brief detection loss ─────────

t = TargetTracker(hold_ticks=3)
seq = [10.0, None, None, None, None, None]   # 3 holds, then gives up (>3)
results = [t.update(b) for b in seq]
check("tracker returns real bearing when detected", results[0] == 10.0)
check("tracker holds last bearing through brief loss (ticks 2-4)",
      results[1] == 10.0 and results[2] == 10.0 and results[3] == 10.0)
check("tracker gives up after hold_ticks exceeded", results[5] is None)

t2 = TargetTracker(hold_ticks=3)
t2.update(10.0)
t2.update(None)
reseen = t2.update(20.0)
check("tracker immediately re-locks to a fresh detection during hold window",
      reseen == 20.0)


# ── 7. bbox_to_bearing / bearing_from_blob / estimate_target_distance ───────

check("bbox dead-center -> bearing 0",
      abs(bbox_to_bearing(320, 640, 90.0) - 0.0) < 1e-6)
check("bbox at right edge -> bearing = +half FOV",
      abs(bbox_to_bearing(640, 640, 90.0) - 45.0) < 1e-6)
check("bbox at left edge -> bearing = -half FOV",
      abs(bbox_to_bearing(0, 640, 90.0) - (-45.0)) < 1e-6)

# Real perception.largest_blob() shape: {"x1","y1","x2","y2","cx","cy","area"}
# in the camera's actual frame pixel space (640x480 in sim).
blob_center = {"x1": 280.0, "y1": 200.0, "x2": 360.0, "y2": 280.0,
               "cx": 320.0, "cy": 240.0, "area": 6400.0}   # cx=320 = dead ahead
blob_right  = {"x1": 560.0, "y1": 200.0, "x2": 640.0, "y2": 280.0,
               "cx": 600.0, "cy": 240.0, "area": 6400.0}   # cx=600, far right
check("bearing_from_blob: centered blob -> bearing 0 (default 640 width)",
      abs(bearing_from_blob(blob_center, 90.0) - 0.0) < 1e-6)
check("bearing_from_blob: right-of-center blob -> positive bearing",
      bearing_from_blob(blob_right, 90.0) > 0)
check("bearing_from_blob: explicit image_width overrides the 640 default",
      abs(bearing_from_blob({"cx": 160.0}, 90.0, image_width=320.0) - 0.0) < 1e-6)
check("blob_height_fraction matches (y2-y1)/480 (default height)",
      abs(blob_height_fraction(blob_center) - (80.0 / 480.0)) < 1e-9)
check("blob_height_fraction respects explicit image_height",
      abs(blob_height_fraction(blob_center, image_height=240.0) - (80.0 / 240.0)) < 1e-9)

scan = {"angles_deg": ANGLES, "distances": OPEN, "max_range": DEFAULT_MAX_RANGE}
scan_hit = {"angles_deg": ANGLES,
            "distances": [DEFAULT_MAX_RANGE if i != 6 else 2.5 for i in range(N)],
            "max_range": DEFAULT_MAX_RANGE}
check("distance lookup on a ray that sees nothing -> None",
      estimate_target_distance(0.0, scan) is None)
check("distance lookup matches the nearest ray with a real hit",
      abs(estimate_target_distance(ANGLES[6], scan_hit) - 2.5) < 1e-6)
check("distance lookup outside tolerance of any ray -> None",
      estimate_target_distance(ANGLES[6] + 20, scan_hit) is None)


# ── 8. Arrival check ─────────────────────────────────────────────────────────

big_blob = {"x1": 100.0, "y1": 50.0, "x2": 500.0, "y2": 450.0}    # height 400/480
small_blob = {"x1": 300.0, "y1": 300.0, "x2": 340.0, "y2": 340.0}  # height 40/480

check("arrived by ray distance", is_arrived(target_distance=1.0))
check("not arrived, still far by ray distance", not is_arrived(target_distance=5.0))
check("arrived by blob-size fallback",
      is_arrived(target_distance=None, blob=big_blob))
check("not arrived, small blob, no distance",
      not is_arrived(target_distance=None, blob=small_blob))


# ── Summary ───────────────────────────────────────────────────────────────

n_fail = sum(1 for _, ok in checks if not ok)
print(f"\n{len(checks) - n_fail}/{len(checks)} checks passed")
if n_fail:
    raise SystemExit(1)