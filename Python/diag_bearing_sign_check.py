"""diag_bearing_turn_test.py — does turning BY the sensed bearing actually
centre the block, or push it further off?

The previous ground-truth version (diag_bearing_sign_check.py) turned out
to be unreliable here: this scene has more red blocks than the two named
ids (block_A_large/block_B_small) diag scripts elsewhere assume, so blob-
to-ground-truth matching by nearest distance kept picking the wrong
physical block (9-11 unit distance errors) and its OPPOSITE SIGN verdicts
aren't trustworthy.

This version sidesteps block identity entirely. It doesn't need to know
WHICH block it's looking at -- only that a pure rotation barely changes
distance to whatever block it's tracking, so the same blob can be
re-identified after turning by nearest distance with a tight tolerance,
no ground truth needed.

Logic: sense the largest block, note its bearing B0. Command a rotation
of exactly B0 (Unity yaw increases clockwise/right, matching this
codebase's "positive bearing = right" convention throughout --
gap_follow, target_seek, visual_scan all agree with each other on this).
If the reported bearing is correct, that turn should aim the robot
straight at the block, so re-sensing should find its bearing near 0. If
the bearing is mirrored, turning "toward" it by B0 actually turns AWAY,
so the block's bearing should roughly DOUBLE instead.

Needs Unity playing. MOVES the robot (rotates in place only, no driving).

    py -3.8 diag_bearing_turn_test.py
"""
import time

import jetson_utils
import sim_client
import sim_truth
from jetbot_nav import block_sensing, visual_scan

DISTANCE_MATCH_TOLERANCE_M = 0.15  # pure rotation shouldn't move distance much

sim_client.connect()
camera = jetson_utils.videoSource("csi://0")
geom = visual_scan.sim_jetank()


def capture():
    img = camera.Capture()
    if img is None:
        return None
    return jetson_utils.cudaToNumpy(img)


def sense():
    frame = capture()
    if frame is None:
        return []
    return block_sensing.sense_blocks(frame, geom, color="red")


# ─── Before ──────────────────────────────────────────────────────────────

before = sense()
if not before:
    print("No blocks sensed -- point the robot at a red block and try again.")
    raise SystemExit(1)

target = before[0]   # largest footprint, same choice autonomous_tower_build.py makes
B0 = target["bearing_deg"]
D0 = target["distance_m"]
pose0 = sim_truth.get_pose()

print(f"before turn: bearing={B0:+.1f} deg  distance={D0:.3f}  "
      f"(robot yaw={pose0['yaw_deg']:.1f})")
print(f"turning by exactly {B0:+.1f} deg (should aim straight at the block "
      f"if the bearing sign is correct)...\n")

sim_client.send_command({
    "command": "set_rotation",
    "rotation_y": pose0["yaw_deg"] + B0,
})
time.sleep(0.5)

# ─── After ───────────────────────────────────────────────────────────────

after = sense()
pose1 = sim_truth.get_pose()

if not after:
    print("Lost the block entirely after turning -- can't compare, but "
          "losing it after turning 'toward' it is itself suspicious.")
    raise SystemExit(1)

# Re-identify the same block: whichever re-sensed blob is closest in
# DISTANCE to the original (a pure rotation shouldn't change this much).
candidate = min(after, key=lambda b: abs(b["distance_m"] - D0))
B1 = candidate["bearing_deg"]
D1 = candidate["distance_m"]

print(f"after turn:  bearing={B1:+.1f} deg  distance={D1:.3f}  "
      f"(robot yaw={pose1['yaw_deg']:.1f})")

if abs(D1 - D0) > DISTANCE_MATCH_TOLERANCE_M:
    print(f"\nWARNING: matched blob's distance moved by {abs(D1 - D0):.3f} m, "
          f"more than the {DISTANCE_MATCH_TOLERANCE_M} m tolerance for a pure "
          f"rotation -- this may not be the same block. Treat the verdict "
          f"below with caution and consider re-running facing a more "
          f"isolated block.")

print()
if abs(B1) < abs(B0) * 0.5:
    print(f"VERDICT: bearing shrank ({B0:+.1f} -> {B1:+.1f}) -- the sign is "
          f"CORRECT. Turning by the reported bearing centres the block, as "
          f"it should.")
elif abs(B1) > abs(B0) * 1.3:
    print(f"VERDICT: bearing grew instead of shrinking ({B0:+.1f} -> {B1:+.1f}) "
          f"-- *** MIRRORED ***. Turning by the reported bearing pushes the "
          f"block further off-centre instead of centring it. The bug is in "
          f"how the bearing is computed/reported (visual_scan / the camera "
          f"capture path), not in navigate_to_bearing()'s motor calls.")
else:
    print(f"VERDICT: ambiguous ({B0:+.1f} -> {B1:+.1f}) -- roughly unchanged. "
          f"Re-run against a block that's more isolated from others so the "
          f"distance-based re-match is unambiguous.")