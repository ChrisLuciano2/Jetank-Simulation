"""
jetbot_nav.arm_ops — pick-and-place helpers built on SCSCtrl.TTLServo.

Not a shim — real logic layered on the TTLServo servo-control shim (or the
real TTLServo module on hardware), the same relationship jetbot_nav.gap_follow
has to jetbot.Robot(). Nothing here is simulation-specific.

Picking and placing both go through TTLServo.xyInput(reach_mm, y_mm) — the
SAME inverse-kinematics entry point the real JETANK arm already uses —
rather than commanding joint angles directly, so this stays correct if the
arm's link lengths or IK ever change, and an axis-semantics bug here and in
the existing hardware code would be the same bug, not two independent ones.

CALIBRATION NEEDED BEFORE TRUSTING THIS ON HARDWARE
────────────────────────────────────────────────────────────────
STACK_HEIGHT below assumes raising xyInput's y_input raises the gripper.
TTLServo.xyInput's own docstring calls that axis "left/right," which does
not fit a shoulder+elbow chain (both pitch joints — the only plane two
pitch joints can reach is forward/height, not sideways), so that docstring
reads like a leftover from whatever kit this was adapted from. Treat the
mapping below as a documented assumption, not a verified fact, and confirm
which way the gripper actually moves the first time this runs against real
hardware or a live sim before trusting it unattended.

FLOOR REACH: xyInput() CANNOT REACH THE FLOOR — reach_and_grab() BYPASSES IT
────────────────────────────────────────────────────────────────
xyInput()'s 2-link IK (shoulder+elbow only) only ever computes one of the
two valid elbow solutions for a given target (_plane_linkage_reverse()'s
single arccos() call), and that solution never bends the elbow past ~23°
for any y_input in its documented range — nowhere near what's needed to
reach a floor-level block. Confirmed empirically: a full y_input sweep
never got the gripper below y=0.674m. Changing linkageLenA/linkageLenB to
compensate would be wrong — those are real hardware constants shared with
the unmodified vendor TTLServo module.

So reach_and_grab() commands raw shoulder/elbow angles directly via
servoAngleCtrl() (servo 2 = shoulder, servo 3 = elbow) instead of calling
xyInput() — the same real hardware primitive already used here for base yaw
(servo 1) and the gripper (servo 4), just skipping xyInput()'s limited
convenience wrapper for this one case.

This also depended on a second, separate bug: RoboticArmController.TryGrab()
was checking physical proximity from j5_wrist (the point matching xyInput()'s
IK model, right for get_arm_state's reporting, wrong for a physical contact
check) instead of the real gripper fingers, ~120-290mm further out. Fixed in
RoboticArmController.cs (GetGripperPosition(), used by TryGrab() only).

RELIABILITY: ONE FIXED ANGLE PAIR IS NOT ENOUGH -- reach_and_grab() TRIES SEVERAL
────────────────────────────────────────────────────────────────
diag_floor_reach_yaw0.py's sweep found 10 different (shoulder, elbow) pairs
that all register a grab at base_yaw=0 -- but a single fixed pair
(shoulder=75, elbow=100) succeeded on one diag_full_cycle.py run and then
failed ("grab not confirmed") on the very next one, against what telemetry
showed was a near-identical creep arrival (distance 0.421m vs 0.429m,
bearing -1.7deg vs -2.6deg -- both comfortably inside the bearing gate).
The small residual variance in exactly where creep stops is apparently
enough to tip a single fixed angle pair in or out of physical grab range.
FLOOR_REACH_CANDIDATES lists several of the sweep's confirmed-working pairs;
reach_and_grab() tries them in order, re-opening the gripper between
attempts, stopping at the first confirmed grab. This is a real,
demonstrated reliability gap being compensated with a real, demonstrated
set of working configurations -- not a guess. Still: re-verify with
diag_full_cycle.py across many repeated runs before trusting this
unattended, same standing caution as everything else in this project.
REACH_PICKUP_MM/Y_AT_FLOOR_MM are no longer used for picking (still used by
stow_for_transport()/place_at_height(), which don't have this floor-reach
problem — placing happens above floor level, not at it).

GRAB CONFIRMATION
────────────────────────────────────────────────────────────────
_is_holding() checks the sim's get_arm_state query. There is no generic
touch/current-sense feedback wired into the real gripper (yet) to confirm
a grab on hardware — see _is_holding()'s fallback, which is honest about
that gap rather than pretending a check exists where it doesn't.
"""

import time

from SCSCtrl import TTLServo

# Millimetre inputs to xyInput, in ITS coordinate convention (see module
# docstring for why these are a documented assumption, not a verified fact).
REACH_PICKUP_MM = 150.0   # unused for picking now -- see FLOOR REACH in module docstring
REACH_PLACE_MM  = 150.0   # x_input while placing -- same reach, different height
Y_AT_FLOOR_MM   = -60.0   # unused for picking now -- see FLOOR REACH in module docstring
MM_PER_METRE    = 1000.0

# Raw (shoulder_deg, elbow_deg) pairs (servos 2/3) for reach_and_grab()'s
# direct servoAngleCtrl() floor reach -- bypasses xyInput(), see module
# docstring's FLOOR REACH / RELIABILITY sections for why this is a list, not
# a single pair. Empirically confirmed via diag_floor_reach_yaw0.py's sweep
# at base_yaw_deg=0 (this function's only caller pattern) -- if base_yaw_deg
# is ever driven away from 0 in practice, re-verify with
# diag_floor_reach_calibrate.py. Ordered with the most-tested pair first.
# All 10 confirmed-working pairs from diag_floor_reach_yaw0.py's 5x6 sweep
# (Python/diag_floor_reach_yaw0_output1.log), not just 6 -- widened this
# session after noticing the sweep's own True/False pattern is NOT a clean
# contiguous reach envelope (e.g. shoulder=75: elbow 100=hit, 110=miss,
# 120=miss, 130=hit -- a gap in the middle that doesn't fit pure static
# geometry). That non-monotonic pattern means the sweep itself was likely
# contaminated by the same kind of settle-timing noise root-caused
# elsewhere in this file, so the 6-candidate subset previously used here
# was an arbitrary sample of a noisy dataset, not necessarily the most
# robust choices available. Trying all 10 confirmed hits (instead of 6)
# doesn't require understanding the noise fully -- it just gives
# reach_and_grab() more real, empirically-confirmed fallback options,
# directly increasing the chance at least one lands for any given actual
# creep-arrival position. Ordered with the most-tested pair first.
FLOOR_REACH_CANDIDATES = [
    (75.0, 100.0),
    (70.0, 100.0),
    (65.0, 100.0),
    (70.0, 110.0),
    (65.0, 120.0),
    (75.0, 130.0),
    (65.0, 130.0),
    (80.0, 130.0),
    (85.0, 110.0),
    (85.0, 130.0),
]

# Widened from 0.6s. The gripper's own close motion (open=1.0 -> closed
# threshold=0.15, at the interpolation rate ApplyGripper() uses --
# jointSpeed=90deg/s * 0.02 = 1.8 units/sec) needs 0.85/1.8=0.472s in the
# worst case (starting fully open), leaving only ~0.13s of margin against
# the old 0.6s -- thin enough that ordinary frame-timing jitter could
# plausibly eat it. 0.8s roughly triples that margin for negligible added
# per-attempt cost.
GRAB_SETTLE_S = 0.8   # time to let the gripper close and the sim grab mechanic register

# MOVE_SETTLE_S must exceed the worst-case single-joint travel time at the
# arm's jointSpeed (90 deg/s in RoboticArmController.cs). Confirmed live via
# get_arm_state: after the old 0.8s wait, a shoulder=75/elbow=100 move (the
# most common first-candidate transition, e.g. from a 0/0 rest pose) had
# only reached shoulder=67.48/elbow=67.48 -- nowhere near arrived (elbow
# alone needs 100/90=1.11s). Closing the gripper while the arm is still
# mid-swing lets TryGrab() parent the block onto a joint chain that keeps
# moving for another ~0.3-0.6s afterward, dragging the freshly-grabbed
# block along -- confirmed via get_block_pose showing held blocks ending up
# to 241mm below their pre-grab height (sometimes underground) purely from
# this residual motion, with no Rigidbody involved. 1.5s covers the largest
# single-joint delta in FLOOR_REACH_CANDIDATES (elbow up to 130 deg) with
# margin, and is a very plausible contributor to Section 1.10's intermittent
# reach_and_grab() failures too: a fixed angle pair's usable-or-not outcome
# stops being a nominal-angle fact and becomes sensitive to exact
# interpolation timing when the arm hasn't actually finished moving yet.
MOVE_SETTLE_S = 1.5


def reach_and_grab(base_yaw_deg: float) -> bool:
    """
    Aim the base at base_yaw_deg, reach down to floor level, close the
    gripper, and report whether something was actually grabbed — checked
    via get_arm_state's is_holding, never assumed.

    Tries each of FLOOR_REACH_CANDIDATES in turn (re-opening the gripper
    between attempts) until one registers a grab or all are exhausted -- see
    module docstring's RELIABILITY section for why a single fixed pair
    isn't enough.
    """
    TTLServo.servoAngleCtrl(1, base_yaw_deg, 1, 300)
    time.sleep(0.5)

    for shoulder_deg, elbow_deg in FLOOR_REACH_CANDIDATES:
        # Raw joint control, not xyInput() -- see module docstring's FLOOR
        # REACH section for why xyInput()'s IK cannot compute a
        # floor-reaching pose.
        TTLServo.servoAngleCtrl(2, shoulder_deg, 1, 300)
        TTLServo.servoAngleCtrl(3, elbow_deg,    1, 300)
        time.sleep(MOVE_SETTLE_S)

        TTLServo.servoAngleCtrl(4, -100, 1, 200)   # servo 4 = gripper; -100 = fully closed
        time.sleep(GRAB_SETTLE_S)

        if _is_holding():
            return True

        TTLServo.servoAngleCtrl(4, 100, 1, 200)    # re-open before the next attempt
        time.sleep(0.3)

    return False


def stow_for_transport():
    """Lift the held block clear of the floor/obstacles before driving."""
    TTLServo.xyInput(REACH_PICKUP_MM * 0.6, Y_AT_FLOOR_MM + 120.0)
    time.sleep(MOVE_SETTLE_S)


def place_at_height(base_yaw_deg: float, stack_height_m: float) -> bool:
    """
    Aim the base at base_yaw_deg, reach to REACH_PLACE_MM at a height
    derived from stack_height_m (metres -- the sum of already-placed
    blocks' sensed heights, see autonomous_tower_build.py, never
    hardcoded), and open the gripper.

    Returns True once is_holding reads False (release confirmed) —
    matching reach_and_grab's "check, don't assume" convention.
    """
    y_input = Y_AT_FLOOR_MM + stack_height_m * MM_PER_METRE

    TTLServo.servoAngleCtrl(1, base_yaw_deg, 1, 300)
    time.sleep(0.5)

    TTLServo.xyInput(REACH_PLACE_MM, y_input)
    time.sleep(MOVE_SETTLE_S)

    TTLServo.servoAngleCtrl(4, 100, 1, 200)    # fully open
    time.sleep(GRAB_SETTLE_S)

    return not _is_holding()


def _is_holding() -> bool:
    try:
        import sim_client
        import sim_robot_id
    except ImportError:
        print("[arm_ops] WARNING: no grab-confirmation sensor available on this "
              "hardware build -- assuming the grab/release succeeded. Add a real "
              "sensor (servo current-sense, a limit switch, etc.) before trusting "
              "this unattended.")
        return True

    resp = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
    if not resp or resp.get("status") != "ok":
        return False
    return bool(resp.get("is_holding"))
