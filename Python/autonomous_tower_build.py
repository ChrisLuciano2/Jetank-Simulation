"""
autonomous_tower_build.py — two robots sense red blocks, sort them by
size, and build a shared tower together: largest block on the bottom,
progressively smaller blocks going up. Nothing about which blocks exist,
their sizes, or the current tower height is hardcoded — all of it is
sensed or computed from what was sensed.

RUN, one copy per robot (real or simulated):

    ROBOT_ID=robot_a MQTT_BROKER_HOST=<broker> python autonomous_tower_build.py
    ROBOT_ID=robot_b MQTT_BROKER_HOST=<broker> python autonomous_tower_build.py

REQUIRES A VISIBLE BUILD-ZONE MARKER (see BUILD_MARKER_COLOR below)
────────────────────────────────────────────────────────────────
This codebase's own constraint (README: "The JETANK has a camera and
nothing else — no lidar, no ultrasonic," and there are no wheel encoders
or an IMU either) means there is no shared (x, y) coordinate frame either
robot can navigate to directly — "drive to world position (2, 0)" is not
something a camera-only robot with no localization can do. So instead of
a hardcoded meeting-point coordinate, the tower's build zone is something
BOTH ROBOTS SENSE the same way they sense blocks: a distinctly-coloured
marker (BUILD_MARKER_COLOR) they steer toward using the identical
bearing-based approach as block_sensing, just with perception targeting a
different colour. This needs a marker of that colour placed in the
workspace (in Unity: a prop tagged however jetbot_nav.perception's colour
thresholds expect — just a solid-coloured object, no special tag needed,
since perception.py works on raw pixels) — this script does not create
one for you.

TWO INDEPENDENT COLLISION-AVOIDANCE LAYERS, ON PURPOSE
────────────────────────────────────────────────────────────────
  1. REACTIVE / HARD (what actually makes "never collide" true):
     jetbot_nav.gap_follow's obstacle avoidance, fed by the camera's own
     free-space scan (visual_scan). The other robot's body is not
     floor-coloured, so it occludes the floor exactly like any obstacle
     and gap_follow steers around or stops for it. This holds even if the
     network, the broker, or the other robot's whole process dies — it
     does not depend on cooperation, which is the whole point.
  2. COOPERATIVE / SOFT (what makes the run smooth, not what makes it
     safe): robot_coord's bounded-slack pacing and placement-clear
     signaling, so the two robots don't even attempt to occupy the build
     zone at once in the common case, rather than relying on last-second
     reactive avoidance on every approach.

The "exclusion zone" in the cooperative layer is each robot's own SENSED
distance to the build marker, not a world coordinate — see
_marker_pose_proxy() below for exactly how that's threaded through
robot_coord.Coordinator's existing {x, y} pose interface without needing
to change it.

WHAT'S SENSED VS COMPUTED VS COORDINATED
────────────────────────────────────────────────────────────────
  - Which blocks exist, where they are, and how big they are: sensed every
    cycle from the camera (jetbot_nav.block_sensing), never assumed.
  - Build order (largest first): a sort on sensed sizes, not a fixed list.
  - Current tower height (where the next block goes): the sum of sensed
    heights of blocks already placed, taken as the max of what either
    robot has published (see current_tower_height()) so the two robots'
    bookkeeping can't silently diverge.

WHAT'S APPROXIMATED / NEEDS REAL-HARDWARE CALIBRATION BEFORE TRUSTING THIS
────────────────────────────────────────────────────────────────
  - jetbot_nav.arm_ops's reach values and its height -> y_input mapping
    are a documented assumption about TTLServo.xyInput's axis convention,
    not a verified fact — see arm_ops.py's module docstring.
  - Grab/release confirmation has a real check in simulation and an
    honest "can't verify, assuming success" fallback on hardware with no
    grip sensor — see arm_ops._is_holding()'s docstring.
  - block_sensing's size/distance estimates carry the same monocular
    accuracy limits as visual_scan's obstacle distances.
  - This has been exercised offline (test_block_sensing.py) and against
    the live Unity sim for the sensing/coordination pieces; the full
    sense-navigate-grab-stack loop has NOT yet been run end-to-end live
    end to end — see the project chat history for exactly what has and
    hasn't been verified before running this unattended.
"""

import csv
import math
import os
import time

import jetson_utils
from jetbot import Robot
from jetbot_nav import arm_ops, block_sensing, gap_follow, perception, visual_scan
from jetbot_nav.target_seek import (
    SeekingGapFollowController, bearing_from_blob, estimate_target_distance, is_arrived,
)
from robot_coord import Coordinator

SENSE_RADIUS_M     = 3.0     # "blocks in front of them, within X radius"
BUILD_MARKER_COLOR = "blue"  # must not collide with block colour or robot colour
MAX_SLACK          = 1
EXCLUSION_RADIUS_M = 0.6     # how close to the marker counts as "the seam"
POLL_HZ            = 10.0
NAV_TIMEOUT_S       = 25.0
MAX_BLOCKS          = 20     # safety cap so a sensing loop bug can't run forever

# target_seek.ARRIVAL_DISTANCE (1.2m) is tuned for general obstacle-avoidance
# driving and is too loose for anything the arm needs to physically reach --
# RoboticArmController.grabRadius in Unity is 0.6m, and TTLServo.xyInput's
# REACH_PICKUP_MM/REACH_PLACE_MM (arm_ops.py) reach only 150mm past that from
# the base pivot. PICKUP_ARRIVAL_DISTANCE_M is the real target for the final
# approach -- comfortably inside grabRadius, with margin for proximity-scan
# noise.
#
# It is NOT passed to navigate_to_bearing() (that was tried first -- see
# nav_logs/robot_a_pickup0_20260821_140529.csv). gap_follow's own
# EMERGENCY_FORWARD (1.6m) and STOP_FORWARD (3m) sit well outside grab
# range, so asking navigate_to_bearing to close all the way to 0.45m makes
# its avoidance stack treat the pickup target itself as a collision threat:
# that log shows BACKUP on the very first tick, 0.98m out, cascading into a
# BACKUP/SEARCH/FORWARD limit cycle that ended farther from the block than
# it started. Instead, navigate_to_bearing() is left at its own proven-safe
# ARRIVAL_DISTANCE (1.2m -- outside EMERGENCY_FORWARD, so a normal arrival
# check can succeed before any avoidance state ever triggers), and
# creep_to_target() below closes the remaining stretch itself, deliberately
# outside the full gap_follow state machine.
PICKUP_ARRIVAL_DISTANCE_M = 0.45

# creep_to_target()'s own tuning. Deliberately separate from gap_follow's
# constants (TURN_GAIN, MAX_SPEED, etc.) -- this is a much simpler
# proportional steer meant only to cover the last ~1m at low speed, not a
# general driving controller.
CREEP_SPEED               = 0.18
CREEP_TURN_GAIN           = 1.2
CREEP_MIN_TURN            = 0.05  # was 0.15 -- comparable to CREEP_SPEED
                           # itself (0.18), so _creep_steer's old raw-radians
                           # turn magnitude (up to 1.0, floored at this)
                           # swamped forward speed for almost any nonzero
                           # bearing, turning "creep toward the target" into
                           # "pivot in place." Live telemetry
                           # (nav_logs/robot_a_creep_pickup0_20260821_174657.csv)
                           # showed distance stuck at ~0.82-0.93m for a full
                           # 8 seconds with motor pairs like (-0.41, 0.77) --
                           # a near-full in-place spin, not a gentle arc.
                           # _creep_steer now scales turn by CREEP_SPEED
                           # (mirroring gap_follow._steer's own approach),
                           # so this floor needs to be small relative to
                           # speed, not comparable to it.
CREEP_TARGET_EXCLUSION_DEG = 20.0  # rays within this many degrees of the
                           # freshly re-sensed target bearing are the target
                           # itself, not a hazard -- matches
                           # target_seek.RAY_MATCH_TOLERANCE_DEG's spirit but
                           # widened, since the target is close and angularly
                           # large in-frame during this phase, not a distant
                           # point.
CREEP_TIMEOUT_S           = 12.0
CREEP_MAX_DISTANCE_JUMP_M = 0.2    # a tick-to-tick RANGE change bigger than
                           # this is treated as a wrong-blob pick, not real
                           # motion, and the previous bearing/distance are
                           # held instead of accepted. Was 0.5 -- too loose:
                           # nav_logs/robot_a_creep_pickup0_20260821_175148.csv
                           # showed real per-tick closing steps of at most
                           # ~0.13m during a good, steadily-closing stretch
                           # (1.175 -> 1.048 -> 0.924 -> 0.882m), then a
                           # 0.286m jump (0.889 -> 1.175m) got ACCEPTED under
                           # the 0.5 bar and silently swapped tracking onto
                           # the farther block, which the approach never
                           # recovered from. 0.2 comfortably covers every
                           # real step seen so far while rejecting that jump.
                           #
                           # An earlier version filtered by BEARING jump
                           # instead (reject a jump-target if its bearing
                           # moved too far). That over-corrected: live
                           # telemetry (nav_logs/robot_a_creep_pickup0_20260821_174027.csv)
                           # showed a real, necessary bearing swing get
                           # rejected as "noise" and held for 5+ ticks
                           # straight (motors frozen at the same command
                           # each tick, since steering is a pure function of
                           # the held bearing), driving the robot right past
                           # the block -- final rows show the scan reading
                           # empty floor dead ahead. A close, physically
                           # large block's on-screen bearing can legitimately
                           # swing a lot tick to tick (it goes from "ahead"
                           # to "beside" over a small approach distance) even
                           # though nothing is wrong; its measured RANGE
                           # doesn't have that problem -- a real target's
                           # distance changes smoothly, so a candidate whose
                           # distance discontinuously jumps is the one to
                           # distrust, not one whose bearing does.
CREEP_LOST_HOLD_TICKS     = 5      # consecutive rejected/blob-less ticks
                           # tolerated before admitting the target is
                           # genuinely gone -- mirrors
                           # target_seek.TargetTracker's hold-through-brief-
                           # loss debounce, same principle applied here.
CREEP_HAZARD_FORWARD_M    = 0.5   # forward-distance hazard bar for creep,
                           # deliberately tighter than gap_follow's own
                           # EMERGENCY_FORWARD (1.6m). That constant assumes
                           # the robot might be moving at up to MAX_SPEED
                           # (0.5) and needs real stopping distance; creep
                           # moves at CREEP_SPEED (0.18) and re-evaluates
                           # every tick, so it needs much less margin to
                           # react safely. Live telemetry
                           # (nav_logs/robot_a_creep_pickup0_20260821_172933.csv)
                           # showed a STATIC scene feature (a wall/prop, not
                           # either red block -- same reading recurred across
                           # many different approach attempts) sitting at
                           # ~0.87-0.99m off to one side, which is closer
                           # than EMERGENCY_FORWARD but not actually
                           # dangerous at creep speed -- reusing gap_follow's
                           # constant here made every approach into this
                           # corner of the scene un-completable.
CREEP_ARRIVAL_MAX_BEARING_DEG = 15.0   # is_arrived()'s bearing gate for
                           # creep's own arrival check (see is_arrived()'s
                           # docstring). arm_ops.reach_and_grab() is always
                           # called with base_yaw_deg=0.0 -- it has no way
                           # to compensate for however far off-center the
                           # chassis actually is, so creep must not declare
                           # "arrived" purely on distance/blob-size while
                           # still meaningfully off-angle. Confirmed via
                           # nav_logs telemetry (creep_pickup*.csv files
                           # from the camera-height-fix testing session):
                           # arrived=True fired on tick 1 with residual
                           # target_bearing_deg of -20 to -27 on every
                           # retry, and reach_and_grab() failed every time.
                           # 15 deg leaves comfortable margin under that.

# Set NAV_LOG_DIR (an env var, so hardware runs stay log-free by default) to
# have every navigate_to_bearing() approach recorded to CSV, one row per
# tick -- same columns jetbot_nav.gap_follow's own drive_with_gap_following()
# logs, plus this module's own arrival diagnostics, so a misbehaving
# approach (e.g. the controller entering PIVOT/SEARCH because it treats the
# pickup target itself as an obstacle) can be replayed and inspected offline
# with jetbot_nav.gap_follow.replay_log() instead of guessed at from prints.
NAV_LOG_DIR = os.environ.get("NAV_LOG_DIR")
NAV_LOG_FIELDS = ["t", "distances", "motor_l", "motor_r", "state", "target_deg",
                   "search_side", "target_bearing_deg", "ray_distance_m", "arrived"]
CREEP_LOG_FIELDS = ["t", "distances", "motor_l", "motor_r", "target_bearing_deg",
                     "ray_distance_m", "arrived", "hazard", "blob_seen"]


def _nav_log_path(coord: "Coordinator", tag: str):
    """None (logging off) unless NAV_LOG_DIR is set; one timestamped CSV per
    approach so successive attempts at the same block don't overwrite each
    other."""
    if not NAV_LOG_DIR:
        return None
    os.makedirs(NAV_LOG_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    return os.path.join(NAV_LOG_DIR, f"{coord.robot_id}_{tag}_{ts}.csv")


def _capture_frame(camera):
    img = camera.Capture()
    if img is None:
        return None
    return jetson_utils.cudaToNumpy(img)


def navigate_to_bearing(robot, controller, camera, geom, target_bearing_deg,
                         timeout_s=NAV_TIMEOUT_S, poll_hz=POLL_HZ,
                         arrival_distance=None, log_path=None):
    """
    Drive toward target_bearing_deg using the SAME camera-based obstacle
    scan gap_follow already trusts. This is what makes the other robot
    register as something to steer around or stop for whenever it's in
    view, with no special-case code: free_space_scan() has no concept of
    "which obstacle is a robot," it just sees non-floor pixels, and the
    other robot's body is one of those.

    arrival_distance overrides target_seek's general-purpose
    ARRIVAL_DISTANCE for this approach if given. Left at the default for
    pickup/placement approaches specifically -- see PICKUP_ARRIVAL_DISTANCE_M's
    comment for why closing all the way to grab range belongs to
    creep_to_target() below instead, not to a tighter value here.

    log_path, if given, records one CSV row per tick -- scan distances,
    motor outputs, controller state/target/search_side (same shape
    gap_follow.drive_with_gap_following() logs), plus this call's own
    target_bearing_deg/ray_distance_m/arrived -- replayable offline with
    jetbot_nav.gap_follow.replay_log(). See NAV_LOG_DIR above.

    Returns True once arrived (per target_seek.is_arrived), False on
    timeout.
    """
    deadline = time.time() + timeout_s
    interval = 1.0 / poll_hz
    arrived_kwargs = {} if arrival_distance is None else {"arrival_distance": arrival_distance}

    log_file = writer = None
    start = time.time()
    if log_path:
        log_file = open(log_path, "w", newline="")
        writer = csv.DictWriter(log_file, fieldnames=NAV_LOG_FIELDS)
        writer.writeheader()

    try:
        while time.time() < deadline:
            frame = _capture_frame(camera)
            if frame is None:
                time.sleep(interval)
                continue
            scan = visual_scan.free_space_scan(frame, geom)
            dist = estimate_target_distance(target_bearing_deg, scan)
            arrived = is_arrived(dist, **arrived_kwargs)
            if arrived:
                if writer:
                    dbg = controller.debug()
                    writer.writerow({
                        "t": round(time.time() - start, 3),
                        "distances": ";".join(f"{x:.2f}" for x in scan["distances"]),
                        "motor_l": 0.0, "motor_r": 0.0,
                        **dbg,
                        "target_bearing_deg": round(target_bearing_deg, 1),
                        "ray_distance_m": "" if dist is None else round(dist, 3),
                        "arrived": True,
                    })
                robot.stop()
                return True
            l, r = controller.step(scan["distances"], target_bearing_deg)
            if writer:
                dbg = controller.debug()
                writer.writerow({
                    "t": round(time.time() - start, 3),
                    "distances": ";".join(f"{x:.2f}" for x in scan["distances"]),
                    "motor_l": round(l, 3), "motor_r": round(r, 3),
                    **dbg,
                    "target_bearing_deg": round(target_bearing_deg, 1),
                    "ray_distance_m": "" if dist is None else round(dist, 3),
                    "arrived": False,
                })
            robot.set_motors(l, r)
            time.sleep(interval)
        robot.stop()
        return False
    finally:
        if log_file:
            log_file.close()
            print(f"[build] nav telemetry written to {log_path} -- replay "
                  f"offline with: jetbot_nav.gap_follow.replay_log('{log_path}')")


def _hazard_outside_window(scan, exempt_bearings_deg, window_deg,
                            hazard_forward_m=CREEP_HAZARD_FORWARD_M):
    """
    True if some ray OUTSIDE +/-window_deg of every bearing in
    exempt_bearings_deg reads close enough to be a genuine, distinct hazard
    during creep_to_target()'s final approach -- e.g. the other robot
    crossing in. Rays inside any exempt window are excluded on purpose.

    exempt_bearings_deg is a LIST, not just the one bearing being walked
    toward: live telemetry (nav_logs/robot_a_creep_pickup0_20260821_172359.csv)
    showed the robot getting wedged between the scene's two red blocks --
    the block NOT being approached was just as close as the one that was,
    so excluding only the tracked target's bearing left every other ray
    reading a "hazard" and the robot could never move. A same-coloured
    block is a known, static, low obstacle -- not the safety-critical case,
    which is the other ROBOT -- so every currently-visible same-coloured
    blob's bearing is exempted, not just the one currently being tracked.

    hazard_forward_m defaults to CREEP_HAZARD_FORWARD_M rather than reusing
    gap_follow.EMERGENCY_FORWARD -- see that constant's comment. Lateral
    cutoff still reuses gap_follow.CORRIDOR_HALF, since the chassis's actual
    width doesn't change with speed.
    """
    for a, d in zip(scan["angles_deg"], scan["distances"]):
        if any(abs(a - b) <= window_deg for b in exempt_bearings_deg):
            continue
        if d >= scan.get("max_range", float("inf")):
            continue
        rad = math.radians(a)
        lateral = d * math.sin(rad)
        forward = d * math.cos(rad)
        if abs(lateral) <= gap_follow.CORRIDOR_HALF and 0 < forward < hazard_forward_m:
            return True
    return False


def _creep_steer(bearing_deg):
    """Simple proportional steer toward bearing_deg at CREEP_SPEED -- not
    gap_follow's tuned arcade mix, just enough control to track a nearby
    target for the last ~1m.

    turn is scaled BY CREEP_SPEED (mirroring gap_follow._steer), not left as
    a raw 0-1 value -- an earlier version used abs(s) directly, which could
    reach 1.0 and completely dominate CREEP_SPEED (0.18), turning "steer
    toward the target while creeping" into "pivot in place, no forward
    progress." See CREEP_MIN_TURN's comment for the telemetry that caught
    this (distance stuck flat for a full 8-second creep attempt)."""
    t = math.radians(bearing_deg)
    s = max(-1.0, min(1.0, CREEP_TURN_GAIN * t))
    turn = max(abs(s) * CREEP_SPEED, CREEP_MIN_TURN) if abs(s) > 1e-6 else 0.0
    turn = turn if s >= 0 else -turn
    left = max(-1.0, min(1.0, CREEP_SPEED + turn))
    right = max(-1.0, min(1.0, CREEP_SPEED - turn))
    return left, right


def _blob_floor_distance(blob, geom):
    """
    Floor-projection distance for one blob -- the same technique
    block_sensing.sense_blocks() uses (project the blob's own bottom-center
    pixel onto the floor plane), independent of the proximity-scan's ray
    spacing. Returns None if the pixel doesn't project onto the floor
    ahead (same "no information" contract as CameraGeometry.pixel_to_ground
    and estimate_target_distance).
    """
    projected = geom.pixel_to_ground((blob["x1"] + blob["x2"]) / 2.0, blob["y2"])
    return None if projected is None else projected[1]


def creep_to_target(robot, camera, geom, color, arrival_distance,
                     expected_bearing_deg=None,
                     timeout_s=CREEP_TIMEOUT_S, poll_hz=POLL_HZ, log_path=None):
    """
    Close the final stretch to a `color`-colored target -- a block for
    pickup, the build marker for placement -- after navigate_to_bearing()
    has already gotten within its own safe ARRIVAL_DISTANCE using the full
    gap_follow avoidance stack.

    WHY THIS EXISTS, NOT JUST A TIGHTER navigate_to_bearing() CALL
    ────────────────────────────────────────────────────────────────
    gap_follow's avoidance has no way to tell "the thing I'm deliberately
    walking up to" apart from "an obstacle in my way." Live telemetry
    (nav_logs/robot_a_pickup0_20260821_140529.csv) showed EMERGENCY_FORWARD
    firing on the very first tick of the approach, 0.98m from the block,
    cascading into a BACKUP/SEARCH/FORWARD limit cycle that ended farther
    from the block than it started -- passing PICKUP_ARRIVAL_DISTANCE_M
    straight into navigate_to_bearing() does not work. This function
    instead re-senses the target's bearing fresh every tick (never frozen,
    unlike navigate_to_bearing()'s fixed target_bearing_deg -- staleness
    after a recovery maneuver was the second failure mode that log showed)
    and steers straight at it, treating only rays OUTSIDE a window around
    that bearing as real hazards (see _hazard_outside_window) -- so the
    "never collide" guarantee still holds for anything else nearby, most
    importantly the other robot, but the creep target itself no longer
    self-triggers a stop.

    WHICH BLOB, WHEN THERE'S MORE THAN ONE THIS COLOR
    ────────────────────────────────────────────────────────────────
    The scene has two red blocks roughly 1.4m apart -- close enough that
    once the robot is near one, the other is often also in frame and close.
    An earlier version of this function called perception.largest_blob()
    fresh every tick, which simply returns whichever same-colored blob is
    BIGGEST in the current frame -- with no memory of which one it was
    actually walking toward. Live telemetry
    (nav_logs/robot_a_creep_pickup0_20260821_141439.csv) caught this
    directly: target_bearing_deg jumped from 17.6 to 7.9 and ray_distance_m
    from 1.09m to 5.48m in a single tick -- a silent handoff to the OTHER
    red block -- which then made the (correct, working-as-designed) hazard
    check trip on the real block it had abandoned mid-approach.
    Fixed by tracking continuity instead: every tick, look at ALL blobs of
    `color` (perception.detect_color_blobs(), not largest_blob()) and keep
    whichever one has the closest RANGE (via estimate_target_distance, not
    bearing -- see CREEP_MAX_DISTANCE_JUMP_M's comment for why range is the
    more trustworthy continuity signal at close range) to where the target
    was LAST measured. expected_bearing_deg seeds which blob to start on --
    pass the bearing navigate_to_bearing() was aiming for, so creep starts
    locked onto the same block Phase 1 approached rather than picking
    fresh.

    Returns True once arrived (per target_seek.is_arrived at
    arrival_distance), False if the target is lost from view, a real hazard
    is detected, or timeout_s elapses.
    """
    deadline = time.time() + timeout_s
    interval = 1.0 / poll_hz
    last_bearing = expected_bearing_deg
    last_distance = None
    last_blob = None
    lost_ticks = 0

    log_file = writer = None
    start = time.time()
    if log_path:
        log_file = open(log_path, "w", newline="")
        writer = csv.DictWriter(log_file, fieldnames=CREEP_LOG_FIELDS)
        writer.writeheader()

    def _log(t_val, distances, motor_l, motor_r, bearing, dist, arrived, hazard, blob_seen):
        if not writer:
            return
        writer.writerow({
            "t": round(t_val, 3),
            "distances": ";".join(f"{x:.2f}" for x in distances),
            "motor_l": round(motor_l, 3), "motor_r": round(motor_r, 3),
            "target_bearing_deg": "" if bearing is None else round(bearing, 1),
            "ray_distance_m": "" if dist is None else round(dist, 3),
            "arrived": arrived, "hazard": hazard, "blob_seen": blob_seen,
        })

    try:
        while time.time() < deadline:
            frame = _capture_frame(camera)
            if frame is None:
                time.sleep(interval)
                continue
            blobs = perception.detect_color_blobs(frame, color)
            scan = visual_scan.free_space_scan(frame, geom)

            candidates = [
                (bearing_from_blob(b, geom.hfov_deg, image_width=geom.width), b)
                for b in blobs
            ]
            scored = [(b, blob, estimate_target_distance(b, scan)) for b, blob in candidates]
            valid = [s for s in scored if s[2] is not None]

            rejected = False
            if last_distance is None:
                # First tick: no range history yet, so pick by bearing --
                # nearest the Phase-1 seed if given, else just the biggest.
                if expected_bearing_deg is not None and candidates:
                    bearing, blob = min(candidates, key=lambda c: abs(c[0] - expected_bearing_deg))
                    dist = estimate_target_distance(bearing, scan)
                    if dist is None:
                        dist = _blob_floor_distance(blob, geom)
                elif candidates:
                    bearing, blob = max(candidates, key=lambda c: c[1]["area"])
                    dist = estimate_target_distance(bearing, scan)
                    if dist is None:
                        dist = _blob_floor_distance(blob, geom)
                else:
                    bearing, blob, dist = None, None, None
            elif valid:
                bearing, blob, dist = min(valid, key=lambda s: abs(s[2] - last_distance))
                rejected = abs(dist - last_distance) > CREEP_MAX_DISTANCE_JUMP_M
            elif candidates:
                # No ray gave a usable distance for ANY blob this tick --
                # at only 13 rays across the fan (~6.25 deg apart), a
                # correctly-sized small block can fall entirely between two
                # of them (confirmed via nav_logs telemetry: bearing/
                # distance frozen for CREEP_LOST_HOLD_TICKS+1 ticks while
                # the raw scan's neighboring rays, and the color blob
                # itself, kept reading real, closing distances the whole
                # time). Fall back to a floor-projection distance
                # (_blob_floor_distance, the same technique
                # block_sensing.sense_blocks() trusts) -- but ONLY for the
                # single candidate nearest in BEARING to the one already
                # being tracked, not indiscriminately for every blob in
                # frame. An earlier version of this fallback scored ALL
                # candidates this way, which let the nearest-DISTANCE
                # continuity match below lock onto a completely different,
                # unrelated blob whose floor-projected distance happened to
                # be close to last_distance (bearing jumped from ~4 deg to
                # ~24 deg mid-approach) -- the same "wrong blob" failure
                # mode Fix #2/#5 (see this function's own docstring) had
                # already solved for the ray-based path, reintroduced by
                # widening the pool of "valid" candidates without a
                # bearing gate. Restricting the fallback to the
                # already-most-plausible candidate, and still applying the
                # same jump-rejection check as the normal path, closes that
                # gap without reopening the older one.
                fallback_bearing, fallback_blob = min(
                    candidates, key=lambda c: abs(c[0] - last_bearing))
                fallback_dist = _blob_floor_distance(fallback_blob, geom)
                if (fallback_dist is not None
                        and abs(fallback_dist - last_distance) <= CREEP_MAX_DISTANCE_JUMP_M):
                    bearing, blob, dist = fallback_bearing, fallback_blob, fallback_dist
                else:
                    bearing, blob, dist = None, None, None
                    rejected = True
            else:
                bearing, blob, dist = None, None, None
                rejected = True

            if rejected or bearing is None:
                lost_ticks += 1
                if lost_ticks > CREEP_LOST_HOLD_TICKS:
                    _log(time.time() - start, scan["distances"], 0.0, 0.0,
                         None, None, False, False, False)
                    robot.stop()
                    return False
                bearing, dist = last_bearing, last_distance   # hold
                blob = last_blob
            else:
                lost_ticks = 0
                last_bearing, last_distance, last_blob = bearing, dist, blob

            # blob (from THIS tick's own detection, not a stale ray-scan
            # match) is passed as a secondary arrival signal: is_arrived()'s
            # blob-height fallback catches the case where the ray-based
            # distance estimate is unreliable this close in (its "nearest
            # ray to bearing" lookup can grab a different nearby feature's
            # range at these angles -- see CREEP_MAX_DISTANCE_JUMP_M's
            # comment for the telemetry this was found from), while the
            # tracked blob's own size in-frame keeps growing correctly
            # regardless.
            arrived = is_arrived(dist, blob=blob, image_height=geom.height,
                                  arrival_distance=arrival_distance,
                                  bearing_deg=bearing,
                                  max_bearing_deg=CREEP_ARRIVAL_MAX_BEARING_DEG)
            # Always exempt the bearing actually being steered at (even a
            # held one, on a rejected/blob-less tick) alongside every other
            # currently-visible same-coloured blob.
            all_bearings = list({bearing, *[c[0] for c in candidates]})
            hazard = _hazard_outside_window(scan, all_bearings, CREEP_TARGET_EXCLUSION_DEG)

            if arrived or hazard:
                _log(time.time() - start, scan["distances"], 0.0, 0.0,
                     bearing, dist, arrived, hazard, True)
                robot.stop()
                return arrived

            l, r = _creep_steer(bearing)
            _log(time.time() - start, scan["distances"], l, r,
                 bearing, dist, False, False, True)
            robot.set_motors(l, r)
            time.sleep(interval)
        robot.stop()
        return False
    finally:
        if log_file:
            log_file.close()
            print(f"[build] creep telemetry written to {log_path}")


def sense_and_sort_blocks(camera, geom):
    frame = _capture_frame(camera)
    if frame is None:
        return []
    return block_sensing.sense_blocks(frame, geom, color="red",
                                       max_radius_m=SENSE_RADIUS_M)


# sense_and_sort_blocks() is a single one-shot capture on purpose -- callers
# that already know roughly where to look (re-sensing mid-approach, e.g.)
# want exactly that, not the robot spinning underneath them. But the
# COLD-START case in build_tower() below has no such prior: nothing says
# a block starts inside the camera's narrow FOV (block_sensing.py's own
# frame-edge-clipping guard now correctly refuses to trust one that's only
# partly in view, rather than silently mis-projecting its distance -- see
# that module's docstring), so a spawn heading with every block outside or
# at the FOV's edge would otherwise make build_tower() give up on attempt
# zero having never actually looked around. gap_follow already has prior
# art for "rotate in place until something usable is found" (its own
# SEARCH state, for obstacle recovery) -- this is that same idea applied
# to the initial "is there anything to pick up at all" question.
BLOCK_SEARCH_TURN_SPEED = 0.4   # matches the turn rate used to calibrate
                                 # this session's manual grab tests (~35
                                 # deg/s differential-drive in this sim)
BLOCK_SEARCH_STEP_S     = 1.0   # ~35 deg per step at BLOCK_SEARCH_TURN_SPEED
BLOCK_SEARCH_MAX_STEPS  = 12    # >360 deg of total coverage, with margin
                                 # for the rate estimate being approximate


def sense_and_sort_blocks_with_search(robot, camera, geom):
    """
    Like sense_and_sort_blocks(), but if nothing is sensed from the
    current heading, rotates in place in small steps and re-senses between
    each, up to a bit more than one full rotation, before giving up.
    """
    for _ in range(BLOCK_SEARCH_MAX_STEPS):
        sensed = sense_and_sort_blocks(camera, geom)
        if sensed:
            return sensed
        robot.set_motors(BLOCK_SEARCH_TURN_SPEED, -BLOCK_SEARCH_TURN_SPEED)
        time.sleep(BLOCK_SEARCH_STEP_S)
        robot.stop()
        time.sleep(0.2)   # let the camera settle before the next capture
    return []


def find_build_marker(camera, geom):
    """
    Returns (bearing_deg, distance_m) to the build marker, or (None, None)
    if it isn't currently visible. distance_m comes from the same
    proximity-scan-ray lookup target_seek uses for blocks — see
    target_seek.estimate_target_distance.
    """
    frame = _capture_frame(camera)
    if frame is None:
        return None, None
    blob = perception.largest_blob(frame, BUILD_MARKER_COLOR)
    if blob is None:
        return None, None
    bearing = bearing_from_blob(blob, geom.hfov_deg, image_width=geom.width)
    scan = visual_scan.free_space_scan(frame, geom)
    distance = estimate_target_distance(bearing, scan)
    return bearing, distance


def _marker_pose_proxy(distance_m):
    """
    robot_coord.Coordinator's wait_for_zone_clearance() compares two
    {"x","y"} poses against a meeting_point by Euclidean distance — built
    for a shared world coordinate frame this camera-only robot doesn't
    have (see module docstring). Reusing that interface for a SENSED
    scalar distance-to-marker, rather than reworking Coordinator, is a
    deliberate 1-D special case: put the sensed distance on the x-axis,
    pin y at 0, and pin meeting_point at the origin, so the Euclidean
    distance Coordinator computes collapses to exactly
    abs(distance_m) — the number that actually means something here.
    """
    if distance_m is None:
        # Not currently visible -- report "far away" so pacing/zone logic
        # doesn't mistake "can't see the marker" for "at the marker."
        return {"x": 1e6, "y": 0.0}
    return {"x": float(distance_m), "y": 0.0}


def current_tower_height(coord: Coordinator) -> float:
    """
    The tallest tower_height_m either robot has published so far — max(),
    not sum(), because both numbers describe the SAME shared tower's
    current top, not two totals to add together. Taking the max is what
    keeps the two robots' bookkeeping from diverging if one is a step
    behind the other's last publish.
    """
    heights = [0.0]
    for rid in (coord.robot_id, coord.other_robot_id()):
        if not rid:
            continue
        state = coord.world.get_state(rid)
        if state and state.get("current_action"):
            heights.append(state["current_action"].get("tower_height_m", 0.0))
    return max(heights)


def build_tower(robot, camera, geom, controller, coord: Coordinator):
    blocks_placed = 0
    tower_height = 0.0
    idle_pose = {"x": 0.0, "y": 0.0, "heading_deg": 0.0}

    coord.publish_state("idle", pose=idle_pose, blocks_placed=blocks_placed,
                         current_action={"tower_height_m": tower_height})

    for attempt in range(MAX_BLOCKS):
        sensed = sense_and_sort_blocks_with_search(robot, camera, geom)
        if not sensed:
            print("[build] no blocks sensed within range even after "
                  "searching -- stopping (either done, or none currently "
                  "visible)")
            break

        target = sensed[0]   # largest footprint first
        print(f"[build] targeting block: bearing={target['bearing_deg']:.1f} deg "
              f"distance={target['distance_m']:.2f} m width={target['width_m']:.3f} m "
              f"height={target['height_m']:.3f} m")

        coord.publish_state("moving_to_pick", pose=idle_pose, blocks_placed=blocks_placed,
                             current_action={"tower_height_m": tower_height})

        # Reactive avoidance (this call) is the hard guarantee; there is
        # no cooperative wait before picking, since two robots picking up
        # two different blocks at once is not a collision risk by itself.
        if not navigate_to_bearing(robot, controller, camera, geom, target["bearing_deg"],
                                    log_path=_nav_log_path(coord, f"pickup{attempt}")):
            print("[build] could not reach the block in time -- skipping this cycle")
            continue

        # Final stretch down to actual grab range -- deliberately NOT more
        # of navigate_to_bearing()/gap_follow (see PICKUP_ARRIVAL_DISTANCE_M's
        # comment for why that doesn't work this close).
        if not creep_to_target(robot, camera, geom, color="red",
                                arrival_distance=PICKUP_ARRIVAL_DISTANCE_M,
                                expected_bearing_deg=target["bearing_deg"],
                                log_path=_nav_log_path(coord, f"creep_pickup{attempt}")):
            print("[build] could not creep within grab range -- skipping this block")
            continue

        if not arm_ops.reach_and_grab(base_yaw_deg=0.0):
            print("[build] grab not confirmed -- skipping this block")
            continue
        arm_ops.stow_for_transport()

        next_count = blocks_placed + 1
        coord.publish_state("moving_to_place", pose=idle_pose, blocks_placed=blocks_placed,
                             current_action={"tower_height_m": tower_height})

        # Cooperative layer 1: don't even start toward the build zone more
        # than MAX_SLACK blocks ahead of the other robot.
        if not coord.wait_for_pacing_clearance(next_count):
            print("[build] pacing wait timed out -- other robot may be stalled; "
                  "proceeding under reactive avoidance only")

        marker_bearing, marker_distance = find_build_marker(camera, geom)
        if marker_bearing is None:
            print("[build] build marker not visible -- cannot navigate to the "
                  "build zone this cycle, holding the block and retrying")
            # Retry a few times before giving up on this block outright.
            for _ in range(int(NAV_TIMEOUT_S * POLL_HZ)):
                time.sleep(1.0 / POLL_HZ)
                marker_bearing, marker_distance = find_build_marker(camera, geom)
                if marker_bearing is not None:
                    break
            if marker_bearing is None:
                print("[build] still no marker -- aborting this block")
                continue

        # Reactive avoidance (this call) is again what actually prevents a
        # collision approaching the shared build zone.
        navigate_to_bearing(robot, controller, camera, geom, marker_bearing,
                             log_path=_nav_log_path(coord, f"place{attempt}"))

        # Final stretch to actual placement range -- same reasoning as the
        # pickup creep above, targeting the marker's colour instead.
        if not creep_to_target(robot, camera, geom, color=BUILD_MARKER_COLOR,
                                arrival_distance=PICKUP_ARRIVAL_DISTANCE_M,
                                expected_bearing_deg=marker_bearing,
                                log_path=_nav_log_path(coord, f"creep_place{attempt}")):
            print("[build] could not creep to placement range -- placing at current "
                  "distance anyway")

        # Cooperative layer 2: don't enter the marker's exclusion radius
        # while the other robot is already in it, and don't approach to
        # place while the other robot hasn't signaled clear yet.
        _, marker_distance = find_build_marker(camera, geom)
        if not coord.wait_for_zone_clearance(_marker_pose_proxy(marker_distance)):
            print("[build] zone wait timed out -- proceeding under reactive avoidance only")
        if not coord.wait_for_placement_clear():
            print("[build] placement-clear wait timed out -- proceeding under reactive avoidance only")

        tower_height = current_tower_height(coord)
        if not arm_ops.place_at_height(base_yaw_deg=0.0, stack_height_m=tower_height):
            print("[build] release not confirmed -- treating the block as placed anyway")

        tower_height += target["height_m"]
        blocks_placed = next_count

        coord.publish_state("placed_clear", pose=idle_pose, blocks_placed=blocks_placed,
                             current_action={"tower_height_m": tower_height})
        print(f"[build] placed block {blocks_placed}, tower height now {tower_height:.3f} m")

    coord.publish_state("idle", pose=idle_pose, blocks_placed=blocks_placed,
                         current_action={"tower_height_m": tower_height})
    print(f"[build] done -- {blocks_placed} block(s) placed, final tower height {tower_height:.3f} m")


def main():
    robot = Robot()
    camera = jetson_utils.videoSource("csi://0")
    geom = visual_scan.sim_jetank()   # swap for a hardware-measured CameraGeometry on the real robot
    controller = SeekingGapFollowController(max_range=12.0)

    coord = Coordinator(meeting_point={"x": 0.0, "y": 0.0}, max_slack=MAX_SLACK,
                         exclusion_radius=EXCLUSION_RADIUS_M)
    print(f"[build] connecting as {coord.robot_id}...")
    coord.connect()

    try:
        build_tower(robot, camera, geom, controller, coord)
    except KeyboardInterrupt:
        print("\n[build] interrupted")
    finally:
        robot.stop()
        coord.disconnect()


if __name__ == "__main__":
    main()
