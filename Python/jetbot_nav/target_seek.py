"""
jetbot_nav.target_seek — steer gap_follow toward a color-detected item.

WHY BIAS GAP SELECTION INSTEAD OF BLENDING TWO STEERING VALUES
────────────────────────────────────────────────────────────────
The obvious approach is: compute a "seek" steering value from the target's
bearing, compute gap_follow's own steering value, average them by some
obstacle-closeness weight. That was rejected. A blended value is a THIRD
number that neither controller ever validated — gap_follow proved a given
steering angle is safe by finding it inside an actual free gap; averaging
that safe angle with an unrelated "point at the target" angle can walk the
result back out of the gap it came from, especially near a gap's edge.

Instead, SeekingGapFollowController changes only WHICH gap gap_follow's
existing, already-validated selection logic prefers: gaps are still found
by the same corridor/threshold rules, the committed-direction hysteresis
that kills oscillation still runs unchanged — the one thing that moves is
the "distance from center" used to rank candidate gaps, which becomes
"distance from the target's bearing" instead of "distance from 0 deg".
The robot can only ever be steered at a gap gap_follow already agrees is
safe to drive through. When PIVOT, BACKUP, or SEARCH are active, target
bias is fully suppressed (returned to None) — those states exist because
gap_follow decided normal driving logic doesn't apply right now, and a
recovery maneuver aimed at a shopping-list target instead of at getting
unstuck is exactly the kind of interaction bug this project's regression
history warns about.

DETECTION SOURCE: jetbot_nav.perception, NOT SimCamera's detect_objects
────────────────────────────────────────────────────────────────
This module was originally written against Unity's SimCamera/detect_objects
(single fixed class, fed through the pycuda/tensorrt simulation shim) but
that pipeline exists only so an unrelated, pre-existing detect_trt.py-style
YOLO script can run unchanged against the sim — it was never meant to be
called directly by a navigation module, and it reports exactly one hardcoded
class. jetbot_nav.perception.largest_blob() is the correct source here: it's
plain OpenCV color thresholding on a raw RGB frame from
jetson_utils.videoSource(...).Capture(), which is why (per its own
docstring) "it runs unmodified on hardware" — no Unity-specific data, no
fake-CUDA plumbing, and it distinguishes items BY COLOR rather than a single
class, which is what lets a future multi-item pass tell targets apart.

A perception blob looks like: {"x1","y1","x2","y2","cx","cy","area"}, in
the camera's ACTUAL frame pixel space (e.g. 640x480 from jetson_utils'
videoSource, though real hardware resolution may differ — see
bearing_from_blob()). This is a different pixel space than SimCamera's
fixed 640x640 detect_objects() output, which is why bearing conversion
here takes image_width/image_height as explicit parameters rather than a
hardcoded constant.

THE DISTANCE PROBLEM: NO CALIBRATED BLOB-SIZE-TO-DISTANCE CURVE
────────────────────────────────────────────────────────────────
A color blob's pixel area depends on both distance AND the target's real
size — two colored items of different physical size would need separate
calibration curves, which breaks the moment a new item color/size is
added. Rather than build that, estimate_target_distance() reuses the
proximity fan gap_follow already trusts: convert the blob's center to a
bearing angle from the camera's horizontal FOV, then look up the scan ray
nearest that bearing. If the target's bearing falls inside the 120-degree
fan and within RAY_MATCH_TOLERANCE_DEG of an actual ray, that ray's
distance IS the target's distance (assuming the item is the nearest thing
on that bearing, which holds once the robot is facing it). Outside the
fan, or before the item is faced, distance is simply None — the caller
doesn't need it yet, since the immediate job is just to steer onto the
target's bearing.

LOSS OF DETECTION: HOLD, DON'T SNAP
────────────────────────────────────────────────────────────────
A camera can drop a detection for a frame or two (motion blur, the item
straddling the frame edge, a lighting flicker affecting the color mask)
without the target having actually moved. TargetTracker holds the
last-known bearing for LOST_HOLD_TICKS before reporting "no target" —
matching the same debounce-over-snap-decisions principle used throughout
gap_follow's own state machine.

TESTING
    py test_target_seek.py   — offline, synthetic blob + scan streams,
    no Unity required, same pattern as test_gap_logic.py.
"""

import math

from jetbot_nav.gap_follow import (
    GapFollowController,
    FORWARD, SEARCH, BACKUP, PIVOT,
    STOP_FORWARD, FULL_CLEAR_FRACTION,
    SIGN_SWITCH_MARGIN_DEG, SIGN_SWITCH_DEBOUNCE_TICKS,
)

# ─── Tuning constants ────────────────────────────────────────────────────────

FAST_PATH_CONE_MARGIN_DEG = 15.0  # how far either side of the swath between
                                 # straight-ahead and the target bearing must
                                 # also be clear before the fast path steers
                                 # straight at the target. The robot is not a
                                 # point and does not pivot instantly, so
                                 # checking only the exact bearings it passes
                                 # through would clip whatever sits beside
                                 # them.

FAST_PATH_CLEARANCE_FRACTION = FULL_CLEAR_FRACTION   # how far that arc must
                                 # be clear, as a fraction of max_range.
                                 #
                                 # UNCHANGED from the whole-scan test this
                                 # replaces, and lowering it was tried and is
                                 # a dead end. Every value from 6.0 up to
                                 # 10.0 fails the offline slalom the same
                                 # way — the robot stops making forward
                                 # progress (z=5 against a bar of 20) — while
                                 # 10.8 passes.
                                 #
                                 # The reason is not safety, it is speed:
                                 # _steer scales speed by (1 - 0.5*|turn|),
                                 # so the more often the fast path fires the
                                 # harder the robot turns and the less ground
                                 # it covers. A course error that sits near
                                 # MAX_CORRECTION_DEG asks for a hard turn on
                                 # every tick, and the robot converges on its
                                 # heading by creeping. Firing the fast path
                                 # more often makes that worse, not better.
                                 #
                                 # So the fast path is not the lever for
                                 # getting course keeping to steer inside an
                                 # enclosed arena. Leaving the bar where it
                                 # was keeps this change to what it is: a
                                 # correction of WHICH rays are consulted.

LOST_HOLD_TICKS         = 5     # keep steering at the last-known bearing this
                                 # many ticks after detection drops, before
                                 # admitting the target is actually gone
RAY_MATCH_TOLERANCE_DEG = 6.0   # how close a target bearing must be to an
                                 # actual scan ray to trust that ray's
                                 # distance as the target's distance (half a
                                 # ray-spacing at 13 rays / 120 deg == 5 deg,
                                 # +1 deg slack for interpolation error)
ARRIVAL_DISTANCE        = 1.2   # ray-based distance below this -> arrived
ARRIVAL_BLOB_FRACTION   = 0.55  # fallback: blob height / image height above
                                 # this -> arrived, for when the target sits
                                 # just outside the fan's edge but is
                                 # obviously close up in the camera frame


# ─── Camera geometry ─────────────────────────────────────────────────────────


def bbox_to_bearing(bbox_center_x: float, image_width: float,
                     camera_fov_deg: float) -> float:
    """
    Convert a pixel x-coordinate to a bearing in the same DEGREES-negative-
    left convention gap_follow's scan uses, assuming a simple pinhole/
    linear-FOV camera (true for the sim's virtual camera, close enough for
    a physical webcam without a fisheye lens). General-purpose version —
    for a raw perception.largest_blob() result, use bearing_from_blob()
    below instead.
    """
    frac = (bbox_center_x / image_width) - 0.5   # -0.5 .. +0.5, left..right
    return frac * camera_fov_deg


def bearing_from_blob(blob: dict, camera_fov_deg: float,
                      image_width: float = None) -> float:
    """
    Convert one perception.largest_blob()/detect_color_blobs() result —
    {"x1","y1","x2","y2","cx","cy","area"} in the camera's actual frame
    pixel space — to a bearing. camera_fov_deg is the camera's horizontal
    field of view (not reported by perception.py itself, so still a
    parameter here). image_width defaults to 640 (jetson_utils' simulated
    videoSource resolution) — pass camera.GetWidth() explicitly if calling
    this against the real Jetson camera or a differently-configured sim
    capture, since a mismatched width silently skews every bearing.
    """
    width = image_width if image_width is not None else 640.0
    return bbox_to_bearing(blob["cx"], width, camera_fov_deg)


def blob_height_fraction(blob: dict, image_height: float = None) -> float:
    """Blob's bbox height as a fraction of the camera frame height — used
    by is_arrived()'s blob-size fallback. image_height defaults to 480
    (jetson_utils' simulated videoSource resolution) — pass
    camera.GetHeight() explicitly otherwise."""
    height = image_height if image_height is not None else 480.0
    return (blob["y2"] - blob["y1"]) / height


def estimate_target_distance(bearing_deg: float, scan: dict) -> float:
    """
    Look up the proximity-scan ray nearest `bearing_deg` and return its
    distance, or None if no ray is close enough to trust (target outside
    the fan, or between two rays by more than RAY_MATCH_TOLERANCE_DEG).
    `scan` is the same dict shape gap_follow.get_proximity_scan() returns:
    {"angles_deg": [...], "distances": [...], "max_range": m}.
    """
    angles = scan["angles_deg"]
    distances = scan["distances"]
    best_i, best_diff = None, None
    for i, a in enumerate(angles):
        diff = abs(a - bearing_deg)
        if best_diff is None or diff < best_diff:
            best_i, best_diff = i, diff
    if best_i is None or best_diff > RAY_MATCH_TOLERANCE_DEG:
        return None
    dist = distances[best_i]
    if dist >= scan.get("max_range", float("inf")):
        return None   # that ray sees nothing — can't confirm the target is there
    return dist


def is_arrived(target_distance: float = None, blob: dict = None,
               image_height: float = None,
               arrival_distance: float = ARRIVAL_DISTANCE,
               bearing_deg: float = None, max_bearing_deg: float = None) -> bool:
    """True if the target is close enough to consider the approach done,
    by whichever signal is available (ray distance preferred; blob-height
    fallback, via the raw perception.largest_blob() dict, for when the
    target sits just off the fan's edge and no ray distance was found).

    arrival_distance defaults to the general-purpose ARRIVAL_DISTANCE
    (tuned for obstacle-avoidance driving), but a caller with a tighter
    physical requirement -- e.g. getting within the arm's actual grab
    radius, which is well inside ARRIVAL_DISTANCE -- should pass its own
    value rather than this module's constant being changed for everyone.

    bearing_deg/max_bearing_deg: optional bearing-alignment gate, on top
    of the distance/blob-size check. A caller whose next step assumes the
    target is roughly straight ahead -- e.g. arm_ops.reach_and_grab()'s
    fixed base_yaw_deg=0.0, which has no way to compensate for a residual
    bearing -- needs both close AND roughly centered before treating the
    approach as done. Without this, a target sensed near the edge of the
    camera's view can read "arrived" purely on distance/blob-size while
    still tens of degrees off-center: confirmed via nav_logs telemetry
    showing arrived=True fire on creep_to_target()'s very first tick with
    a ~20-27 degree residual target_bearing_deg, every single retry, once
    a camera-geometry fix let the robot get close from a wider range of
    initial angles than before. Pass both to require |bearing_deg| <=
    max_bearing_deg in addition to the existing distance/blob check;
    leave either as None (the default) to skip this gate entirely, e.g.
    for navigate_to_bearing()'s looser, non-final arrival check."""
    if bearing_deg is not None and max_bearing_deg is not None:
        if abs(bearing_deg) > max_bearing_deg:
            return False
    if target_distance is not None and target_distance <= arrival_distance:
        return True
    if blob is not None:
        if blob_height_fraction(blob, image_height) >= ARRIVAL_BLOB_FRACTION:
            return True
    return False


# ─── Detection hold/debounce ─────────────────────────────────────────────────


class TargetTracker:
    """
    Wraps a raw per-tick detection (bearing_deg or None) and holds the
    last-known bearing across brief detection drops, so a single missed
    camera frame doesn't make the robot snap out of seeking and fall back
    to plain gap-following mid-approach.
    """

    def __init__(self, hold_ticks: int = LOST_HOLD_TICKS):
        self.hold_ticks = hold_ticks
        self._last_bearing = None
        self._ticks_since_seen = 0

    def update(self, bearing_deg: float = None) -> float:
        """Feed this tick's raw detection (or None if nothing detected).
        Returns the bearing to steer toward, or None if the target has
        been lost long enough to give up."""
        if bearing_deg is not None:
            self._last_bearing = bearing_deg
            self._ticks_since_seen = 0
            return self._last_bearing

        if self._last_bearing is None:
            return None

        self._ticks_since_seen += 1
        if self._ticks_since_seen > self.hold_ticks:
            self._last_bearing = None
            return None
        return self._last_bearing   # still within hold window

    def reset(self):
        self._last_bearing = None
        self._ticks_since_seen = 0


# ─── Controller ──────────────────────────────────────────────────────────────


class SeekingGapFollowController(GapFollowController):
    """
    GapFollowController whose gap selection is biased toward an optional
    target bearing. See module docstring for why this biases gap RANKING
    rather than blending motor outputs. Fully backward compatible: calling
    step(distances) with no target_bearing_deg behaves identically to the
    base class (same object, same regression suite should still pass
    unmodified against it).
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._target_bearing_deg = None   # this tick's active seek bearing

    def step(self, distances, target_bearing_deg: float = None) -> tuple:
        # Suppress seeking outright during any recovery state — those exist
        # because normal driving logic doesn't apply right now, and seeking
        # has no business overriding a maneuver aimed at getting unstuck.
        if self.state in (PIVOT, BACKUP, SEARCH):
            self._target_bearing_deg = None
        else:
            self._target_bearing_deg = target_bearing_deg

        # Base class's "totally open field" fast path hardcodes a straight
        # line (MAX_SPEED, MAX_SPEED) — correct with no target, wrong with
        # one, since open field is exactly when seeking should just turn
        # straight at the target with nothing else to consider. Intercept
        # only that specific case; every other path (obstacles present,
        # any recovery state) still runs through the unmodified base logic
        # via self._choose_gap, which IS overridden below.
        if self.state == FORWARD and self._target_bearing_deg is not None:
            d = [min(x, self.max_range) for x in distances]
            fwd_clear = self._forward_clearance(d)
            if (fwd_clear >= STOP_FORWARD
                    and self._swath_clear(d, self._target_bearing_deg)):
                self._target_deg = self._target_bearing_deg
                return self._steer(self._target_bearing_deg, fwd_clear)

        return super().step(distances)

    def _swath_clear(self, d, bearing: float) -> bool:
        """
        Is the arc the robot is about to sweep through open enough to steer
        across without consulting the gap logic?

        Two changes from the check this replaces, which asked whether EVERY
        ray was at least 0.9 * max_range.

        SCOPE. Only the arc between straight-ahead and the target bearing
        can be driven into, so only that arc bears on whether steering
        there is safe. The old test answered a much broader question — "is
        the robot in open country" — which is a different thing, and in an
        enclosed arena it is never true: measured over a drive, the nearest
        ray ran 3.8 to 8.9 units against a 10.8 bar while 12.0 (max range,
        nothing there) sat dead ahead, and the fast path fired on 1% of
        ticks. Everything fell through to gap ranking, where a single gap
        spanning the whole fan is steered at its MIDPOINT, so the course
        preference was not overruled — it was never read. Mean distance
        between the bearing asked for and the one steered was 39 deg.

        BAR. FAST_PATH_CLEARANCE rather than 0.9 * max_range, because the
        question is "does anything need avoiding yet", not "is the world
        empty". SLOW_FORWARD is where gap_follow itself starts reacting to
        an obstacle, so a swath clear beyond it needs no maneuvering, and
        the gate is re-checked every tick — the moment something comes
        closer, gap selection takes over again. Narrowing the scope alone
        was tried first and left the path firing on 1% of ticks; the bar
        was doing the blocking.
        """
        # Steering at a bearing the sensor cannot see means turning toward
        # something unobserved, which is exactly when the gap logic should
        # decide instead. Checking only the rays that fall inside the cone
        # is not enough on its own: a bearing of 200 deg yields a cone
        # containing the entire fan, every ray in it reads clear, and the
        # unseen arc the robot would actually turn through is never looked
        # at.
        if not (self.angles_deg[0] <= bearing <= self.angles_deg[-1]):
            return False

        lo = min(0.0, bearing) - FAST_PATH_CONE_MARGIN_DEG
        hi = max(0.0, bearing) + FAST_PATH_CONE_MARGIN_DEG

        in_cone = [dist for ang, dist in zip(self.angles_deg, d)
                   if lo <= ang <= hi]
        if not in_cone:
            return False

        # The margin may run off the end of the fan; what is left is still
        # checked. Deliberate rather than sloppy: the bearing itself is in
        # view, the robot turns gradually, and this is re-evaluated every
        # tick, so the unverified sliver past the edge is only entered
        # after another look at it.
        bar = FAST_PATH_CLEARANCE_FRACTION * self.max_range
        return all(x >= bar for x in in_cone)

    def _choose_gap(self, gaps):
        """Same committed-direction hysteresis as the base class — that
        logic is about not flip-flopping steering direction in the robot's
        own frame and is orthogonal to seeking — but gaps are ranked by
        distance from the TARGET bearing instead of distance from 0 deg
        when a target is active."""
        if self._target_bearing_deg is None:
            return super()._choose_gap(gaps)

        bearing = self._target_bearing_deg

        def rank(g):
            return (abs(g["center_deg"] - bearing), -g["width"])

        committed = 0
        if self._target_deg is not None and abs(self._target_deg) >= 5:
            committed = 1 if self._target_deg > 0 else -1

        if committed != 0:
            on_side = [g for g in gaps if g["center_deg"] * committed > 0]
            off_side = [g for g in gaps if g["center_deg"] * committed <= 0]

            best_on = min(on_side, key=rank) if on_side else None
            best_off = min(off_side, key=rank) if off_side else None

            if best_on is None:
                self._switch_streak = 0
                return best_off["center_deg"]
            if best_off is None:
                self._switch_streak = 0
                return best_on["center_deg"]
            if (abs(best_off["center_deg"] - bearing) + SIGN_SWITCH_MARGIN_DEG
                    < abs(best_on["center_deg"] - bearing)):
                self._switch_streak += 1
                if self._switch_streak >= SIGN_SWITCH_DEBOUNCE_TICKS:
                    return best_off["center_deg"]
                return best_on["center_deg"]
            self._switch_streak = 0
            return best_on["center_deg"]

        self._switch_streak = 0
        best = min(gaps, key=rank)
        return best["center_deg"]