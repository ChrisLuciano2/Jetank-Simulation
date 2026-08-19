"""
jetbot_nav.course_keep — hold a heading across obstacle detours.

THE PROBLEM
────────────────────────────────────────────────────────────────
gap_follow is very good at not hitting things and has no idea where it is
going. Every avoidance manoeuvre — a steer around a block, a PIVOT, a
BACKUP — leaves the robot pointing somewhere new, and it then drives
straight from that new heading forever. Over a course with three
obstacles the robot ends up travelling in an essentially random direction,
having behaved correctly at every individual step.

CourseKeeper closes that loop: remember the heading the robot was on,
let gap_follow do whatever avoidance requires, and steer back afterwards.

REUSE, NOT A NEW CONTROLLER
────────────────────────────────────────────────────────────────
"Get back to bearing X" is the same problem target_seek already solved
for "drive toward the red block": bias WHICH GAP gap_follow prefers,
rather than blending a second steering value into its output. That
distinction is load-bearing and target_seek's docstring argues it at
length — a blended value is a third number neither controller validated,
and it can steer the robot out of the very gap that was proven safe.

So this module owns no steering logic at all. It computes one number, the
course error, and hands the corresponding bearing to
SeekingGapFollowController. Everything that makes gap_follow safe —
corridor clearance, gap hysteresis, the PIVOT/BACKUP/SEARCH recovery
states, and the rule that target bias is SUPPRESSED during those states —
keeps working untouched. Course-keeping can never talk the robot into an
unsafe heading, only express a preference among headings gap_follow has
already accepted.

TWO HEADING SOURCES, AND WHY ONLY ONE IS USED
────────────────────────────────────────────────────────────────
From jetbot_nav.heading:

  - VisualGyro accumulates frame to frame. It tracks arbitrarily large
    rotations, which matters because a PIVOT can swing well past the
    ~28 deg a single snapshot comparison can measure. It drifts.
  - CourseLock compares the live frame against a photograph taken when
    the course was set, so its error cannot accumulate — but it only
    works within that ~28 deg window and needs texture in view.

The design was for the gyro to carry the robot through a big detour and
the lock's drift-free reading to re-anchor it whenever the original view
came back into sight. That reasoning is sound and the measurement does
not support it, so `use_lock` defaults to False and the gyro runs alone.

The premise that fails is "the lock compares against the original view".
It compares against the original PHOTOGRAPH, taken from the original
POSITION. Once the robot has driven a few units the scene genuinely no
longer matches, and correlation does not fail loudly — it settles on the
same wrong alignment every time, which is worse than noise because a
consistent error looks like a confident measurement.

Measured by driving one keeper and feeding a second the identical frames,
so the two differ only in whether a lock reading may re-anchor the gyro.
Six runs, 1007 ticks each side:

    lock ON    mean error 26.32 deg   worst 93.12   544 ticks over 20 deg
    lock OFF   mean error  4.67 deg   worst 21.28     2 ticks over 20 deg

Every individual run agreed, whichever keeper was driving. Closed-loop
final heading error was 79/76/56 deg driving with the lock against
35/9/15 without.

So the gyro's drift, which the lock exists to bound, is not currently the
limiting error — over a 28 second drive it holds to about 4.7 deg. Over a
much longer run it would be, and nothing here solves that. Doing so needs
a reference that survives translation: recognisable landmarks, or
re-capturing the reference with a known heading. Until then, bounded
runs.

CourseLock itself is not the problem and is left intact — it is accurate
for rotation in place, which is what its own tests cover. Set
use_lock=True to re-enable re-anchoring, and re-run the A/B above before
trusting it.

WHEN HEADING IS UNKNOWN, DO NOT GUESS
────────────────────────────────────────────────────────────────
Both sources return None on an untextured or unmatchable view. The robot
then reverts to plain gap-following: it keeps avoiding obstacles
correctly and simply stops trying to hold a course, which is the honest
degradation. Steering toward a fabricated heading would be worse than
not steering at all, and `state` reports UNKNOWN so a caller can tell the
difference between "on course" and "no idea".

USAGE
    from jetbot_nav import course_keep
    course_keep.drive_keeping_course(duration=30.0)

TESTING
    py -3.8 test_course_keep.py    — offline, closed-loop, no Unity
"""

import math
import time


def _wrap180(deg: float) -> float:
    """Signed angle in (-180, 180]. The gyro's accumulated heading is not
    bounded, so a raw subtraction against a wrapped reading can make two
    headings 17 deg apart look 343 deg apart."""
    return (deg + 180.0) % 360.0 - 180.0

from jetbot_nav.gap_follow import (
    FORWARD, SEARCH, BACKUP, PIVOT, MAX_SPEED,
)
from jetbot_nav.target_seek import SeekingGapFollowController

# ─── States (reported, not driven — gap_follow owns the real state) ──────────

ON_COURSE = "ON_COURSE"     # within tolerance of the remembered heading
OFF_COURSE = "OFF_COURSE"   # heading known, and we are working back to it
UNKNOWN = "UNKNOWN"         # no trustworthy heading; plain gap-following

# ─── Tuning ──────────────────────────────────────────────────────────────────

COURSE_TOLERANCE_DEG = 5.0   # inside this, consider ourselves on course.
                             # Deliberately looser than heading.ARRIVED_DEG:
                             # this is a moving robot with skid-steer yaw
                             # friction, not a stationary alignment, and
                             # demanding more just burns ticks micro-turning.

# ─── Trusting a drift-free reading ───────────────────────────────────────────
#
# CourseLock is the anchor: its error cannot accumulate, so when it answers
# its number replaces the gyro's running total outright. That is right when
# the reading is right, and it is how a single WRONG reading does more
# damage than anything else in this module — measured live, one bad lock at
# tick 144 of a drive moved the accumulated heading 73 deg the wrong way in
# a single step, and nothing afterwards could tell that it had happened.
#
# The gyro is the only second opinion available, and it is a good one over
# SHORT spans: about 0.7 deg of error per tick, drifting maybe a quarter of
# a degree per tick net. So a lock reading gets checked against it before
# being believed.
#
# The tolerance has to GROW with time since the last anchor, which is the
# whole difficulty. Correcting drift is the lock's job, so disagreement is
# expected and gets larger the longer the gyro has run unanchored — a fixed
# window would either reject genuine corrections or keep admitting the bad
# ones. Growing it means the check is tight exactly when the gyro is
# trustworthy and loose when it is not.

ANCHOR_TOLERANCE_BASE_DEG = 12.0   # disagreement allowed immediately after an
                                   # anchor, when the gyro has barely run.

ANCHOR_DRIFT_PER_TICK_DEG = 0.35   # how much further apart they are allowed to
                                   # drift per unanchored tick. Above the ~0.25
                                   # measured, deliberately: this is a
                                   # plausibility bound, and being too strict
                                   # rejects the corrections that keep the
                                   # course honest.

ANCHOR_TOLERANCE_MAX_DEG = 60.0    # never stretch further than this, or the
                                   # check stops being one.

# A disagreement does not prove the LOCK is wrong — the gyro may be the one
# that has come adrift, and then rejecting every correction would strand the
# robot on a heading it invented. So a persistent, self-consistent
# disagreement is believed after all.
ANCHOR_CONFIRM_TICKS = 3           # consecutive outlying readings needed
ANCHOR_CONFIRM_SPREAD_DEG = 8.0    # and how closely they must agree with each
                                   # other to count as corroboration rather
                                   # than noise

ANCHOR_CONFIRM_MIN_TICKS = 30      # ...and the gyro must have run at least this
                                   # long unanchored before corroboration is
                                   # allowed to overrule it.
                                   #
                                   # Without this the escape hatch defeats the
                                   # guard. CourseLock's failure once the robot
                                   # has driven away from its reference photo
                                   # is not noise — the view genuinely no
                                   # longer matches, so correlation settles on
                                   # the same wrong alignment every time, and a
                                   # systematically wrong reading CORROBORATES
                                   # ITSELF. Measured: a 46 deg error walked
                                   # straight through three consistent
                                   # readings.
                                   #
                                   # Consistency is not correctness. Requiring
                                   # the gyro to have been running long enough
                                   # for drift to be a plausible explanation is
                                   # what makes the override mean something: a
                                   # gyro anchored three ticks ago has not
                                   # drifted 46 deg, so a lock claiming it has
                                   # is wrong however firmly it insists.

MAX_STALE_TICKS = 3          # consecutive unmatched frames before the gyro's
                             # accumulated heading is treated as expired.
                             # Coasting through one or two is worth it — a
                             # single bad frame during an otherwise good turn
                             # should not throw the course away — but the
                             # accumulator has no way to catch up on the
                             # rotation it missed, so the error is permanent
                             # and grows with every tick spent pretending
                             # otherwise. Better to admit UNKNOWN and let
                             # CourseLock re-anchor when the view comes back.

MAX_CORRECTION_DEG = 35.0    # never ask for a bearing further off than this.
                             # The correction is a PREFERENCE among gaps, and
                             # asking for something outside the scan's fan
                             # just pins it to the outermost gap while
                             # discarding the information that it is extreme.


class CourseKeeper:
    """
    Wraps a SeekingGapFollowController with a remembered heading.

    Feed it a scan and (a frame, or an explicit heading) each tick; it
    returns motor commands exactly as GapFollowController.step() does.

    heading_deg may be supplied directly instead of a frame. That exists
    for tests — which need an exact, noise-free heading to check the
    control loop rather than the estimator, since the estimator has its
    own suite — and it is also the hook for a real IMU, should one ever
    be fitted.
    """

    def __init__(self, geom=None, controller=None,
                 tolerance_deg: float = COURSE_TOLERANCE_DEG,
                 use_lock: bool = False):
        self.geom = geom
        self.controller = controller or SeekingGapFollowController()
        self.tolerance_deg = tolerance_deg
        self.use_lock = use_lock

        self.state = UNKNOWN
        self.course_error_deg = None    # + means pointing RIGHT of course
        self.lock_used = False          # did the drift-free source answer?
        self.anchors_refused = 0        # lock readings the gyro contradicted

        self._gyro = None
        self._lock = None
        self._armed = False
        self._ticks_since_anchor = 0
        self._disputed = []             # recent refused readings, for the
                                        # corroboration rule

    # ── Course management ────────────────────────────────────────────────

    def set_course(self, frame=None) -> bool:
        """
        Adopt the current heading as the course to hold.

        With a frame, this photographs the view (CourseLock) and zeroes the
        gyro. Returns False if the view has too little texture to lock
        onto — the caller should know, because an unarmed keeper silently
        degrades to plain gap-following.

        Without a frame, the course is simply "whatever heading_deg is
        measured relative to", which is what the explicit-heading path
        already provides.
        """
        self.course_error_deg = None
        self.state = UNKNOWN

        if frame is None:
            self._armed = True
            return True

        from jetbot_nav.heading import VisualGyro, CourseLock
        if self._gyro is None:
            self._gyro = VisualGyro(self.geom)
            self._lock = CourseLock(self.geom)

        self._gyro.reset()
        self._gyro.update(frame)          # prime, so the next delta is real
        ok = self._lock.capture(frame)
        self._armed = ok
        return ok

    # ── Heading estimation ───────────────────────────────────────────────

    def _believable_anchor(self, locked: float) -> bool:
        """
        Should this drift-free reading replace the accumulated heading?

        Yes if the gyro roughly agrees, within a window that widens the
        longer the gyro has run unanchored. Yes anyway if several
        consecutive readings have disagreed in the SAME place, since that
        is the signature of a gyro that has come adrift rather than of a
        bad match. No otherwise — and "no" only costs this one correction,
        because the gyro keeps running and the next reading gets its own
        hearing.
        """
        gap = abs(_wrap180(locked - self._gyro.heading_deg))
        tolerance = min(
            ANCHOR_TOLERANCE_BASE_DEG
            + ANCHOR_DRIFT_PER_TICK_DEG * self._ticks_since_anchor,
            ANCHOR_TOLERANCE_MAX_DEG)

        if gap <= tolerance:
            self._disputed = []
            return True

        # Outlier. Keep it: if the next few agree with it AND the gyro has
        # run long enough for drift to explain the gap, the gyro is the one
        # that is wrong and this becomes the correction that rescues it.
        self._disputed.append(locked)
        recent = self._disputed[-ANCHOR_CONFIRM_TICKS:]
        if (self._ticks_since_anchor >= ANCHOR_CONFIRM_MIN_TICKS
                and len(recent) >= ANCHOR_CONFIRM_TICKS
                and max(recent) - min(recent) <= ANCHOR_CONFIRM_SPREAD_DEG):
            self._disputed = []
            return True

        self.anchors_refused += 1
        return False

    def _measure(self, frame, heading_deg, depth=None):
        """Course error in degrees (+ = right of course), or None."""
        if heading_deg is not None:
            self.lock_used = False
            return heading_deg

        if frame is None or not self._armed or self._gyro is None:
            return None

        # Always advance the gyro, even when the lock ends up answering:
        # skipping it would leave a gap in the accumulator the moment the
        # lock loses sight of the reference view.
        self._gyro.update(frame, depth=depth)
        self._ticks_since_anchor += 1

        locked = (self._lock.error_deg(frame, depth=depth)
                  if self.use_lock else None)
        if locked is not None and self._believable_anchor(locked):
            # Drift-free reading, and the gyro does not contradict it —
            # re-anchor the accumulator. This is the entire point of
            # running both sources, and the check above is what stops one
            # bad match from rewriting the heading in a single step.
            self._gyro.heading_deg = locked
            self._ticks_since_anchor = 0
            self.lock_used = True
            return locked

        self.lock_used = False
        if self._gyro.lost_frames > MAX_STALE_TICKS:
            # The accumulator has gone unupdated for long enough that it
            # describes a pose the robot has since driven away from. It is
            # not an imprecise heading, it is an OLD one, and steering on
            # it is the "fabricated heading" this module's docstring
            # promises never to act on. Report UNKNOWN and let the caller
            # degrade to plain gap-following.
            #
            # This branch used to return heading_deg exactly like the line
            # below it, i.e. it did nothing. That went unnoticed because
            # the gyro almost never lost frames on synthetic scenes; the
            # first live run sat on a frozen value for 15 consecutive
            # ticks while steering hard on it.
            return None
        return self._gyro.heading_deg

    # ── Per-tick ─────────────────────────────────────────────────────────

    def step(self, distances, frame=None, heading_deg: float = None) -> tuple:
        """
        One control tick. Returns (left_motor, right_motor).

        Identical in contract to GapFollowController.step(), so this is a
        drop-in for any loop already driving one.
        """
        # The scan already says how far away the scene is, so the lever-arm
        # correction costs nothing extra and no caller has to know about it.
        # The MEDIAN, not the nearest: one close obstacle in an otherwise
        # open view should not shrink the depth the whole correction is
        # scaled by, and it is the bulk of the view that the heading
        # correlation is reading.
        depth = None
        if distances:
            ordered = sorted(distances)
            depth = ordered[len(ordered) // 2]

        error = self._measure(frame, heading_deg, depth=depth)

        # Wrap before ANY use. VisualGyro accumulates without bound — it has
        # been measured past +360 on a half-minute drive — and a raw
        # accumulated heading is not a course error. Two things break if it
        # is used as one:
        #
        #   - the correction inverts. bearing = -error clamped, so an error
        #     of +183 asks for a hard LEFT when the robot is 177 deg to the
        #     LEFT of course and needs to go right. Measured before this
        #     wrap: 80 of 170 ticks had |error| > 180, and the robot steered
        #     toward its course on only 41% of the ticks it was off it —
        #     worse than choosing at random, while steering hard the whole
        #     time.
        #   - a robot exactly back on course after a full turn reads as 360
        #     deg off and never reports ON_COURSE.
        if error is not None:
            error = _wrap180(error)
        self.course_error_deg = error

        if error is None:
            self.state = UNKNOWN
            return self.controller.step(distances)

        self.state = (ON_COURSE if abs(error) <= self.tolerance_deg
                      else OFF_COURSE)

        # Steering toward the course means turning by the NEGATIVE of the
        # error: positive error is "pointing right of course", which needs
        # a left turn, and gap_follow's bearings are negative-left.
        bearing = -error
        bearing = max(-MAX_CORRECTION_DEG, min(MAX_CORRECTION_DEG, bearing))

        return self.controller.step(distances, target_bearing_deg=bearing)

    # ── Introspection ────────────────────────────────────────────────────

    def debug(self) -> dict:
        d = dict(self.controller.debug())
        d.update({
            "course_state": self.state,
            "course_error_deg": (None if self.course_error_deg is None
                                 else round(self.course_error_deg, 2)),
            "drift_free": self.lock_used,
            "anchors_refused": self.anchors_refused,
        })
        return d


# ─── Live driving loop ───────────────────────────────────────────────────────

def drive_keeping_course(duration: float = 20.0, poll_hz: float = 10.0,
                         verbose: bool = True):
    """
    Drive forward holding the starting heading, detouring around whatever
    is in the way and returning to course afterwards.

    Needs Unity playing with RobotCamera in the scene (Tools > Setup Robot
    Simulator Scene). Everything it uses — the camera-derived scan and the
    camera-derived heading — runs identically on the Jetson.
    """
    import jetson_utils
    from jetbot import Robot
    from jetbot_nav import visual_scan

    camera = jetson_utils.videoSource("csi://0")
    img = camera.Capture()
    if img is None:
        print("[course_keep] No camera frame. Is the sim playing with "
              "RobotCamera in the scene?")
        return
    frame = jetson_utils.cudaToNumpy(img)

    geom = visual_scan.sim_jetank(width=frame.shape[1], height=frame.shape[0])
    keeper = CourseKeeper(geom)

    if not keeper.set_course(frame):
        print("[course_keep] Could not lock onto the view ahead — too little "
              "texture. Driving with obstacle avoidance only; the robot will "
              "not hold a course.")

    robot = Robot()
    interval = 1.0 / poll_hz
    deadline = time.time() + duration

    try:
        while time.time() < deadline:
            img = camera.Capture()
            if img is None:
                time.sleep(interval)
                continue
            frame = jetson_utils.cudaToNumpy(img)

            scan = visual_scan.free_space_scan(frame, geom, n_rays=13)
            left, right = keeper.step(scan["distances"], frame=frame)
            robot.set_motors(left, right)

            if verbose:
                d = keeper.debug()
                err = d["course_error_deg"]
                print(f"  {d['state']:<7} {d['course_state']:<10} "
                      f"err={'  n/a' if err is None else f'{err:+6.1f}'} "
                      f"{'lock' if d['drift_free'] else 'gyro'}  "
                      f"motors=({left:+.2f},{right:+.2f})")

            time.sleep(interval)
    finally:
        robot.stop()
        print("[course_keep] stopped")
