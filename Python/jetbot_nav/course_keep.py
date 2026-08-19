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

TWO HEADING SOURCES, BECAUSE NEITHER IS ENOUGH ALONE
────────────────────────────────────────────────────────────────
From jetbot_nav.heading:

  - VisualGyro accumulates frame to frame. It tracks arbitrarily large
    rotations, which matters because a PIVOT can swing well past the
    ~28 deg a single snapshot comparison can measure. It drifts.
  - CourseLock compares the live frame against a photograph taken when
    the course was set, so its error cannot accumulate — but it only
    works within that ~28 deg window and needs texture in view.

Used together they cover each other: the gyro carries the robot through
a big detour, and whenever the lock CAN see the original view its
drift-free reading replaces the accumulated one. That re-anchoring is the
whole reason to run both; the gyro alone would slowly walk the "course"
somewhere else, and the lock alone would go blind exactly when a large
recovery manoeuvre needed it most.

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
                 tolerance_deg: float = COURSE_TOLERANCE_DEG):
        self.geom = geom
        self.controller = controller or SeekingGapFollowController()
        self.tolerance_deg = tolerance_deg

        self.state = UNKNOWN
        self.course_error_deg = None    # + means pointing RIGHT of course
        self.lock_used = False          # did the drift-free source answer?

        self._gyro = None
        self._lock = None
        self._armed = False

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

    def _measure(self, frame, heading_deg):
        """Course error in degrees (+ = right of course), or None."""
        if heading_deg is not None:
            self.lock_used = False
            return heading_deg

        if frame is None or not self._armed or self._gyro is None:
            return None

        # Always advance the gyro, even when the lock ends up answering:
        # skipping it would leave a gap in the accumulator the moment the
        # lock loses sight of the reference view.
        self._gyro.update(frame)

        locked = self._lock.error_deg(frame)
        if locked is not None:
            # Drift-free reading available — re-anchor the accumulator to
            # it. This is the entire point of running both sources.
            self._gyro.heading_deg = locked
            self.lock_used = True
            return locked

        self.lock_used = False
        if self._gyro.lost_frames > 0:
            # The accumulator has not been updated this tick, so it is
            # stale rather than merely imprecise. Still the best estimate
            # available, but callers watching lock_used/state can tell.
            return self._gyro.heading_deg
        return self._gyro.heading_deg

    # ── Per-tick ─────────────────────────────────────────────────────────

    def step(self, distances, frame=None, heading_deg: float = None) -> tuple:
        """
        One control tick. Returns (left_motor, right_motor).

        Identical in contract to GapFollowController.step(), so this is a
        drop-in for any loop already driving one.
        """
        error = self._measure(frame, heading_deg)
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
