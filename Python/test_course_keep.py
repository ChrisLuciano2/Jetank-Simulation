"""
test_course_keep.py — closed-loop tests for jetbot_nav.course_keep.

Reuses test_gap_logic's 2D world and differential-drive physics, and
feeds CourseKeeper the simulator's EXACT heading rather than a
camera-derived one. That is deliberate: jetbot_nav.heading already has
its own suite and has been measured against real Unity renders, so
re-estimating here would only blur two questions together. What is under
test is the control loop — does the robot actually come back to its
course after being pushed off it, and does course-keeping stay out of the
way when avoidance needs to happen.

The headline check is the one comparing against plain gap-following:
"returns to course" only means something if the same run WITHOUT course
keeping does not.

    py -3.8 test_course_keep.py
    py -3.8 test_course_keep.py -v
"""

import math
import sys

import test_gap_logic as G
from jetbot_nav.course_keep import (
    CourseKeeper, ON_COURSE, OFF_COURSE, UNKNOWN, COURSE_TOLERANCE_DEG,
)
from jetbot_nav.gap_follow import GapFollowController
from jetbot_nav.target_seek import SeekingGapFollowController

G.SENSOR = G.CAMERA          # the sensor the robot actually has
_results = []


def make_seeking():
    """A seeking controller wired to the active sensor's geometry."""
    return SeekingGapFollowController(angles_deg=G.SENSOR.angles(),
                                      max_range=G.MAX_RANGE)


def check(label, ok, detail=""):
    _results.append(bool(ok))
    print(f"{'PASS' if ok else 'FAIL'}  {label}" + (f"  {detail}" if detail and not ok else ""))


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


class _Driver:
    """
    Adapts CourseKeeper to MiniSim's controller interface, supplying the
    true heading error each tick. MiniSim reads `.state` for its history.
    """

    def __init__(self, keeper, sim, course_deg):
        self.keeper = keeper
        self.sim = sim
        self.course_deg = course_deg
        self.errors = []
        self.states = []

    @property
    def state(self):
        return self.keeper.controller.state

    def step(self, distances):
        err = wrap(math.degrees(self.sim.h) - self.course_deg)
        self.errors.append(err)
        motors = self.keeper.step(distances, heading_deg=err)
        self.states.append(self.keeper.state)
        return motors


def run_course(segments, seconds=30.0, start=(0.0, 0.0, 0.0), keep=True):
    """Drive the scenario with or without course keeping. Returns
    (sim, final_heading_error_deg, driver_or_None)."""
    x, z, h = start
    sim = G.MiniSim(segments, x=x, z=z, h=math.radians(h))
    course = h

    if keep:
        keeper = CourseKeeper(controller=make_seeking())
        keeper.set_course(frame=None)
        driver = _Driver(keeper, sim, course)
        sim.run(driver, seconds=seconds)
        return sim, wrap(math.degrees(sim.h) - course), driver

    ctrl = G.make_controller()
    sim.run(ctrl, seconds=seconds)
    return sim, wrap(math.degrees(sim.h) - course), None


# ─── 1. Open field: holding a course must not disturb straight running ──────

sim, err, drv = run_course([], seconds=12.0)
check("open field: still drives straight", abs(err) < 2.0,
      f"final heading error {err:+.1f} deg")
check("open field: no collision", not sim.collided)
check("open field: reports ON_COURSE throughout",
      all(s == ON_COURSE for s in drv.states),
      f"states seen: {sorted(set(drv.states))}")
check("open field: makes progress", sim.z > 15, f"z={sim.z:.1f}")


# ─── 2. One obstacle: detour, then come back ────────────────────────────────
# The whole point of the module. gap_follow will steer around the cube;
# without course keeping it then drives off on the new heading forever.

CUBE = G.cube(0.0, 9, 1.0)

sim_k, err_k, drv_k = run_course(CUBE, seconds=30.0)
sim_p, err_p, _ = run_course(CUBE, seconds=30.0, keep=False)

check("one obstacle: no collision with course keeping", not sim_k.collided)
check("one obstacle: returns to the original heading",
      abs(err_k) <= 8.0, f"final heading error {err_k:+.1f} deg")
check("one obstacle: plain gap-following does NOT return to course "
      "(otherwise this module proves nothing)",
      abs(err_p) > abs(err_k),
      f"plain {err_p:+.1f} vs kept {err_k:+.1f}")
check("one obstacle: the robot actually left course during the detour",
      max(abs(e) for e in drv_k.errors) > 10.0,
      f"max deviation only {max(abs(e) for e in drv_k.errors):.1f} deg — "
      f"it never detoured, so recovery was not exercised")
check("one obstacle: ends up OFF_COURSE at some point, ON_COURSE at the end",
      OFF_COURSE in drv_k.states and drv_k.states[-1] == ON_COURSE,
      f"last state {drv_k.states[-1]}, saw {sorted(set(drv_k.states))}")


# ─── 3. Several obstacles: error must not accumulate ────────────────────────
# One detour is recoverable by luck. The real failure mode is drift
# compounding over a course, which is what makes plain gap-following
# useless for going anywhere in particular.

SLALOM = (G.cube(0.0, 8, 1.0) + G.cube(-2.5, 16, 1.0) + G.cube(2.0, 24, 1.0))

sim_k3, err_k3, drv_k3 = run_course(SLALOM, seconds=45.0)
sim_p3, err_p3, _ = run_course(SLALOM, seconds=45.0, keep=False)

check("slalom: no collision", not sim_k3.collided)
check("slalom: still on course after three detours",
      abs(err_k3) <= 10.0, f"final heading error {err_k3:+.1f} deg")
check("slalom: beats plain gap-following",
      abs(err_k3) < abs(err_p3),
      f"plain {err_p3:+.1f} vs kept {err_k3:+.1f}")
check("slalom: makes forward progress along the course",
      sim_k3.z > 20, f"z={sim_k3.z:.1f}")

# Heading-only course keeping corrects DIRECTION, not position: the robot
# ends up parallel to the original line, generally offset sideways from
# it. Worth pinning so the limitation is not mistaken for a bug later.
print(f"      (lateral offset after slalom: x={sim_k3.x:+.1f} — heading-only "
      f"keeping does not remove this; see README)")


# ─── 4. Recovering from a large initial error ───────────────────────────────
# Start badly misaligned in open field and check convergence, and that it
# does not overshoot into an oscillation.

for start_err in (-40.0, -20.0, 20.0, 40.0):
    s, e, d = run_course([], seconds=18.0, start=(0.0, 0.0, start_err))
    settled = abs(e) <= COURSE_TOLERANCE_DEG + 3.0
    # Oscillation would show as the sign of the error flipping repeatedly.
    signs = [1 if x > 2 else -1 if x < -2 else 0 for x in d.errors]
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a and b and a != b)
    check(f"recovers from {start_err:+.0f} deg off course",
          settled and flips <= 2,
          f"final {e:+.1f} deg, {flips} sign flips")


# ─── 5. Unknown heading degrades to plain gap-following, not to a crash ─────
# Both heading sources return None on a featureless view. The robot must
# keep avoiding obstacles and simply stop holding a course.

sim_u = G.MiniSim(CUBE, x=0, z=0, h=0.0)
keeper_u = CourseKeeper(controller=make_seeking())


class _Blind:
    def __init__(self, keeper):
        self.keeper = keeper
        self.states = []

    @property
    def state(self):
        return self.keeper.controller.state

    def step(self, distances):
        m = self.keeper.step(distances)      # no frame, no heading
        self.states.append(self.keeper.state)
        return m


blind = _Blind(keeper_u)
sim_u.run(blind, seconds=25.0)
check("no heading available: still avoids the obstacle",
      not sim_u.collided)
check("no heading available: reports UNKNOWN rather than pretending",
      all(s == UNKNOWN for s in blind.states),
      f"states seen: {sorted(set(blind.states))}")
check("no heading available: still gets past the obstacle",
      sim_u.z > 12, f"z={sim_u.z:.1f}")


# ─── 6. Course bias never overrides a recovery manoeuvre ────────────────────
# Inherited from SeekingGapFollowController, but assert it here too: this
# is the layer that would make it a safety problem, by pulling a PIVOT
# toward a heading instead of toward getting unstuck.

keeper_r = CourseKeeper(controller=make_seeking())
keeper_r.set_course(frame=None)

plain_r = GapFollowController(angles_deg=G.SENSOR.angles(),
                              max_range=G.MAX_RANGE)

blocked = [12.0] * 13
blocked[6] = 1.2                     # inside EMERGENCY_FORWARD

same = True
for _ in range(6):
    a = keeper_r.step(blocked, heading_deg=30.0)   # badly off course
    b = plain_r.step(blocked)
    if a != b:
        same = False

check("course bias is fully suppressed during emergency recovery",
      same, f"keeper diverged from plain gap-following while in "
            f"{keeper_r.controller.state}")
check("emergency recovery actually engaged (otherwise nothing was tested)",
      keeper_r.controller.state in ("BACKUP", "PIVOT"),
      f"state was {keeper_r.controller.state}")


# ─── Summary ────────────────────────────────────────────────────────────────

passed = sum(_results)
print(f"\n{passed}/{len(_results)} checks passed")
sys.exit(0 if all(_results) else 1)
