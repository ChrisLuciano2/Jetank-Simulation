"""
test_gap_logic.py — CLOSED-LOOP offline tests for jetbot_nav.gap_follow.

Rather than feeding pre-scripted sensor values, this file contains a
miniature 2D physics simulator: walls are line
segments, the scan is real raycasting, and the robot moves by
differential-drive kinematics from the controller's motor outputs. The
controller is therefore tested in a true feedback loop — its steering
changes what it senses next — with NO Unity required.

Both field-reported failure geometries of the old 3-sensor system are
encoded here as scenarios:

  A. WALL PARALLEL: driving alongside a long wall with only wall-side rays
     touching it. Old system: grazing side-ray backups racked up the
     failed-attempt counter, flipped direction into the wall, oscillated
     forever. Required here: steady progress along the wall, no heading
     reversals, no collision.
  B. NARROW OBJECT BETWEEN RAYS: a thin pole offset a few degrees from
     center — invisible to rays at 0/+-30 deg. Old system: rammed it,
     backed up, rammed it again. Required here: detect, steer around, no
     collision, and end up PAST it.

Run:
    cd Python
    py test_gap_logic.py           (summary)
    py test_gap_logic.py -v        (adds per-tick trace on failures)
"""

import math
import sys

from jetbot_nav.gap_follow import (
    GapFollowController, SCAN_FOV_DEG, SCAN_RAY_COUNT,
    SIGN_SWITCH_DEBOUNCE_TICKS,
)

# ─── Mini 2D world ───────────────────────────────────────────────────────────
# Coordinates: x = right, z = forward. Heading h in radians, h = 0 faces +z,
# positive h rotates toward +x (i.e. to the robot's right), matching the
# controller's "positive angle = right" convention.

MAX_RANGE   = 12.0
SPEED_SCALE = 3.0     # world units per second at motor value 1.0
TRACK_WIDTH = 1.7     # matches the JETANK chassis width
ROBOT_RADIUS = 1.2    # collision radius (chassis half-diagonal + a little)
DT = 0.1              # 10 Hz, matching the live poll rate

# Motor realism — the Unity truck (and the real one) does NOT respond
# instantly. Ideal kinematics hid a real field bug: the truck turns slower
# and later than the ideal model, creeping into stop range mid-turn.
MOTOR_TAU      = 0.35   # first-order lag (s): wheels approach commands slowly
MOTOR_DEADBAND = 0.04   # commands below this produce no motion (friction)
YAW_FRICTION   = 0.08   # skid-steer: wheel differential below this produces
                        # NO rotation (lateral tire friction resists yaw).
                        # Calibrated from run1.csv, where a 0.06 differential
                        # visibly failed to rotate the truck at all.

VERBOSE = "-v" in sys.argv
_results = []


# ─── Sensor model ────────────────────────────────────────────────────────────
# What the robot is allowed to perceive. Originally this was implicit and
# ideal: 13 rays over 120 deg, every one exact, nothing hidden. That is a
# 2D lidar, and the JETANK does not have one — the scan now comes from the
# camera via jetbot_nav.visual_scan, which is narrower AND has a near
# blind zone. Making the model explicit lets the SAME scenarios run under
# both, so the cost of the real sensor is measurable rather than asserted.

class SensorModel:
    """
    fov_deg / ray_count define the scan geometry. min_range is the killer:
    a camera derives distance from where an object's base occludes the
    floor, so anything nearer than the bottom row of the frame can see is
    not merely unmeasured — it reads as OPEN FLOOR. There is no "too
    close" reading to react to, which is a failure mode a range sensor
    simply does not have.
    """

    def __init__(self, name, fov_deg, ray_count, min_range=0.0):
        self.name = name
        self.fov_deg = fov_deg
        self.ray_count = ray_count
        self.min_range = min_range

    def angles(self):
        step = self.fov_deg / (self.ray_count - 1)
        return [-self.fov_deg / 2 + i * step for i in range(self.ray_count)]

    def observe(self, true_dist):
        if true_dist < self.min_range:
            return MAX_RANGE          # inside the blind zone: looks clear
        return min(true_dist, MAX_RANGE)


LIDAR = SensorModel("lidar", SCAN_FOV_DEG, SCAN_RAY_COUNT, min_range=0.0)

# Matches jetbot_nav.visual_scan.sim_jetank(): ~75 deg of GROUND fov (an
# IMX219's 62.2 deg lens fans wider on the ground once tilted down) and a
# ~0.82-unit blind zone at the mounted height and tilt.
CAMERA = SensorModel("camera", 75.0, SCAN_RAY_COUNT, min_range=0.82)

SENSOR = LIDAR      # test_camera_nav.py swaps this before running scenarios


def make_controller(**kwargs):
    """Controller wired to whatever SENSOR is active."""
    return GapFollowController(angles_deg=SENSOR.angles(),
                               max_range=MAX_RANGE, **kwargs)


def _ray_segment(ox, oz, dx, dz, ax, az, bx, bz):
    """Distance along ray (o + t*d) to segment a-b, or None."""
    ex, ez = bx - ax, bz - az
    denom = dx * ez - dz * ex
    if abs(denom) < 1e-12:
        return None
    t = ((ax - ox) * ez - (az - oz) * ex) / denom
    s = ((ax - ox) * dz - (az - oz) * dx) / denom
    if t >= 0 and 0.0 <= s <= 1.0:
        return t
    return None


def _point_segment_dist(px, pz, ax, az, bx, bz):
    ex, ez = bx - ax, bz - az
    L2 = ex * ex + ez * ez
    if L2 < 1e-12:
        return math.hypot(px - ax, pz - az)
    t = max(0.0, min(1.0, ((px - ax) * ex + (pz - az) * ez) / L2))
    return math.hypot(px - (ax + t * ex), pz - (az + t * ez))


class MiniSim:
    def __init__(self, segments, x=0.0, z=0.0, h=0.0):
        self.segments = segments
        self.x, self.z, self.h = x, z, h
        self.sensor = SENSOR
        self.angles_deg = self.sensor.angles()
        self.collided = False
        self.history = []   # (x, z, h, motors, state)
        self._wl = 0.0      # actual (lagged) wheel speeds
        self._wr = 0.0

    def scan(self):
        out = []
        for a in self.angles_deg:
            w = self.h + math.radians(a)          # world angle of this ray
            dx, dz = math.sin(w), math.cos(w)
            best = MAX_RANGE
            for (ax, az, bx, bz) in self.segments:
                t = _ray_segment(self.x, self.z, dx, dz, ax, az, bx, bz)
                if t is not None and t < best:
                    best = t
            out.append(self.sensor.observe(best))
        return out

    def run(self, controller, seconds):
        ticks = int(seconds / DT)
        for _ in range(ticks):
            d = self.scan()
            ml, mr = controller.step(d)
            # first-order motor lag toward the command
            alpha = DT / (MOTOR_TAU + DT)
            self._wl += alpha * (ml - self._wl)
            self._wr += alpha * (mr - self._wr)
            wl = 0.0 if abs(self._wl) < MOTOR_DEADBAND else self._wl
            wr = 0.0 if abs(self._wr) < MOTOR_DEADBAND else self._wr
            v = (wl + wr) / 2.0 * SPEED_SCALE
            diff = wl - wr
            eff = 0.0 if abs(diff) < YAW_FRICTION else \
                  (abs(diff) - YAW_FRICTION) * (1 if diff > 0 else -1)
            w = eff * SPEED_SCALE / TRACK_WIDTH         # +: nose swings right
            self.h += w * DT
            self.x += math.sin(self.h) * v * DT
            self.z += math.cos(self.h) * v * DT
            self.history.append((self.x, self.z, self.h, (ml, mr),
                                 controller.state))
            for (ax, az, bx, bz) in self.segments:
                if _point_segment_dist(self.x, self.z, ax, az, bx, bz) < ROBOT_RADIUS:
                    self.collided = True
                    return
        return


def check(name, condition, sim=None, detail=""):
    if condition:
        print(f"PASS  {name}")
        _results.append(True)
    else:
        print(f"FAIL  {name}  {detail}")
        if sim is not None and VERBOSE:
            for i, (x, z, h, m, st) in enumerate(sim.history):
                if i % 5 == 0:
                    print(f"   t={i*DT:5.1f}s  x={x:6.2f} z={z:6.2f} "
                          f"h={math.degrees(h):7.1f}deg  "
                          f"motors=({m[0]:5.2f},{m[1]:5.2f})  {st}")
        _results.append(False)


def heading_reversals(sim, axis_deg, tol_deg=90):
    """How many times the heading left the +-tol band around axis_deg —
    i.e. the robot turned around. The wall-oscillation bug shows up as
    repeated reversals."""
    flips, outside = 0, False
    for (_, _, h, _, _) in sim.history:
        err = (math.degrees(h) - axis_deg + 180) % 360 - 180
        now_outside = abs(err) > tol_deg
        if now_outside and not outside:
            flips += 1
        outside = now_outside
    return flips


def wall(ax, az, bx, bz):
    return (ax, az, bx, bz)


def cube(cx, cz, size):
    """Four walls forming a closed box — the simplest real obstacle.
    Module level so other suites (test_course_keep) can build the same
    scenarios rather than reimplementing the geometry."""
    hw = size / 2
    return [wall(cx - hw, cz - hw, cx + hw, cz - hw),
            wall(cx + hw, cz - hw, cx + hw, cz + hw),
            wall(cx + hw, cz + hw, cx - hw, cz + hw),
            wall(cx - hw, cz + hw, cx - hw, cz - hw)]


# ─── Scenario A: parallel wall — the reported oscillation geometry ───────────

def scenario_wall_parallel():
    # Long wall along the robot's LEFT at 1.9 units; robot faces along it
    # (+x direction, heading 90 deg). Only wall-side rays touch it.
    segs = [wall(-5, 1.9, 60, 1.9)]
    sim = MiniSim(segs, x=0, z=0, h=math.radians(90))
    ctrl = make_controller()
    sim.run(ctrl, seconds=25)

    check("wall parallel: no collision", not sim.collided, sim)
    check("wall parallel: no heading reversals (was: endless back-and-forth)",
          heading_reversals(sim, 90) == 0, sim,
          detail=f"reversals={heading_reversals(sim, 90)}")
    check("wall parallel: makes real progress along/away from the wall",
          sim.x > 20, sim, detail=f"final x={sim.x:.1f}")


# ─── Scenario A2: head-on wide wall — must slide off one side and go ─────────

def scenario_wall_head_on():
    segs = [wall(-25, 12, 25, 12)]
    sim = MiniSim(segs, x=0, z=0, h=0.0)
    ctrl = make_controller()
    sim.run(ctrl, seconds=35)

    check("head-on wall: no collision", not sim.collided, sim)
    moved_off = abs(sim.x) > 12
    check("head-on wall: escapes sideways instead of oscillating",
          moved_off, sim, detail=f"final x={sim.x:.1f}, z={sim.z:.1f}")


# ─── Scenario B: thin pole between the old rays — the reported ram loop ──────

def scenario_thin_pole():
    # A 0.35-wide pole centered at (0.6, 9): angular position ~3.8 deg off
    # center at spawn — dead inside the old 0/30-degree blind zone (the old
    # center ray at x=0 misses it forever; the 30-deg rays pass 5+ units to
    # its side). The robot's half-width (0.85) overlaps it -> guaranteed
    # collision under the old system.
    px, pz, hw = 0.6, 9.0, 0.175
    segs = [
        wall(px - hw, pz - hw, px + hw, pz - hw),
        wall(px + hw, pz - hw, px + hw, pz + hw),
        wall(px + hw, pz + hw, px - hw, pz + hw),
        wall(px - hw, pz + hw, px - hw, pz - hw),
    ]
    sim = MiniSim(segs, x=0, z=0, h=0.0)
    ctrl = make_controller()
    sim.run(ctrl, seconds=25)

    check("thin pole: no collision (was: ram, back up, ram again)",
          not sim.collided, sim)
    check("thin pole: ends up PAST the pole", sim.z > pz + 2, sim,
          detail=f"final z={sim.z:.1f} (pole at z={pz})")


# ─── Scenario C: dead-end box — must turn around and leave ───────────────────

def scenario_dead_end():
    segs = [
        wall(-6, 12, 6, 12),     # back wall
        wall(-6, 2, -6, 12),     # left wall
        wall(6, 2, 6, 12),       # right wall
    ]
    sim = MiniSim(segs, x=0, z=5, h=0.0)   # already inside, facing the back
    ctrl = make_controller()
    sim.run(ctrl, seconds=60)

    check("dead end: no collision", not sim.collided, sim)
    check("dead end: escapes through the opening", sim.z < 0, sim,
          detail=f"final z={sim.z:.1f}")


# ─── Scenario D: doorway — thread a 4-unit opening ───────────────────────────

def scenario_doorway():
    segs = [
        wall(-25, 10, -2, 10),
        wall(2, 10, 25, 10),
    ]
    sim = MiniSim(segs, x=-1.0, z=0, h=0.0)   # slightly off the opening axis
    ctrl = make_controller()
    sim.run(ctrl, seconds=30)

    check("doorway: no collision", not sim.collided, sim)
    check("doorway: passes through", sim.z > 12, sim,
          detail=f"final z={sim.z:.1f}")


# ─── Scenario E: open field — straight line, full speed, no wandering ────────

def scenario_open_field():
    sim = MiniSim([], x=0, z=0, h=0.0)
    ctrl = make_controller()
    sim.run(ctrl, seconds=10)

    drifted = abs(sim.x)
    check("open field: drives straight (no phantom steering)",
          drifted < 0.5 and sim.z > 12, sim,
          detail=f"x drift={drifted:.2f}, z={sim.z:.1f}")


# ─── Scenario F: single 1x1 cube — the field-reported wiggle loop ────────────
# Recorded live (run1.csv): the robot creeped at the cube, backed up with the
# nose swinging the WRONG way (stale remembered side), searched 0.2 s, and
# repeated forever — a stable limit cycle with zero net progress. Required
# here (with realistic motor lag): pass the cube with at most one backup.

def scenario_single_cube():
    for ox in (0.0, 0.3, 0.6):
        sim = MiniSim(cube(ox, 9, 1.0), x=0, z=0, h=0.0)
        ctrl = make_controller()
        sim.run(ctrl, seconds=30)

        states = [s for (_, _, _, _, s) in sim.history]
        backups = sum(1 for a, b in zip(states, states[1:])
                      if b == "BACKUP" and a != "BACKUP")
        check(f"single cube (offset {ox}): no collision", not sim.collided, sim)
        check(f"single cube (offset {ox}): passes it (was: wiggle loop)",
              sim.z > 11, sim, detail=f"final z={sim.z:.1f}")
        check(f"single cube (offset {ox}): at most one backup",
              backups <= 1, sim, detail=f"backups={backups}")


# ─── Scenario G: recorded-run replay — the "waving" field bug (run2) ─────────
# The controller ran against a live 1x1 cube in Unity and oscillated between
# steering left and right of it five times in 30 s. Root cause was the gap
# hysteresis snapping the committed target back to the containing gap's
# center whenever the scan momentarily read fully-free (a small cube going
# just above GAP_THRESHOLD as the robot rotated off-axis). Replaying the
# same scans through the fixed controller must NOT flip target sign within
# any single FORWARD phase — legitimate flips only across BACKUP/SEARCH
# rotations, which reset the target on purpose.

def scenario_recorded_wave_replay():
    import csv, os
    path = os.path.join(os.path.dirname(__file__), "test_data",
                        "run2_wave_bug.csv")
    if not os.path.exists(path):
        print(f"SKIP  recorded-wave replay: {path} not present")
        return

    from jetbot_nav.gap_follow import GapFollowController
    rows = list(csv.DictReader(open(path)))
    ctrl = GapFollowController(max_range=12.0)

    flips_within_forward = 0
    phase_sign = 0            # sign locked when this FORWARD phase started
    prev_state = None
    for r in rows:
        d = [float(x) for x in r["distances"].split(";")]
        ctrl.step(d)
        state = ctrl.state
        t_deg = ctrl.debug()["target_deg"]

        if state != "FORWARD":
            phase_sign = 0
        elif t_deg is not None and abs(t_deg) > 0.5:
            sign = 1 if t_deg > 0 else -1
            if prev_state != "FORWARD":
                phase_sign = sign
            elif phase_sign != 0 and sign != phase_sign:
                flips_within_forward += 1
        prev_state = state

    check("recorded run: no target sign-flips within a FORWARD phase "
          "(was: 5 flips per run)",
          flips_within_forward == 0, None,
          detail=f"flips within a single FORWARD phase = {flips_within_forward}")


# ─── Scenario H: pivot swings the SHORT way, not the committed way ──────────
# Field bug (run3): PIVOT rotated toward the long-term gap-following target
# instead of toward whichever side already cleared the obstacle soonest.
# When the obstacle sat near-center-left (bearing -30deg, needing only ~3deg
# more to exit the corridor) but the committed target pointed right, the
# OLD code swung right anyway -- sweeping 63deg through dead-center instead
# of 3deg -- blew through PIVOT_TIMEOUT_TICKS, and dumped into BACKUP,
# undoing the whole approach and causing the reported "waving" behavior.

def scenario_pivot_swings_short_way():
    from jetbot_nav.gap_follow import GapFollowController

    ctrl = GapFollowController(max_range=12.0)
    ctrl._target_deg = 20.0   # committed to routing right, same as run3

    d = [12.0] * 13
    step = 120.0 / 12
    for i in range(13):
        a = -60 + i * step
        if abs(a - (-30)) < 1e-6:
            d[i] = 2.59
        elif abs(a - (-20)) < 1e-6 or abs(a - (-40)) < 1e-6:
            d[i] = 2.7

    ctrl.step(d)
    check("pivot swings toward the obstacle's own side, not the committed "
          "target's side (was: swung the long way through dead-center)",
          ctrl.state == "PIVOT" and ctrl._search_side == "left", None,
          detail=f"state={ctrl.state} search_side={ctrl._search_side}")


# ─── Scenario I: committed side has a gap, but a MUCH better one exists ──────
# on the other side (the doorway field bug). Sign commitment must be a
# preference, not an absolute lock -- otherwise once the robot has rotated
# far enough that the real opening ends up on the "wrong" side of center,
# it keeps steering at a distant, technically-on-side gap forever.

def scenario_prefers_much_better_off_side_gap():
    from jetbot_nav.gap_follow import GapFollowController
    ctrl = GapFollowController(max_range=12.0)
    ctrl._target_deg = 45.0   # committed right, same as the doorway approach

    # Actual scan captured from the doorway mini-sim at the tick where the
    # old code jumped to +45deg: a wide, dead-ahead gap (-30..+10, center
    # -10) alongside a narrower on-side gap (+30..+60, center +45).
    d = [4.86, 4.30, 3.96, 12.0, 12.0, 12.0, 12.0, 12.0, 4.83, 5.75, 7.38, 10.76, 12.0]

    gaps = ctrl._find_gaps(d)
    # The sign switch is debounced (see run5 field bug) -- it must SUSTAIN
    # for SIGN_SWITCH_DEBOUNCE_TICKS in a row before the controller trusts
    # it, so call it repeatedly with the same scan as a real multi-tick
    # approach would.
    #
    # This loop used to run a hardcoded 5 times against a debounce of 6, so
    # it asserted the switch had happened exactly one tick before the
    # controller was ever going to make it. That read as a controller
    # defect for a long time; it was not. Derive the count from the
    # constant so the two cannot drift apart again.
    choice = None
    for _ in range(SIGN_SWITCH_DEBOUNCE_TICKS):
        choice = ctrl._choose_gap(gaps)
    check("switches to a dramatically better off-side gap instead of a "
          "distant on-side one (was: doorway ignored in favor of empty "
          "space past the wall\'s end)",
          choice < 0, None, detail=f"chose target={choice} (want negative)")


# ─── Scenario J: a single clean tick mid-recovery must not be trusted ────────
# A 13-ray/10-degree scan can miss a nearby corner for exactly one frame as
# it crosses between two rays. Trusting that one tick's "all clear" reading
# sent the robot straight back into the same obstacle next tick (run4 field
# bug: rapid FORWARD/PIVOT flicker before falling into BACKUP).

def scenario_single_tick_clear_blip_ignored():
    from jetbot_nav.gap_follow import GapFollowController
    ctrl = GapFollowController(max_range=12.0)

    blocked = [12.0] * 13
    blocked[6] = 2.3   # dead ahead, well inside STOP_FORWARD
    clear = [12.0] * 13

    ctrl.step(blocked)               # enters PIVOT
    state_before = ctrl.state
    ctrl.step(clear)                 # ONE clean tick -- should not be enough
    state_after_one_blip = ctrl.state
    ctrl.step(blocked)               # obstacle is still really there

    check("one clean tick mid-PIVOT does not alone authorize resuming "
          "FORWARD (was: instant flicker back into the obstacle)",
          state_before == "PIVOT" and state_after_one_blip == "PIVOT",
          None, detail=f"before={state_before} after_blip={state_after_one_blip}")


# ─── Scenario K: a multi-tick clear blip must not be trusted either ─────────
# The single-tick debounce (Scenario J) wasn't enough -- run6 showed a
# corridor reading "fully clear" for THREE consecutive ticks (a corner
# swinging past the ray fan during PIVOT, then swinging back) before the
# obstacle reappeared. The old 2-tick debounce was fooled by it, exited
# PIVOT, picked a near-edge target, and drove right back into the same
# corner one tick after it reappeared ("almost clears, curves right back
# into it"). This replays the exact recorded sequence and checks the
# controller stays in PIVOT through the whole blip.

def scenario_multi_tick_clear_blip_ignored():
    import os, csv
    path = os.path.join(os.path.dirname(__file__), "test_data",
                        "run6_pivot_exit_bug.csv")
    if not os.path.exists(path):
        print(f"SKIP  multi-tick clear blip replay: {path} not present")
        return

    from jetbot_nav.gap_follow import GapFollowController
    rows = list(csv.DictReader(open(path)))
    ctrl = GapFollowController(max_range=12.0)

    # The old bug fired at ticks 244-245 (0-indexed) of this exact recording.
    exited_during_blip = False
    for i, r in enumerate(rows):
        d = [float(x) for x in r["distances"].split(";")]
        ctrl.step(d)
        if i in (244, 245) and ctrl.state == "FORWARD":
            exited_during_blip = True

    check("stays in PIVOT through a 3-tick clear blip instead of exiting "
          "early (was: exited, then drove back into the same corner)",
          not exited_during_blip, None,
          detail=f"exited_during_blip={exited_during_blip}")


# ─── Scenario L: PIVOT and SEARCH rotation speeds are truly independent ──────
# PIVOT_SPEED existed as a constant but was dead code -- both states shared
# one motor helper that only ever read SEARCH_SPEED, so PIVOT silently ran
# at whatever SEARCH_SPEED happened to be. Speeding up SEARCH (run7 field
# report: a stuck-rotating stall took ~5.5s) then also sped up PIVOT and
# broke a narrow-gap scenario as an unintended side effect. This checks the
# two motor helpers actually differ when the two speed constants differ.

def scenario_pivot_search_speeds_independent():
    from jetbot_nav.gap_follow import GapFollowController
    import jetbot_nav.gap_follow as gf

    ctrl = GapFollowController(max_range=12.0)
    ctrl._search_side = "left"
    pivot_l, pivot_r = ctrl._pivot_motors()
    search_l, search_r = ctrl._search_motors()

    check("PIVOT and SEARCH motor helpers use their own distinct speed "
          "constants (was: PIVOT_SPEED was dead code, both shared "
          "SEARCH_SPEED)",
          abs(pivot_l) == gf.PIVOT_SPEED and abs(search_l) == gf.SEARCH_SPEED,
          None, detail=f"pivot={pivot_l} (want {gf.PIVOT_SPEED}), "
                        f"search={search_l} (want {gf.SEARCH_SPEED})")


# ─── Scenario L: the minimum gap is an ANGLE, not a ray count ───────────────
# MIN_GAP_RAYS used to be a literal 3, which silently means a different
# physical opening on every sensor: 30 deg on the 13-ray/120-deg fan but
# 18.8 deg on the 13-ray/75-deg camera scan. Swapping to the camera
# therefore halved the narrowest gap the robot would attempt, with nothing
# anywhere saying so. Pin the invariant instead of the ray count.

def scenario_min_gap_is_sensor_independent():
    from jetbot_nav.gap_follow import GapFollowController, MIN_GAP_DEG

    def rays_for(fov, n):
        step = fov / (n - 1)
        ctrl = GapFollowController(
            angles_deg=[-fov / 2 + i * step for i in range(n)], max_range=12.0)
        return ctrl.min_gap_rays, ctrl.min_gap_rays * step

    configs = [(120.0, 13), (75.0, 13), (62.2, 13), (90.0, 7), (75.0, 25)]
    spans = [rays_for(f, n)[1] for f, n in configs]

    check("minimum gap stays ~MIN_GAP_DEG across every scan resolution",
          all(MIN_GAP_DEG <= s < MIN_GAP_DEG + 10 for s in spans), None,
          detail=f"spans={[round(s, 1) for s in spans]}")

    # Never round down: accepting a gap narrower than MIN_GAP_DEG is the
    # failure mode that matters (driving at an opening too tight to fit).
    check("derived ray count never rounds the gap below MIN_GAP_DEG",
          all(s >= MIN_GAP_DEG for s in spans), None,
          detail=f"spans={[round(s, 1) for s in spans]}")

    # A single free ray is a hole in the data, not an opening.
    coarse_rays, _ = rays_for(180.0, 3)     # 90 deg per ray
    check("a very coarse scan still needs more than one free ray",
          coarse_rays >= 2, None, detail=f"got {coarse_rays}")

    check("the legacy 120-deg fan keeps its original 3-ray behaviour",
          rays_for(120.0, 13)[0] == 3, None,
          detail=f"got {rays_for(120.0, 13)[0]}")


# ─── Run all ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    scenario_open_field()
    scenario_wall_parallel()
    scenario_wall_head_on()
    scenario_thin_pole()
    scenario_dead_end()
    scenario_doorway()
    scenario_single_cube()
    scenario_recorded_wave_replay()
    scenario_pivot_swings_short_way()
    scenario_prefers_much_better_off_side_gap()
    scenario_single_tick_clear_blip_ignored()
    scenario_multi_tick_clear_blip_ignored()
    scenario_pivot_search_speeds_independent()
    scenario_min_gap_is_sensor_independent()

    passed = sum(_results)
    print(f"\n{passed}/{len(_results)} checks passed")
    sys.exit(0 if all(_results) else 1)