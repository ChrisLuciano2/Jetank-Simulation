"""
jetbot_nav.gap_follow — obstacle avoidance from an N-ray proximity scan.

WHY A SCAN RATHER THAN A FEW FIXED SENSORS
──────────────────────────────────────────
An earlier 3-fixed-ray design (0, +/-30 deg) failed in the field two ways:

  1. WALL OSCILLATION: driving parallel to a wall, the single wall-side ray
     grazes it at a shallow angle and keeps dipping into "too close" range
     even though the robot is progressing fine. Those false alarms racked
     up the failed-attempt counter, which then flipped the escape direction
     — steering the robot back INTO the wall, forever.
  2. BLIND GAPS: 30-degree spacing leaves huge angular blind zones. A
     narrow object slightly off-center is invisible to all three rays
     until contact (or permanently), so the robot rams it, backs up, and
     rams it again.

Both are information problems, not tuning problems. This module uses a
13-ray scan across a 120-degree fan and a FOLLOW-THE-GAP controller:

  - CORRIDOR CLEARANCE: a ray only counts as "blocking" if its obstacle
    point actually falls inside the robot's forward corridor (its width
    plus margin). A wall beside the robot no longer triggers emergency
    behavior — fixes the oscillation at the root, with no flip counter
    to misfire.
  - GAP STEERING: mark rays reading beyond GAP_THRESHOLD as free, find
    contiguous free runs wide enough to fit through, steer at the chosen
    gap's center. Wall-following, straightening-out after an obstacle,
    and threading between objects are all emergent — there is no
    committed left/right direction to get stuck in.
  - GAP HYSTERESIS: keep steering at the previously chosen gap while it
    still exists, so two similar gaps can't cause dithering.
  - SEARCH + BACKUP recovery: if no gap fits, rotate in place (sticky
    direction) until one appears; if something enters emergency range,
    back up briefly first. Because SEARCH re-orients before driving
    forward again, the "back up then ram the same object" loop is gone.

HARDWARE PARITY NOTE (important — read before physical deployment)
──────────────────────────────────────────────────────────────────
This widens the sensor contract: the physical robot must provide an
equivalent scan. Two realistic options for the JETANK:
  - an ultrasonic/IR sensor swept by a servo (the JETANK's TTL servo bus,
    already driven via SCSCtrl, has capacity for this — the classic cheap
    "radar" build), or
  - a low-cost 2D lidar (e.g. RPLidar-class) downsampled to N rays.
Either way, the robot-side shim only has to answer get_proximity_scan()
with {"fov": deg, "count": N, "max_range": m, "distances": [...]} and all
Python logic here runs unchanged. A build limited to 3 fixed sensors
cannot drive this controller — see the blind-zone failures above.

USAGE
    from jetbot_nav import gap_follow
    gap_follow.drive_with_gap_following(duration=20.0)

TUNED FOR THE CAMERA, NOT THE OLD FAN
─────────────────────────────────────
The scan now comes from jetbot_nav.visual_scan (the JETANK has a camera
and no range sensor), which is narrower than the 120-deg fan this module
was originally written against and blind inside ~0.8 units. Two constants
carry the consequences:

  - CORRIDOR_HALF dropped 2.0 -> 1.7. At 2.0 a 4-unit doorway put both
    edges exactly on the blocking threshold, and without peripheral
    vision there is no early view of the far edge to centre against.
  - MIN_GAP_DEG replaced a raw MIN_GAP_RAYS count, which meant a
    different physical opening on every sensor.

The near blind zone is currently safe only because EMERGENCY_FORWARD
(1.6) sits outside it — the robot stops before anything can vanish
underneath the camera. Re-check that relationship before changing either
the mounting or EMERGENCY_FORWARD; test_visual_scan.py prints the
comparison on every run.

TROUBLESHOOTING
    - Logic regressions:  py test_gap_logic.py   (offline, closed-loop 2D
      simulator — includes both field-reported failure geometries)
    - Camera-model runs:  py test_camera_nav.py  (same scenarios, but
      through the narrow FOV and blind zone the real sensor has)
    - Record a live run:  gap_follow.drive_with_gap_following(20, log_path="run.csv")
    - Replay it offline:  gap_follow.replay_log("run.csv")
"""

import csv
import math
import time

import sim_client
import sim_robot_id
from jetbot import Robot

# ─── Scan geometry (must match ProximitySensor.cs defaults) ──────────────────

SCAN_FOV_DEG      = 120.0
SCAN_RAY_COUNT    = 13
DEFAULT_MAX_RANGE = 12.0

# ─── Tuning constants ────────────────────────────────────────────────────────

CORRIDOR_HALF     = 1.7  # robot half-width (0.85) + margin: obstacles whose
                           # lateral offset exceeds this are BESIDE us, not in
                           # our way, no matter how close their ray reads.
                           #
                           # Was 2.0, which demanded a 4.0-wide clear corridor
                           # from a 1.7-wide robot. A 4-unit doorway therefore
                           # put BOTH its edges at lateral exactly 2.0 -- on
                           # the blocking threshold by construction -- so any
                           # approach even slightly off-centre read the near
                           # edge as an in-corridor obstacle and recovered
                           # instead of threading. The 120-deg fan hid this:
                           # it could see both edges early enough to centre up
                           # first. The camera sees ~75 deg, so the far edge is
                           # outside the frame during the approach and there is
                           # no centring evidence to act on.
                           #
                           # 1.7 is "half a robot-width of margin per side",
                           # and is the LARGEST value that passes both the
                           # camera and lidar suites -- lower values (1.5 and
                           # below) start failing the lidar doorway instead.
                           # Verified across start offsets -1.0..+1.0; beyond
                           # that the robot physically overlaps the doorway
                           # frame and correctly routes around rather than
                           # forcing it.
STOP_FORWARD      = 3    # forward-corridor clearance below this -> BACKUP
                           # (was 2.4 -- run6/run7/run8 field reports all
                           # showed a pattern of getting close, backing off
                           # just barely enough to pass the exit check, then
                           # needing a SECOND full recovery cycle shortly
                           # after. More standoff here means every threshold
                           # that scales off it (PIVOT_EXIT_MARGIN,
                           # BACKUP_CLEAR_TARGET, SEARCH_EXIT_CLEARANCE)
                           # gets more real margin too, not just this one.
                           # 2.6 is the highest value that stays clean
                           # against the full suite; 2.7+ makes the run6
                           # CSV-replay regression's hardcoded tick numbers
                           # go stale (the recorded trajectory legitimately
                           # reacts earlier and diverges) even though the
                           # real closed-loop behavior is still fine --
                           # staying at 2.6 avoids that question entirely.
EMERGENCY_FORWARD = 1.6    # hard floor: triggers BACKUP even mid-SEARCH
SLOW_FORWARD      = 6.0    # speed scales down from here toward STOP_FORWARD

GAP_THRESHOLD     = 4.5    # a ray is "free" if it reads at least this far

MIN_GAP_DEG       = 30.0   # a gap must subtend at least this ANGLE to count.
                           # Was a raw ray count (3), which silently means a
                           # different physical gap on every sensor: 3 rays is
                           # 30 deg on the old 13-ray/120-deg fan but only
                           # 18.8 deg on the 13-ray/75-deg camera scan, so
                           # swapping the sensor quietly halved the minimum
                           # opening the robot would attempt to drive through.
                           # An angle is sensor-independent; the ray count is
                           # derived per-controller in __init__.
MIN_GAP_RAYS_FLOOR = 2     # never accept a "gap" of a single ray, however
                           # coarse the scan — one ray is a hole in the data,
                           # not evidence of a traversable opening.

# Sign-switch debounce, shared with target_seek's overridden _choose_gap.
# These were local variables duplicated in both implementations; a test
# hardcoded "5" against a debounce of 6 and read as a controller bug for a
# long time. Module scope so there is exactly one source of truth.
SIGN_SWITCH_MARGIN_DEG     = 20.0
SIGN_SWITCH_DEBOUNCE_TICKS = 6

TURN_GAIN         = 1.6    # steering strength per radian of target angle
MIN_TURN_DIFF     = 0.24   # minimum wheel differential of any commanded turn
                           # (must beat skid-steer yaw friction; see _steer)
MAX_SPEED         = 0.5
MIN_FORWARD_SPEED = 0.12

BACKUP_SPEED      = 0.3
BACKUP_MIN_TICKS  = 4      # ~0.4 s minimum reverse before even checking
                           # clearance (avoid quitting after one noisy tick)
BACKUP_MAX_TICKS  = 35     # ~3.5 s safety cap (no rear sensor -- can't tell
                           # if backing further would hit something behind,
                           # so this is a blind ceiling, not a target)
BACKUP_CLEAR_TARGET = STOP_FORWARD + 2.0   # real cushion to back up TO, not
                           # just a fixed duration. A fixed 0.8s backup kept
                           # returning the robot to nearly the identical
                           # encounter with the SAME obstacle every recovery
                           # cycle -- a stable no-progress limit cycle (field
                           # bug: run5, blocked-ray pattern (1,2) and (10,11)
                           # recurring exactly across 4 separate approaches
                           # in 30s). Backing up until actual room exists,
                           # not just "however long we always back up," is
                           # what breaks that cycle.

PIVOT_SPEED       = 0.18   # in-place rotation (strong enough to beat motor
                           # static friction / deadband on the real truck).
                           # This constant existed but was DEAD CODE until
                           # today -- PIVOT and SEARCH shared one motor
                           # helper that only ever read SEARCH_SPEED, so
                           # PIVOT has actually been running at 0.18 (never
                           # 0.22) this whole time, through every test and
                           # every field run so far. Now that the two are
                           # properly split, keeping PIVOT at its real,
                           # already-validated value (0.18) rather than the
                           # never-tested one (0.22) it was "set" to.
PIVOT_EXIT_MARGIN = 2    # forward room above STOP_FORWARD needed to resume
                           # (was 0.8 -- too thin: PIVOT only checks the
                           # CURRENT, still-rotating heading, but the truck's
                           # actual heading lags the newly-chosen FORWARD
                           # target by real motor lag. One tick of that lag
                           # was enough to eat a 0.8m buffer and re-trigger
                           # the same obstacle immediately -- run6 field bug,
                           # "almost clears, curves right back into it.")
PIVOT_TIMEOUT_TICKS = 30   # ~3 s pivoting without progress -> BACKUP

PIVOT_CLEAR_DEBOUNCE_TICKS = 4   # a "corridor is clear" reading must hold this
                           # many CONSECUTIVE ticks before PIVOT trusts it
                           # enough to resume driving. A 13-ray/10-degree
                           # scan can miss a nearby corner for several
                           # frames while it crosses between two rays --
                           # first seen as a 1-tick blip (run4: rapid
                           # FORWARD/PIVOT flicker), then a 3-tick blip
                           # (run6: exited PIVOT, picked a near-edge target,
                           # and drove right back into the same corner one
                           # tick after it reappeared). 2 ticks of debounce
                           # covered the first case but not the second: 4
                           # gives a real margin over the longest blip seen
                           # so far, not just the minimum that would have
                           # passed the last failure.

SEARCH_SPEED          = 0.30   # was 0.18. The obvious "just raise this"
                               # fix broke a narrow-gap scenario at first
                               # try -- turned out PIVOT_SPEED was DEAD CODE
                               # and PIVOT had silently been sharing this
                               # same constant the whole time, so raising it
                               # sped up PIVOT too, which has a much
                               # tighter tolerance (see _pivot_motors).
                               # With the two properly split, this value is
                               # tested safe up to 0.60 in the offline
                               # suite; kept well under that AND under
                               # MAX_SPEED (0.5, the truck's own forward
                               # driving cap) since a bare in-place rotation
                               # exceeding its own forward speed cap isn't
                               # something the physical hardware should be
                               # asked to do just because the sim allows it.
                               # Cuts a real ~5.5s stuck-rotating stall (run7
                               # field report) to roughly 3.3s.
SEARCH_CLEAR_DEBOUNCE_TICKS = 2   # SEARCH's own debounce, separate from
                           # PIVOT's. PIVOT's check is a bare corridor
                           # reading (prone to the multi-tick corner-slip
                           # blips above); SEARCH's check additionally
                           # requires a real MIN_GAP_RAYS-wide gap to exist
                           # within SEARCH_EXIT_ANGLE_DEG of center, which
                           # is a much less ambiguous signal on its own --
                           # it doesn't need as many confirming ticks.
                           # This matters because the debounce is tick-
                           # COUNTED, not time-based, so at a faster search
                           # rotation speed the same tick count covers MORE
                           # angle, effectively shrinking how narrow a gap
                           # can be and still stay "usable" for the full
                           # debounce window. Sharing PIVOT's larger count
                           # here made a narrow gap (thin-pole scenario)
                           # sweep past faster than it could be confirmed.
SEARCH_EXIT_CLEARANCE = STOP_FORWARD + 1.5   # forward room needed to leave SEARCH
SEARCH_EXIT_ANGLE_DEG = 20.0                 # ...with a gap this close to center
SEARCH_FLIP_TICKS     = 90                   # ~9 s rotating one way -> try other

FULL_CLEAR_FRACTION   = 0.9  # all rays >= this * max_range -> totally open field


# ─── Sensor query ────────────────────────────────────────────────────────────


def get_proximity_scan() -> dict:
    """
    Query the scanning sensor. Returns
        {"angles_deg": [...], "distances": [...], "max_range": m}
    (angles ascend left -> right; negative = left of forward) or None if the
    query failed. Works against Unity's SimQueryServer or any physical shim
    answering the same shape.
    """
    resp = sim_client.send_query({"command": "get_proximity_scan", "robot_id": sim_robot_id.TRUCK_ID})
    if not resp or resp.get("status") != "ok":
        return None
    fov, count = float(resp["fov"]), int(resp["count"])
    step = fov / (count - 1)
    return {
        "angles_deg": [-fov / 2 + i * step for i in range(count)],
        "distances":  [float(d) for d in resp["distances"]],
        "max_range":  float(resp.get("max_range", DEFAULT_MAX_RANGE)),
    }


# ─── Controller ──────────────────────────────────────────────────────────────

FORWARD, SEARCH, BACKUP, PIVOT = "FORWARD", "SEARCH", "BACKUP", "PIVOT"


class GapFollowController:
    """
    Feed one scan per tick to step(); it returns (left_motor, right_motor).
    Pure function of (internal state, scan) — no I/O — so it is unit-testable
    and replayable offline (see test_gap_logic.py / replay_log()).

    Angle convention: DEGREES in the scan, negative = left, positive = right.
    """

    def __init__(self, angles_deg=None, max_range: float = None):
        if angles_deg is None:
            step = SCAN_FOV_DEG / (SCAN_RAY_COUNT - 1)
            angles_deg = [-SCAN_FOV_DEG / 2 + i * step for i in range(SCAN_RAY_COUNT)]
        self.angles_deg = list(angles_deg)
        self.angles_rad = [math.radians(a) for a in self.angles_deg]
        self.max_range = float(max_range or DEFAULT_MAX_RANGE)

        # Convert MIN_GAP_DEG into a ray count for THIS scan's resolution,
        # so the minimum opening stays a fixed angle whatever sensor is
        # feeding us (13 rays over 120 deg and over 75 deg are different
        # instruments). Ceil, so a gap is never accepted below the angle.
        if len(self.angles_deg) >= 2:
            step = abs(self.angles_deg[1] - self.angles_deg[0])
        else:
            step = MIN_GAP_DEG
        self.min_gap_rays = max(MIN_GAP_RAYS_FLOOR,
                                int(math.ceil(MIN_GAP_DEG / step)) if step else
                                MIN_GAP_RAYS_FLOOR)

        if GAP_THRESHOLD >= self.max_range:
            raise ValueError(
                f"GAP_THRESHOLD ({GAP_THRESHOLD}) must be below the sensor "
                f"max range ({self.max_range}) or no ray can ever be 'free' — "
                "raise the sensor's Max Range in the Unity Inspector."
            )

        self.state = FORWARD
        self._target_deg = None     # last chosen gap center (hysteresis)
        self._backup_ticks_elapsed = 0
        self._search_side = None    # sticky only within one SEARCH/recovery
        self._search_ticks = 0
        self._pivot_ticks = 0
        self._clear_streak = 0      # consecutive ticks the exit test held
                                    # (debounce — see _debounced_clear)
        self._switch_streak = 0     # consecutive ticks the off-side gap has
                                    # looked clearly better (debounce for
                                    # _choose_gap's sign switch — see there)

    # ── Public API ───────────────────────────────────────────────────────────

    def debug(self) -> dict:
        return {
            "state": self.state,
            "target_deg": None if self._target_deg is None
                          else round(self._target_deg, 1),
            "search_side": self._search_side,
        }

    def step(self, distances) -> tuple:
        d = [min(x, self.max_range) for x in distances]

        # ── BACKUP in progress ───────────────────────────────────────────
        fwd_clear = self._forward_clearance(d)

        # ── BACKUP in progress ───────────────────────────────────────────
        # Adaptive: keep reversing until real clearance opens up (not just
        # a fixed duration), so the next SEARCH/FORWARD attempt starts from
        # a genuinely different position instead of nearly re-touching the
        # same obstacle it just backed away from.
        if self.state == BACKUP:
            self._backup_ticks_elapsed += 1
            reached_min = self._backup_ticks_elapsed >= BACKUP_MIN_TICKS
            cleared = fwd_clear >= BACKUP_CLEAR_TARGET
            timed_out = self._backup_ticks_elapsed >= BACKUP_MAX_TICKS
            if reached_min and (cleared or timed_out):
                self._enter_search(d)       # continues the SAME swing side
            return self._backup_motors()

        # ── Emergency range: reverse first, think later ──────────────────
        if fwd_clear < EMERGENCY_FORWARD:
            return self._enter_backup(d)

        # ── Totally open field: drive straight, forget everything ────────
        # Skips the debounce ONLY when already driving normally -- a single
        # clean tick is plenty of reason to keep going straight. Coming FROM
        # a recovery state (PIVOT/SEARCH) is different: that's exactly the
        # single-tick sensor blip this whole debounce exists for (a nearby
        # corner can slip between two rays for one frame), so recovery
        # states fall through to their own debounced checks below instead
        # of trusting this fast path.
        if all(x >= FULL_CLEAR_FRACTION * self.max_range for x in d):
            if self.state not in (PIVOT, SEARCH):
                self.state = FORWARD
                self._target_deg = None
                self._search_side = None
                return MAX_SPEED, MAX_SPEED

        gaps = self._find_gaps(d)

        # ── PIVOT: rotate in place until the corridor opens ──────────────
        # Replaces backing up when merely inside STOP range: the robot
        # holds its ground and rotates the obstacle out of its corridor.
        # (Backing up here UNDID the escape progress each cycle — with the
        # truck's sluggish physics that produced a stable wiggle loop
        # against a single small cube.)
        if self.state == PIVOT:
            if fwd_clear >= STOP_FORWARD + PIVOT_EXIT_MARGIN and gaps:
                self._clear_streak += 1
                if self._clear_streak >= PIVOT_CLEAR_DEBOUNCE_TICKS:
                    self.state = FORWARD
                    self._pivot_ticks = 0
                    self._clear_streak = 0
                    target = self._choose_gap(gaps)
                    self._target_deg = target
                    return self._steer(target, fwd_clear)
                return self._pivot_motors()   # one clean tick isn't proof yet
            self._clear_streak = 0
            self._pivot_ticks += 1
            if self._pivot_ticks >= PIVOT_TIMEOUT_TICKS:
                return self._enter_backup(d)   # pivot isn't working
            return self._pivot_motors()        # rotate toward swing side

        # ── SEARCH: rotate in place until a usable, roughly-ahead gap ────
        if self.state == SEARCH:
            usable = [g for g in gaps
                      if abs(g["center_deg"]) <= SEARCH_EXIT_ANGLE_DEG]
            if usable and fwd_clear >= SEARCH_EXIT_CLEARANCE:
                self._clear_streak += 1
                if self._clear_streak < SEARCH_CLEAR_DEBOUNCE_TICKS:
                    return self._search_motors()   # one clean tick isn't proof yet
                self._clear_streak = 0
                self.state = FORWARD
                self._target_deg = usable[0]["center_deg"]
                self._search_ticks = 0
            else:
                self._clear_streak = 0
                self._search_ticks += 1
                if self._search_ticks >= SEARCH_FLIP_TICKS:
                    # A full sweep this way found nothing usable — try the
                    # other rotation direction (deliberate, rare, logged).
                    self._search_side = ("right" if self._search_side == "left"
                                         else "left")
                    self._search_ticks = 0
                    print(f"[gap_follow] search direction flipped -> "
                          f"{self._search_side}")
                return self._search_motors()

        # ── FORWARD ──────────────────────────────────────────────────────
        if fwd_clear < STOP_FORWARD:
            # Rotate toward whichever side clears the immediate blockage
            # soonest (see _pivot_side) — NOT toward the long-term gap
            # commitment, which can point the long way around dead-center.
            # Also do NOT back up here (that undoes progress).
            self._search_side = self._pivot_side(d)
            self.state = PIVOT
            self._pivot_ticks = 0
            self._clear_streak = 0
            return self._pivot_motors()

        if not gaps:
            self._enter_search(d)
            return self._search_motors()

        target = self._choose_gap(gaps)
        self._target_deg = target
        return self._steer(target, fwd_clear)

    # ── Geometry ─────────────────────────────────────────────────────────────

    def _forward_clearance(self, d) -> float:
        """
        Distance to the nearest obstacle that is actually in our way:
        for each ray, project its hit point to (lateral, forward); only
        points inside the corridor (|lateral| <= CORRIDOR_HALF) block us.
        A wall grazed by an edge ray but standing BESIDE the corridor
        contributes nothing — this is what kills the wall-oscillation bug.
        """
        best = self.max_range
        for dist, a in zip(d, self.angles_rad):
            if dist >= self.max_range:
                continue                       # nothing on this ray
            lateral = dist * math.sin(a)
            forward = dist * math.cos(a)
            if abs(lateral) <= CORRIDOR_HALF and 0 < forward < best:
                best = forward
        return best

    def _ray_blocked(self, dist: float, a_rad: float) -> bool:
        """A ray is blocked if it reads inside GAP_THRESHOLD — or, even when
        farther, if its obstacle sits inside (or near) our forward corridor
        within SLOW_FORWARD. Without the second clause, a small in-corridor
        obstacle stayed 'free' until quite close, so steering started too
        late for the truck's sluggish physics to matter."""
        if dist < GAP_THRESHOLD:
            return True
        if dist >= self.max_range:
            return False
        lateral = dist * math.sin(a_rad)
        forward = dist * math.cos(a_rad)
        return abs(lateral) <= CORRIDOR_HALF + 0.4 and 0 < forward < SLOW_FORWARD

    def _find_gaps(self, d):
        """Contiguous runs of free rays, wide enough to fit through."""
        gaps, run_start = [], None
        for i, dist in enumerate(d):
            free = not self._ray_blocked(dist, self.angles_rad[i])
            if free and run_start is None:
                run_start = i
            if (not free or i == len(d) - 1) and run_start is not None:
                run_end = i if free else i - 1
                if run_end - run_start + 1 >= self.min_gap_rays:
                    lo, hi = self.angles_deg[run_start], self.angles_deg[run_end]
                    gaps.append({
                        "lo_deg": lo, "hi_deg": hi,
                        "center_deg": (lo + hi) / 2.0,
                        "width": run_end - run_start + 1,
                    })
                run_start = None
        return gaps

    def _choose_gap(self, gaps) -> float:
        """Sign-PREFERRING gap selection (not sign-LOCKING). Once we've
        committed to a direction (|target| >= 5 deg), the on-side gap
        closest to center wins UNLESS the best gap on the OTHER side is
        substantially more direct (by SIGN_SWITCH_MARGIN_DEG or more) — in
        which case we switch.

        Why a margin instead of an absolute lock: an early lock (this
        module's previous version) fixed the original wave bug but went too
        far — once committed, it refused to even LOOK at the opposite side,
        so if the robot had to rotate far enough that the real opening
        ended up on the other side of center, it kept steering at a distant,
        technically-on-side gap instead (the doorway field bug: a doorway
        dead ahead at -10deg was ignored in favor of empty space at +45deg,
        because the robot had committed 'right' many ticks earlier).

        Why not just always pick the closest gap with no preference at all:
        that's what caused the ORIGINAL wave bug — two similarly-placed
        gaps trading places as 'closest' tick to tick. The margin is the
        compromise: small, noise-scale differences don't flip the choice;
        a large, geometrically real difference does.

        The margin alone still isn't enough, though: a single ray grazing
        just the corner of the SAME obstacle can transiently make almost
        the whole scan read free on one side for exactly one tick, which
        looks "clearly better" by the margin test even though nothing
        really changed (field bug: run5, a lone blocked ray flipped the
        commitment on one noisy tick). So the margin condition also has to
        hold for SIGN_SWITCH_DEBOUNCE_TICKS in a row before it's trusted —
        same debounce principle as the PIVOT/SEARCH exit checks."""
        committed = 0
        if self._target_deg is not None and abs(self._target_deg) >= 5:
            committed = 1 if self._target_deg > 0 else -1

        if committed != 0:
            on_side  = [g for g in gaps if g["center_deg"] * committed > 0]
            off_side = [g for g in gaps if g["center_deg"] * committed <= 0]

            best_on = (min(on_side, key=lambda g: (abs(g["center_deg"]), -g["width"]))
                       if on_side else None)
            best_off = (min(off_side, key=lambda g: (abs(g["center_deg"]), -g["width"]))
                        if off_side else None)

            if best_on is None:
                self._switch_streak = 0
                return best_off["center_deg"]      # committed side fully closed
            if best_off is None:
                self._switch_streak = 0
                return best_on["center_deg"]
            if abs(best_off["center_deg"]) + SIGN_SWITCH_MARGIN_DEG < abs(best_on["center_deg"]):
                self._switch_streak += 1
                if self._switch_streak >= SIGN_SWITCH_DEBOUNCE_TICKS:
                    return best_off["center_deg"]   # other side is CLEARLY,
                                                     # REPEATEDLY better
                return best_on["center_deg"]        # not proven yet — stay put
            self._switch_streak = 0
            return best_on["center_deg"]

        self._switch_streak = 0
        best = min(gaps, key=lambda g: (abs(g["center_deg"]), -g["width"]))
        return best["center_deg"]

    # ── State entries ────────────────────────────────────────────────────────

    def _swing_side(self, d) -> str:
        """Which way to rotate/swing for BACKUP/SEARCH recovery: continue
        the side we are already steering to if we have a target, else the
        currently more open half. Derived fresh at each recovery entry — a
        remembered side from an earlier encounter pointed the wrong way and
        made every backup fight the forward steering (the back-and-forth
        wiggle bug)."""
        if self._target_deg is not None:
            return "left" if self._target_deg < 0 else "right"
        return self._more_open_side(d)

    def _pivot_side(self, d) -> str:
        """Which way to rotate for PIVOT specifically: toward whichever
        direction gets the CURRENT blocking obstacle out of the corridor
        soonest. This is a different question from _swing_side's "which
        side am I ultimately routing around" — and conflating them was a
        bug. A pivot only has to rotate until the closest in-corridor
        point's bearing exceeds ~atan(CORRIDOR_HALF / distance); rotating
        AWAY from zero on whichever side the obstacle already leans reaches
        that threshold in the shortest arc. Rotating toward the committed
        gap-following target instead could send the pivot the LONG way
        around (through dead-center, the worst-blocked heading) even when
        the obstacle sat one nudge from clearing on the other side — this
        is what blew through PIVOT_TIMEOUT_TICKS and forced a BACKUP that
        undid the whole approach (the run3 field bug)."""
        blocking = [(dist, a) for dist, a in zip(d, self.angles_rad)
                    if dist < self.max_range
                    and abs(dist * math.sin(a)) <= CORRIDOR_HALF]
        if not blocking:
            return self._swing_side(d)
        # Closest in-corridor point decides — it is the one actually
        # holding fwd_clear down and the one the pivot must clear first.
        _, a = min(blocking, key=lambda pair: pair[0])
        return "left" if a < 0 else "right"

    def _enter_backup(self, d):
        self.state = BACKUP
        self._backup_ticks_elapsed = 0
        self._search_side = self._swing_side(d)
        return self._backup_motors()

    def _enter_search(self, d):
        self.state = SEARCH
        self._search_ticks = 0
        self._clear_streak = 0
        self._target_deg = None
        if self._search_side is None:
            self._search_side = self._more_open_side(d)

    def _more_open_side(self, d) -> str:
        half = len(d) // 2
        left_room = sum(d[:half])            # negative angles = left
        right_room = sum(d[-half:])
        return "left" if left_room >= right_room else "right"

    # ── Motor outputs ────────────────────────────────────────────────────────

    def _steer(self, target_deg: float, fwd_clear: float) -> tuple:
        t = math.radians(target_deg)
        s = max(-1.0, min(1.0, TURN_GAIN * t))    # + = turn right

        frac = (fwd_clear - STOP_FORWARD) / (SLOW_FORWARD - STOP_FORWARD)
        frac = max(0.0, min(1.0, frac))
        speed = max(MIN_FORWARD_SPEED, frac * MAX_SPEED)
        speed *= 1.0 - 0.5 * min(1.0, abs(s))     # slow down in sharp turns

        # Arcade mix with a floor on turn authority. A skid-steer chassis
        # produces NO yaw until the wheel differential beats lateral tire
        # friction — run1.csv showed gentle low-speed arcs (differential
        # ~0.06) not rotating the truck at all, so it crept straight into
        # stop range. MIN_TURN_DIFF keeps every commanded turn effective;
        # the inner wheel may counter-rotate at low speeds.
        turn = 0.0
        if abs(s) > 1e-6:
            turn = max(abs(s) * speed, MIN_TURN_DIFF / 2.0)
            turn = turn if s > 0 else -turn
        left = max(-1.0, min(1.0, speed + turn))
        right = max(-1.0, min(1.0, speed - turn))
        return left, right

    def _backup_motors(self):
        if self._search_side == "left":
            return -BACKUP_SPEED, -BACKUP_SPEED * 0.4   # nose swings left
        return -BACKUP_SPEED * 0.4, -BACKUP_SPEED

    def _search_motors(self):
        if self._search_side == "left":
            return -SEARCH_SPEED, SEARCH_SPEED          # rotate CCW (left)
        return SEARCH_SPEED, -SEARCH_SPEED

    def _pivot_motors(self):
        """Same shape as _search_motors but at PIVOT_SPEED. These used to
        share _search_motors, which only ever used SEARCH_SPEED -- so
        PIVOT_SPEED sat completely unused, and PIVOT's rotation rate was
        silently tied to SEARCH's. That coupling meant speeding up SEARCH
        (to cut down the long rotate-until-clear stalls users were seeing)
        also sped up PIVOT, which has a much tighter tolerance for error
        (it operates close to obstacles, not after backing off to safety)
        and broke a narrow-gap scenario as a direct result. Splitting them
        lets each be tuned for what it actually does."""
        if self._search_side == "left":
            return -PIVOT_SPEED, PIVOT_SPEED
        return PIVOT_SPEED, -PIVOT_SPEED


# ─── Live driving loop ───────────────────────────────────────────────────────

_LOG_FIELDS = ["t", "distances", "motor_l", "motor_r",
               "state", "target_deg", "search_side"]


def drive_with_gap_following(duration: float = 10.0, poll_hz: float = 10.0,
                             log_path: str = None):
    """
    Drive for `duration` seconds using the scan-based gap follower. Stops
    the robot when finished (including on error/interrupt). If log_path is
    given, every tick is recorded to CSV for offline replay_log().
    """
    robot = Robot()
    interval = 1.0 / poll_hz

    scan = get_proximity_scan()
    if scan is None:
        print("[gap_follow] Could not read the proximity scan. Check that "
              "the sim is playing and that ProximitySensor is attached to "
              "the robot GameObject (Tools > Setup Robot Simulator Scene "
              "adds it).")
        return
    controller = GapFollowController(angles_deg=scan["angles_deg"],
                                     max_range=scan["max_range"])

    log_file = writer = None
    if log_path:
        log_file = open(log_path, "w", newline="")
        writer = csv.DictWriter(log_file, fieldnames=_LOG_FIELDS)
        writer.writeheader()

    start = time.time()
    end_time = start + duration
    try:
        while time.time() < end_time:
            if scan is None:
                print("[gap_follow] No scan data — stopping")
                robot.stop()
                time.sleep(interval)
                scan = get_proximity_scan()
                continue

            l_motor, r_motor = controller.step(scan["distances"])
            robot.set_motors(l_motor, r_motor)

            dbg = controller.debug()
            dmin = min(scan["distances"])
            print(f"[gap_follow] min={dmin:.2f} -> "
                  f"motors=({l_motor:.2f}, {r_motor:.2f}) "
                  f"state={dbg['state']} target={dbg['target_deg']} "
                  f"search={dbg['search_side']}")
            if writer:
                writer.writerow({
                    "t": round(time.time() - start, 3),
                    "distances": ";".join(f"{x:.2f}" for x in scan["distances"]),
                    "motor_l": round(l_motor, 3), "motor_r": round(r_motor, 3),
                    **dbg,
                })

            time.sleep(interval)
            scan = get_proximity_scan()
    finally:
        robot.stop()
        if log_file:
            log_file.close()
            print(f"[gap_follow] Telemetry written to {log_path} — replay "
                  f"offline with: gap_follow.replay_log('{log_path}')")


# ─── Offline replay ──────────────────────────────────────────────────────────


def replay_log(csv_path: str, max_range: float = None, verbose: bool = True):
    """
    Re-run a recorded scan sequence through a FRESH controller and print
    each decision — reproduce a misbehaving run entirely offline, or verify
    that a code change fixes it against the exact same sensor data.
    """
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"[replay] {csv_path} is empty")
        return []

    first = [float(x) for x in rows[0]["distances"].split(";")]
    step = SCAN_FOV_DEG / (len(first) - 1)
    angles = [-SCAN_FOV_DEG / 2 + i * step for i in range(len(first))]
    controller = GapFollowController(angles_deg=angles, max_range=max_range)

    out = []
    for row in rows:
        d = [float(x) for x in row["distances"].split(";")]
        motors = controller.step(d)
        dbg = controller.debug()
        out.append((d, motors, dbg))
        if verbose:
            print(f"[replay] t={row['t']} min={min(d):.2f} -> "
                  f"motors=({motors[0]:.2f}, {motors[1]:.2f}) "
                  f"state={dbg['state']} target={dbg['target_deg']} "
                  f"(recorded: {row['motor_l']}, {row['motor_r']}, {row['state']})")
    return out