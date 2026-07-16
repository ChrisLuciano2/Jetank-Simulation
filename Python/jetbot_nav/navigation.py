"""
jetbot_nav.navigation — obstacle-avoidance driving loop (state-machine version).

Reads a 3-sensor proximity array (left, center, right — the sim equivalent of
a small ultrasonic/IR array a real JetBot could carry) and drives forward,
steering around obstacles, backing up to recover from bad approach angles,
and returning to straight-line driving once the path is clear.

This module only calls sim_client.send_query()/send_command() and does simple
math on the results — no simulation-only logic — so it runs the same way
against the real Jetson's sensor array, as long as the physical robot exposes
get_proximity() with the same shape:

    {"status": "ok", "left": <m>, "center": <m>, "right": <m>, "max_range": <m>}

("max_range" is optional but strongly recommended — see MAX-RANGE HANDLING.)

──────────────────────────────────────────────────────────────────────────────
DESIGN — explicit state machine (replaces the previous implicit-state logic)

    CRUISE  — path clear, drive straight at MAX_SPEED
    AVOID   — obstacle in the slow-down band: arc forward toward the
              committed escape side, speed proportional to center distance
    PIVOT   — center blocked (< STOP_DISTANCE): rotate in place toward the
              escape side until the center sensor opens up
    BACKUP  — something inside BACKUP_DISTANCE: reverse for a short window
              while swinging the nose toward the escape side, then PIVOT

Key behavioral rules (each fixes a specific observed bug — see
test_avoidance_logic.py for the regression test that encodes it):

  1. ESCAPE DIRECTION IS PER-ENCOUNTER, NOT PER-TICK.  The escape side is
     chosen once when an obstacle encounter begins and held through every
     AVOID/PIVOT/BACKUP cycle of that encounter.  It is *never* re-derived
     from raw left/right readings mid-encounter — while pivoting next to a
     wall, the wall-side sensor sweeps close and raw readings flip which
     side looks "more open" from one tick to the next, which is what caused
     the infinite direction-flipping loop against large walls.

  2. THE ONLY WAY TO FLIP is the failed-attempt counter: if
     MAX_BACKUP_ATTEMPTS backups happen within one encounter without
     reaching CRUISE, the chosen direction is demonstrably not working
     (e.g. we picked the short way out of a corner) — flip once, reset the
     counter, and persist with the new direction just as stubbornly.

  3. TURN STRENGTH COMES FROM THE FORWARD PATH, NOT TRAILING SENSORS.
     While passing a cleared block, the side sensor facing it still reads
     close — that must not keep forcing a turn (that lingering turn was
     half of the "keeps turning after clearing" bug; the other half was a
     sensor-range mismatch, see below).  Turn strength is driven by the
     center reading, plus the obstacle-side reading only when it is close
     enough (SIDE_THREAT_DISTANCE) to actually clip the robot's corner.

  4. SUSTAINED CLEARANCE before going straight (CLEAR_HOLD_TICKS in a row) —
     a single clean instant isn't trusted, since that caused straightening
     while still angled toward the wall.

──────────────────────────────────────────────────────────────────────────────
MAX-RANGE HANDLING (root cause of the "keeps turning when clear" bug)

The sensor reports its max range when nothing is detected.  If Python's
SLOW_DOWN_DISTANCE is >= that max range, "completely clear" is
indistinguishable from "obstacle at max range" and the robot steers forever.
That exact mismatch shipped once (sensor maxRange 5.0 vs SLOW_DOWN 8.0).

Defenses now in place:
  - Unity's get_proximity response includes "max_range"; the controller
    clamps its slow-down band to 90% of it and prints a loud warning if it
    had to clamp.
  - If "max_range" is missing (old Unity build), we fall back to
    LEGACY_MAX_RANGE = 5.0 — the value known to be serialized in older
    scenes — and warn.  Update the ProximitySensor's Max Range in the Unity
    Inspector to 12 to get the full look-ahead distance.

──────────────────────────────────────────────────────────────────────────────
STRAIGHT vs. RETRACE after clearing an obstacle

Once clear, the robot resumes driving STRAIGHT on its new heading; it does
not try to return to its original line.  Reasons:
  - The physical JetBot has no odometry, encoders, or compass; retracing
    requires dead reckoning, which diverges quickly on real hardware
    (track slip), violating the sim/hardware-parity rule.
  - The upcoming "pathfinding to detected items" feature will steer toward a
    camera-visible target after every avoidance anyway, so course
    correction becomes emergent and sensor-driven rather than dead-reckoned.
The controller ends every encounter in CRUISE with all memory reset, so a
future target-seeking layer can simply take over steering at that point.

Typical usage:

    from jetbot_nav import navigation
    navigation.drive_with_avoidance(duration=10.0)

Troubleshooting workflow:

    navigation.drive_with_avoidance(duration=20, log_path="run1.csv")
    # ... robot misbehaves ...
    navigation.replay_log("run1.csv")   # re-run decisions offline, no Unity
"""

import csv
import time

import sim_client
from jetbot import Robot

# ─── Tuning constants ────────────────────────────────────────────────────────

DEFAULT_MAX_RANGE   = 12.0  # expected sensor max range (Unity Inspector value)
LEGACY_MAX_RANGE    = 5.0   # fallback if Unity doesn't report max_range at all

STOP_DISTANCE       = 3.0   # meters — center below this = forward path blocked
SLOW_DOWN_DISTANCE  = 8.0   # start slowing/steering inside this range
                            # (auto-clamped to 0.9 * sensor max at runtime)
BACKUP_DISTANCE     = STOP_DISTANCE + 0.15  # any sensor at/inside this → BACKUP
SIDE_THREAT_DISTANCE = STOP_DISTANCE + 1.0  # side readings inside this add turn
                            # (outside it, a side reading is "beside us, not in
                            # our way" and must NOT keep forcing a turn)

CLEAR_SIDE_MARGIN   = 0.5   # sides must exceed STOP_DISTANCE by this to count
                            # toward a "clear" tick
CLEAR_HOLD_TICKS    = 6     # consecutive clear ticks (~0.6s) before CRUISE

MAX_SPEED           = 0.5   # motor value at full speed, 0..1
MIN_AVOID_SPEED     = 0.15  # forward-speed floor while arcing in AVOID
MAX_TURN_FRACTION   = 0.9   # steepest arc (near-total inner-wheel cut)

BACKUP_SPEED          = 0.3
BACKUP_DURATION_TICKS = 6   # ~0.6s of reversing

PIVOT_SPEED         = 0.18  # wheel speed while rotating in place
PIVOT_EXIT_DISTANCE = STOP_DISTANCE + 1.5   # center must open to this before
                                            # driving forward again
PIVOT_TIMEOUT_TICKS = 40    # ~4s spinning without the center opening counts
                            # as a failed attempt → BACKUP

MAX_BACKUP_ATTEMPTS = 3     # failed attempts in one encounter before the
                            # escape direction is flipped (corner escape)

# ─── Sensor query ────────────────────────────────────────────────────────────


def get_proximity() -> dict:
    """
    Query the sensor array. Returns
        {"left": m, "center": m, "right": m, "max_range": m-or-None}
    or None if the query failed (e.g. sim not running).
    """
    resp = sim_client.send_query({"command": "get_proximity"})
    if not resp or resp.get("status") != "ok":
        return None
    return {
        "left":      resp["left"],
        "center":    resp["center"],
        "right":     resp["right"],
        "max_range": resp.get("max_range"),  # absent on older Unity builds
    }


# ─── Controller ──────────────────────────────────────────────────────────────

CRUISE, AVOID, PIVOT, BACKUP = "CRUISE", "AVOID", "PIVOT", "BACKUP"


class AvoidanceController:
    """
    Stateful obstacle avoidance. Feed it one sensor reading per tick via
    step(); it returns (left_motor, right_motor).

    Pure function of (internal state, sensor input) — no I/O — so it can be
    unit-tested and replayed offline (see test_avoidance_logic.py and
    replay_log()).
    """

    def __init__(self, max_range: float = None):
        self.state = CRUISE
        self.escape_side = None       # "left" / "right", persists per encounter
        self.backup_attempts = 0      # failed attempts within this encounter
        self.flips = 0                # times the escape side was flipped
        self._clear_streak = 0
        self._backup_ticks_left = 0
        self._pivot_ticks = 0

        self._configure_ranges(max_range)

    # ── Range configuration / sanity checks ─────────────────────────────────

    def _configure_ranges(self, max_range):
        if max_range is None:
            print(
                "[navigation] WARNING: Unity did not report the sensor's "
                f"max_range — assuming legacy value {LEGACY_MAX_RANGE}. "
                "Update SimQueryServer/ProximitySensor so get_proximity "
                "includes max_range, and set Max Range to ~12 in the "
                "Inspector for better look-ahead."
            )
            max_range = LEGACY_MAX_RANGE

        self.max_range = float(max_range)
        self.slow_down = min(SLOW_DOWN_DISTANCE, 0.9 * self.max_range)
        if self.slow_down < SLOW_DOWN_DISTANCE:
            print(
                f"[navigation] WARNING: SLOW_DOWN_DISTANCE ({SLOW_DOWN_DISTANCE}) "
                f">= sensor max range ({self.max_range}). Clamped slow-down "
                f"band to {self.slow_down:.2f} so 'clear' stays detectable. "
                "Raise the sensor's Max Range in the Unity Inspector to "
                "restore full look-ahead."
            )
        if self.slow_down <= STOP_DISTANCE:
            raise ValueError(
                f"Sensor max range {self.max_range} leaves no usable band "
                f"above STOP_DISTANCE {STOP_DISTANCE} — raise the sensor's "
                "Max Range in Unity."
            )

    # ── Public API ───────────────────────────────────────────────────────────

    def reset(self):
        """Full reset — encounter over."""
        self.state = CRUISE
        self.escape_side = None
        self.backup_attempts = 0
        self._clear_streak = 0
        self._backup_ticks_left = 0
        self._pivot_ticks = 0

    def debug(self) -> dict:
        """Snapshot of internal state for logging/telemetry."""
        return {
            "state": self.state,
            "escape_side": self.escape_side,
            "backup_attempts": self.backup_attempts,
            "flips": self.flips,
            "clear_streak": self._clear_streak,
        }

    def step(self, prox: dict) -> tuple:
        left_d, center_d, right_d = prox["left"], prox["center"], prox["right"]

        # ── BACKUP in progress: finish it before evaluating anything else ──
        if self.state == BACKUP:
            self._backup_ticks_left -= 1
            if self._backup_ticks_left <= 0:
                self.state = PIVOT
                self._pivot_ticks = 0
            return self._backup_motors()

        # ── Emergency: anything too close → start a BACKUP ─────────────────
        if min(left_d, center_d, right_d) <= BACKUP_DISTANCE:
            self._begin_backup(left_d, right_d)
            return self._backup_motors()

        # ── Clearance bookkeeping (evaluated in every non-backup state) ────
        if self._is_clear(left_d, center_d, right_d):
            self._clear_streak += 1
        else:
            self._clear_streak = 0

        if self._clear_streak >= CLEAR_HOLD_TICKS:
            self.reset()
            return MAX_SPEED, MAX_SPEED

        # ── State logic ─────────────────────────────────────────────────────
        if self.state == CRUISE:
            if self._obstacle_ahead(left_d, center_d, right_d):
                self._begin_encounter(left_d, right_d)
                self.state = AVOID
            else:
                return MAX_SPEED, MAX_SPEED

        if self.state == PIVOT:
            if center_d >= PIVOT_EXIT_DISTANCE:
                self.state = AVOID                     # forward path open again
            else:
                self._pivot_ticks += 1
                if self._pivot_ticks >= PIVOT_TIMEOUT_TICKS:
                    # Spun for a while without the center opening — this
                    # attempt has failed; back up and try again (the attempt
                    # counter may flip the escape side).
                    self._begin_backup(left_d, right_d)
                    return self._backup_motors()
                return self._pivot_motors()

        # AVOID (also reached from CRUISE/PIVOT transitions above)
        if center_d <= STOP_DISTANCE:
            # Forward path blocked but nothing in backup range yet —
            # rotate in place instead of arcing into it.
            self.state = PIVOT
            self._pivot_ticks = 0
            return self._pivot_motors()

        return self._avoid_motors(left_d, center_d, right_d)

    # ── Encounter / backup management ────────────────────────────────────────

    def _begin_encounter(self, left_d, right_d):
        """Called once when an obstacle encounter starts. Direction is chosen
        HERE and only here (or by the failed-attempt flip)."""
        if self.escape_side is None:
            self.escape_side = "left" if left_d > right_d else "right"
            self.backup_attempts = 0

    def _begin_backup(self, left_d, right_d):
        # Ensure a direction exists even if backup fires straight from CRUISE
        # (e.g. something appeared very close very fast).
        self._begin_encounter(left_d, right_d)

        self.backup_attempts += 1
        if self.backup_attempts >= MAX_BACKUP_ATTEMPTS:
            # Persistent failure in this direction — flip ONCE and persist
            # with the new one. This is the ONLY mid-encounter direction
            # change (fixes the wall flip-flop loop).
            self.escape_side = "right" if self.escape_side == "left" else "left"
            self.backup_attempts = 0
            self.flips += 1
            print(f"[navigation] escape direction flipped -> {self.escape_side} "
                  f"(previous direction failed {MAX_BACKUP_ATTEMPTS} attempts)")

        self.state = BACKUP
        self._backup_ticks_left = BACKUP_DURATION_TICKS
        self._clear_streak = 0

    # ── Predicates ───────────────────────────────────────────────────────────

    def _obstacle_ahead(self, left_d, center_d, right_d):
        return (
            center_d < self.slow_down
            or min(left_d, right_d) < SIDE_THREAT_DISTANCE
        )

    def _is_clear(self, left_d, center_d, right_d):
        return (
            center_d >= self.slow_down
            and left_d  >= STOP_DISTANCE + CLEAR_SIDE_MARGIN
            and right_d >= STOP_DISTANCE + CLEAR_SIDE_MARGIN
        )

    # ── Motor outputs ────────────────────────────────────────────────────────

    def _backup_motors(self):
        # Reverse while swinging the nose toward the escape side:
        # (-fast, -slow) rotates the nose left; mirrored for right.
        if self.escape_side == "left":
            return -BACKUP_SPEED, -BACKUP_SPEED * 0.4
        return -BACKUP_SPEED * 0.4, -BACKUP_SPEED

    def _pivot_motors(self):
        # Rotate in place toward the escape side. NOTE: while pivoting next
        # to a wall, the wall-side sensor WILL read close — that is expected
        # and deliberately ignored (judging progress by the center sensor is
        # what prevents the direction flip-flop).
        if self.escape_side == "left":
            return -PIVOT_SPEED, PIVOT_SPEED
        return PIVOT_SPEED, -PIVOT_SPEED

    def _avoid_motors(self, left_d, center_d, right_d):
        # Forward speed from how open the center is.
        span = self.slow_down - STOP_DISTANCE
        frac = (center_d - STOP_DISTANCE) / span
        frac = max(0.0, min(1.0, frac))
        speed = max(MIN_AVOID_SPEED, frac * MAX_SPEED)

        # Turn strength: how blocked is the FORWARD path?
        center_closeness = 1.0 - frac

        # The obstacle-side sensor only adds turn when close enough to clip
        # the robot's corner. A block we've already passed sitting 3.5m off
        # to the side contributes nothing — so the robot straightens out
        # instead of orbiting it.
        obstacle_d = right_d if self.escape_side == "left" else left_d
        side_span = SIDE_THREAT_DISTANCE - STOP_DISTANCE
        side_closeness = 1.0 - (obstacle_d - STOP_DISTANCE) / side_span
        side_closeness = max(0.0, min(1.0, side_closeness))

        turn_fraction = max(center_closeness, side_closeness) * MAX_TURN_FRACTION
        inner = speed * (1.0 - turn_fraction)

        if self.escape_side == "left":
            return inner, speed
        return speed, inner


# ─── Convenience wrapper (kept for backward compatibility) ───────────────────


def compute_motor_speeds(prox: dict, controller: AvoidanceController = None) -> tuple:
    """
    One-off convenience for quick tests. For real driving use a single
    persistent AvoidanceController (or drive_with_avoidance) so the
    encounter/backup state carries across ticks.
    """
    controller = controller or AvoidanceController(max_range=prox.get("max_range"))
    return controller.step(prox)


# ─── Live driving loop ───────────────────────────────────────────────────────

_LOG_FIELDS = ["t", "left", "center", "right", "motor_l", "motor_r",
               "state", "escape_side", "backup_attempts", "flips",
               "clear_streak"]


def drive_with_avoidance(duration: float = 10.0, poll_hz: float = 10.0,
                         log_path: str = None):
    """
    Drive forward for `duration` seconds with obstacle avoidance. Stops the
    robot when finished (including on error/interrupt).

    If log_path is given, every tick is appended to a CSV (sensors, motors,
    controller state) that replay_log() can re-run offline — record a run
    that misbehaves, then debug it without Unity.
    """
    robot = Robot()
    interval = 1.0 / poll_hz

    # First reading also tells us the sensor's max range.
    first = get_proximity()
    if first is None:
        print("[navigation] Could not read proximity sensor — is the sim "
              "running with ProximitySensor attached?")
        return
    controller = AvoidanceController(max_range=first.get("max_range"))

    log_file = writer = None
    if log_path:
        log_file = open(log_path, "w", newline="")
        writer = csv.DictWriter(log_file, fieldnames=_LOG_FIELDS)
        writer.writeheader()

    start = time.time()
    end_time = start + duration
    prox = first

    try:
        while time.time() < end_time:
            if prox is None:
                print("[navigation] No proximity data — stopping")
                robot.stop()
                time.sleep(interval)
                prox = get_proximity()
                continue

            l_motor, r_motor = controller.step(prox)
            robot.set_motors(l_motor, r_motor)

            dbg = controller.debug()
            print(
                f"[navigation] L={prox['left']:.2f} C={prox['center']:.2f} "
                f"R={prox['right']:.2f} -> motors=({l_motor:.2f}, {r_motor:.2f}) "
                f"state={dbg['state']} escape={dbg['escape_side']} "
                f"attempts={dbg['backup_attempts']} clear={dbg['clear_streak']}"
            )
            if writer:
                writer.writerow({
                    "t": round(time.time() - start, 3),
                    "left": prox["left"], "center": prox["center"],
                    "right": prox["right"],
                    "motor_l": round(l_motor, 3), "motor_r": round(r_motor, 3),
                    **dbg,
                })

            time.sleep(interval)
            prox = get_proximity()
    finally:
        robot.stop()
        if log_file:
            log_file.close()
            print(f"[navigation] Telemetry written to {log_path} — replay "
                  f"offline with: navigation.replay_log('{log_path}')")


# ─── Offline replay (troubleshooting without Unity) ──────────────────────────


def replay_log(csv_path: str, max_range: float = None, verbose: bool = True):
    """
    Feed the sensor readings from a recorded run back through a FRESH
    controller and print each decision. Lets you reproduce and step through
    a misbehaving run entirely offline (and check whether a code change
    would have fixed it, by replaying the same sensors against new logic).

    Returns the list of (prox, motors, debug) tuples for programmatic checks.
    """
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"[replay] {csv_path} is empty")
        return []

    controller = AvoidanceController(max_range=max_range)
    out = []
    for row in rows:
        prox = {"left": float(row["left"]), "center": float(row["center"]),
                "right": float(row["right"])}
        motors = controller.step(prox)
        dbg = controller.debug()
        out.append((prox, motors, dbg))
        if verbose:
            recorded = f"recorded=({row['motor_l']}, {row['motor_r']}, {row['state']})"
            print(f"[replay] t={row['t']} L={prox['left']:.2f} "
                  f"C={prox['center']:.2f} R={prox['right']:.2f} -> "
                  f"motors=({motors[0]:.2f}, {motors[1]:.2f}) "
                  f"state={dbg['state']} escape={dbg['escape_side']}  {recorded}")
    return out