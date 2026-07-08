"""
jetbot_nav.navigation — obstacle-avoidance driving loop.

Reads a 3-sensor proximity array (left, center, right — the sim
equivalent of a small ultrasonic/IR array a real JetBot could carry)
and drives forward, slowing down and steering toward whichever side
has more room as an obstacle gets closer. Backs up and re-orients if
it gets too close to recover from a bad approach angle.

This module only calls sim_client.send_query()/send_command() and does
simple math on the results — no simulation-only logic — so it runs
the same way against the real Jetson's sensor array, as long as the
physical robot exposes get_proximity() the same shape:
    {"status": "ok", "left": <meters>, "center": <meters>, "right": <meters>}

Typical usage:

    from jetbot_nav import navigation

    navigation.drive_with_avoidance(duration=10.0)
"""

import time

import sim_client
from jetbot import Robot

# ─── Tuning constants ────────────────────────────────────────────────────────

SLOW_DOWN_DISTANCE = 8.0   # meters — start slowing/steering inside this range
STOP_DISTANCE       = 3.0  # meters — treat as "blocked", steer hard
WALL_MARGIN         = 0.3  # if left/right are also within STOP_DISTANCE+this,
                            # treat it as a wide obstacle/wall, not a passable gap
CLEAR_MARGIN        = 0.5  # left/right must be this far beyond STOP_DISTANCE
                            # before resuming straight driving
CLEAR_HOLD_TICKS    = 6    # must see "clear enough" for this many CONSECUTIVE
                            # ticks (~0.6s) before actually going straight —
                            # a single clean instant isn't trusted, since that's
                            # what was causing it to straighten out while still
                            # angled toward the wall and clip it again shortly after
MAX_SPEED           = 0.5  # motor value at full speed, 0..1
MIN_SPEED           = 0.0  # motor value floor — 0 lets it fully stop
MAX_TURN_FRACTION   = 0.9  # steepest turn allowed (near-total wheel cut) at max closeness
COMMIT_TICKS        = 8    # ticks (~0.8s) to hold a turn direction before
                            # re-evaluating — prevents rapid left/right flapping

BACKUP_DISTANCE      = STOP_DISTANCE + 0.15  # trigger backup at/inside this range
BACKUP_SPEED         = 0.3   # reverse speed while backing up
BACKUP_DURATION_TICKS = 6    # how long to reverse before re-attempting (~0.6s)


def get_proximity() -> dict:
    """
    Query the sensor array. Returns {"left": m, "center": m, "right": m}
    (each in meters, or the sensor's max range if nothing is detected).
    Returns None if the query failed (e.g. sim not running).
    """
    resp = sim_client.send_query({"command": "get_proximity"})
    if not resp or resp.get("status") != "ok":
        return None
    return {"left": resp["left"], "center": resp["center"], "right": resp["right"]}


def _speed_for_distance(distance: float) -> float:
    """Linearly scale speed down as distance shrinks from SLOW_DOWN_DISTANCE to STOP_DISTANCE."""
    if distance >= SLOW_DOWN_DISTANCE:
        return MAX_SPEED
    if distance <= STOP_DISTANCE:
        return MIN_SPEED

    span = SLOW_DOWN_DISTANCE - STOP_DISTANCE
    frac = (distance - STOP_DISTANCE) / span  # 0..1
    return MIN_SPEED + frac * (MAX_SPEED - MIN_SPEED)


class AvoidanceController:
    """
    Stateful obstacle avoidance with three layers of memory:

      1. Commit to a turn direction and hold it (COMMIT_TICKS) instead of
         re-deciding every tick — prevents flapping on wide/even obstacles.
      2. Require SUSTAINED clearance (CLEAR_HOLD_TICKS in a row) before
         declaring "clear" and going straight — prevents snapping back to
         full speed while still angled toward the obstacle, which was
         causing it to clip the same wall again a moment later.
      3. If something gets too close (BACKUP_DISTANCE), stop trying to
         steer out of it and instead reverse for a short window, turning
         away from the blocked side while backing up, then re-attempt.
    """

    def __init__(self):
        self._committed_side = None   # "left" or "right" or None
        self._commit_ticks_left = 0
        self._clear_streak = 0
        self._backup_ticks_left = 0
        self._backup_side = None

    def reset(self):
        self._committed_side = None
        self._commit_ticks_left = 0
        self._clear_streak = 0
        self._backup_ticks_left = 0
        self._backup_side = None

    def step(self, prox: dict) -> tuple:
        left_d, center_d, right_d = prox["left"], prox["center"], prox["right"]

        # ── Backing up takes priority over everything else ──────────────
        if self._backup_ticks_left > 0:
            self._backup_ticks_left -= 1
            if self._backup_ticks_left == 0:
                # Backup just finished — carry the same escape direction
                # straight into the turn that follows, instead of letting
                # the steering logic below re-derive a direction from raw
                # sensor values (which is what was causing it to sometimes
                # flip direction right after backing up).
                self._committed_side = self._backup_side
                self._commit_ticks_left = COMMIT_TICKS
            # Reverse while turning away from the blocked side, so it's
            # re-orienting during the backup, not just retreating straight.
            if self._backup_side == "left":
                return -BACKUP_SPEED, -BACKUP_SPEED * 0.4
            else:
                return -BACKUP_SPEED * 0.4, -BACKUP_SPEED

        too_close = (
            center_d <= BACKUP_DISTANCE
            or left_d  <= BACKUP_DISTANCE
            or right_d <= BACKUP_DISTANCE
        )
        if too_close:
            # Prefer continuing whatever direction we were already steering
            # toward — recomputing from raw left/right distances at the
            # moment of contact is noisy (a roughly symmetric wall can flip
            # which side reads as "more open" from one attempt to the next,
            # which was causing it to alternate escape directions forever
            # instead of committing to one and actually clearing it).
            self._backup_side = self._committed_side or (
                "left" if left_d < right_d else "right"
            )
            self._backup_ticks_left = BACKUP_DURATION_TICKS
            self._committed_side = None
            self._commit_ticks_left = 0
            self._clear_streak = 0
            if self._backup_side == "left":
                return -BACKUP_SPEED, -BACKUP_SPEED * 0.4
            else:
                return -BACKUP_SPEED * 0.4, -BACKUP_SPEED

        base_speed = _speed_for_distance(center_d)

        instant_clear = (
            center_d >= SLOW_DOWN_DISTANCE
            and left_d  >= STOP_DISTANCE + CLEAR_MARGIN
            and right_d >= STOP_DISTANCE + CLEAR_MARGIN
        )
        if instant_clear:
            self._clear_streak += 1
        else:
            self._clear_streak = 0

        if self._clear_streak >= CLEAR_HOLD_TICKS:
            self.reset()
            return MAX_SPEED, MAX_SPEED

        is_wide_obstacle = (
            center_d <= STOP_DISTANCE + WALL_MARGIN
            and left_d  <= STOP_DISTANCE + WALL_MARGIN
            and right_d <= STOP_DISTANCE + WALL_MARGIN
        )

        if self._commit_ticks_left > 0 and self._committed_side is not None:
            committed_dist = left_d if self._committed_side == "left" else right_d
            if committed_dist <= STOP_DISTANCE:
                self._committed_side = None
                self._commit_ticks_left = 0
            else:
                self._commit_ticks_left -= 1

        if self._committed_side is None:
            self._committed_side = "left" if left_d > right_d else "right"
            self._commit_ticks_left = COMMIT_TICKS

        # Purely proportional to how close the nearest relevant reading is —
        # no fixed floor. This means turn strength genuinely reaches ~0 as
        # readings approach SLOW_DOWN_DISTANCE, instead of always applying
        # at least STEER_BIAS worth of turn even when nothing's really
        # ahead (that floor was why it kept turning after clearing).
        nearest = min(center_d, left_d, right_d)
        closeness = 1.0 - (nearest - STOP_DISTANCE) / (SLOW_DOWN_DISTANCE - STOP_DISTANCE)
        closeness = max(0.0, min(1.0, closeness))
        turn_fraction = closeness * MAX_TURN_FRACTION
        slow_side = base_speed * (1.0 - turn_fraction)

        if is_wide_obstacle:
            pivot_speed = max(MAX_SPEED * 0.3, 0.15)
            if self._committed_side == "left":
                return 0.0, pivot_speed
            else:
                return pivot_speed, 0.0

        if self._committed_side == "left":
            return slow_side, base_speed
        else:
            return base_speed, slow_side


def compute_motor_speeds(prox: dict, controller: AvoidanceController = None) -> tuple:
    """
    Convenience wrapper for one-off testing. For real driving, use a single
    persistent AvoidanceController via drive_with_avoidance() (or your own
    loop) so the commit/hysteresis/backup state carries across ticks.
    """
    controller = controller or AvoidanceController()
    return controller.step(prox)


def drive_with_avoidance(duration: float = 10.0, poll_hz: float = 10.0):
    """
    Drive forward for `duration` seconds, continuously adjusting speed
    and steering based on the proximity sensor array. Stops the robot
    when finished (including on error/interrupt).
    """
    robot = Robot()
    controller = AvoidanceController()
    interval = 1.0 / poll_hz
    end_time = time.time() + duration

    try:
        while time.time() < end_time:
            prox = get_proximity()
            if prox is None:
                print("[navigation] No proximity data — stopping")
                robot.stop()
                time.sleep(interval)
                continue

            left, right = controller.step(prox)
            robot.set_motors(left, right)

            state = "BACKUP" if controller._backup_ticks_left > 0 else controller._committed_side
            print(
                f"[navigation] L={prox['left']:.2f}m C={prox['center']:.2f}m "
                f"R={prox['right']:.2f}m -> motors=({left:.2f}, {right:.2f}) "
                f"state={state} clear_streak={controller._clear_streak}"
            )
            time.sleep(interval)
    finally:
        robot.stop()