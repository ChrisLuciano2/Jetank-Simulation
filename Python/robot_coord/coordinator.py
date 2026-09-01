"""
robot_coord.coordinator — High-level API a build script calls to stay in
sync with the other robot (brainstorm doc sections 5 & 6).

Two pacing mechanisms, because they guard against two different failure
modes (doc section 5's recommendation):
  - Bounded slack: caps how far ahead one robot's placed-block count can
    get before it must wait. Guards against overall imbalance/instability.
  - Exclusion zone: only one robot may be within `exclusion_radius` of the
    meeting point at a time. Guards against arm/base collision at the seam
    where the two build-fronts meet.

Default behavior on a stale/unknown peer is to keep waiting, not to treat
silence as clearance — doc section 6 is explicit about this: "if a robot
hasn't updated its state ... the other robot should pause rather than
assume progress," and "if the shared communication channel drops, default
behavior should be pause and wait, not keep building blind." Every
wait_for_*() call still times out (returns False) rather than blocking
forever, so a genuine stall surfaces to the caller's own retry/abort logic
instead of hanging the script — but timing out is a *reported failure*,
not a silent green light.

Set solo_ok=True to invert that default for one-robot dev iteration (no
second process running yet): stale/unknown peer then means "nothing to
wait on," so you can exercise the build loop alone without a partner
process. Leave it False for anything that's supposed to be a real
two-robot run, including in simulation — a solo run silently passing
every coordination check is not a validation of the coordination logic.
"""

import math
import time

from . import config, messages
from .world_model import SharedWorldModel


class Coordinator:
    def __init__(self, meeting_point, max_slack=1, exclusion_radius=0.5,
                 robot_id=None, world_model=None, solo_ok=False,
                 poll_interval=0.25, wait_timeout=60.0):
        """
        meeting_point: {"x": float, "y": float} — the midline both robots
            build toward (doc section 4's "single shared meeting point").
        max_slack: how many more blocks this robot may place than the
            other before wait_for_pacing_clearance() blocks. 1 is close to
            lockstep — raise it once you trust the system (doc section 5).
        exclusion_radius: distance from meeting_point that counts as "the
            seam" for wait_for_zone_clearance()'s single-occupant rule.
        solo_ok: if True, a stale/never-seen peer counts as clearance
            (fail open) instead of something to keep waiting on. Only for
            one-robot dev runs — see module docstring.
        wait_timeout: max seconds any wait_for_*() call blocks before
            giving up and returning False. Tune this well above one
            build-cycle's expected duration — too short and a slow-but-fine
            cycle looks like a stall; too long and a real stall wastes
            real time before your script's own retry/abort logic sees it.
        """
        self.robot_id = robot_id or config.ROBOT_ID
        self.meeting_point = dict(meeting_point)
        self.max_slack = max_slack
        self.exclusion_radius = exclusion_radius
        self.solo_ok = solo_ok
        self.poll_interval = poll_interval
        self.wait_timeout = wait_timeout
        self.world = world_model or SharedWorldModel(robot_id=self.robot_id)

    def connect(self):
        self.world.connect()

    def disconnect(self):
        self.world.disconnect()

    # ── Publishing ──────────────────────────────────────────────────────

    def publish_state(self, state, pose, blocks_placed=0, current_action=None,
                       detected_structure_top=None, flags=None):
        msg = messages.build_state_message(
            self.robot_id, state, pose,
            blocks_placed=blocks_placed,
            current_action=current_action,
            detected_structure_top=detected_structure_top,
            flags=flags,
        )
        self.world.publish_state(msg)
        return msg

    def flag_defect(self, flag, location=None, note=""):
        """Section 6: misalignment / assist-request flagging."""
        msg = messages.build_flag_message(self.robot_id, flag, location, note)
        self.world.publish_flag(msg)
        return msg

    # ── Bounded slack (section 5, mechanism 2) ───────────────────────────

    def wait_for_pacing_clearance(self, my_blocks_placed):
        """
        Block until my_blocks_placed minus the other robot's reported
        blocks_placed is <= max_slack. Call this *before* starting the
        next block, passing the count including the block you're about
        to start (i.e. "if I finish this one, how far ahead am I?").

        Returns True once clear. Returns False if wait_timeout elapses
        first — including while waiting on a peer that's stale or has
        never been heard from, unless solo_ok=True (see class docstring).
        """
        def _clear():
            other = self._other_state_or_none()
            if other is None:
                return self.solo_ok
            other_count = other.get("blocks_placed", 0)
            return my_blocks_placed - other_count <= self.max_slack

        return self._poll_until(_clear)

    # ── Exclusion zone (section 5, mechanism 4) ──────────────────────────

    def wait_for_zone_clearance(self, my_pose):
        """
        Block until the meeting-point exclusion zone is free, given the
        pose you're about to move to. If that pose isn't within
        exclusion_radius of meeting_point, returns True immediately —
        this only guards the seam, not the whole workspace.
        """
        if self._distance(my_pose, self.meeting_point) > self.exclusion_radius:
            return True

        def _clear():
            other = self._other_state_or_none()
            if other is None:
                return self.solo_ok
            other_pose = other.get("pose")
            if other_pose is None:
                return self.solo_ok
            return self._distance(other_pose, self.meeting_point) > self.exclusion_radius

        return self._poll_until(_clear)

    # ── Placement clearance (section 6) ──────────────────────────────────

    def wait_for_placement_clear(self):
        """
        Block until the other robot has signaled it's clear of the shared
        workspace (state in {"placed_clear", "idle", "inspecting"}). This
        is the arm-to-arm / arm-to-structure collision guard from section
        6 — call it before approaching the structure to place a block.
        """
        def _clear():
            other = self._other_state_or_none()
            if other is None:
                return self.solo_ok
            return other.get("state") in ("placed_clear", "idle", "inspecting")

        return self._poll_until(_clear)

    # ── Internal helpers ──────────────────────────────────────────────────

    def _poll_until(self, is_clear):
        deadline = time.time() + self.wait_timeout
        while True:
            if is_clear():
                return True
            if time.time() >= deadline:
                return False
            time.sleep(self.poll_interval)

    def other_robot_id(self):
        """The other robot's id, or None if we've never heard from one.
        Public because callers building on top of the state model (e.g.
        shared tower-height bookkeeping) legitimately need it, not just
        internal wait_for_*() logic."""
        others = self.world.get_other_robots()
        return next(iter(others), None)

    def _other_id(self):
        return self.other_robot_id()

    def _other_state_or_none(self):
        """
        The other robot's latest state, or None if we've never heard from
        it or its last message is stale — the two situations solo_ok
        distinguishes between "no partner running" and "partner present
        and current."
        """
        other_id = self._other_id()
        if other_id is None or self.world.is_stale(other_id):
            return None
        return self.world.get_state(other_id)

    @staticmethod
    def _distance(pose_a, pose_b):
        return math.hypot(pose_a.get("x", 0) - pose_b.get("x", 0),
                           pose_a.get("y", 0) - pose_b.get("y", 0))
