"""
robot_coord.messages — The shared message schema for the build-coordination
channel (collaborative-build-brainstorm.md, section 4).

Defined once, here, so both robots — and any tooling that subscribes to the
MQTT topics to watch the build live — agree on field names instead of each
hand-rolling a dict literal that quietly drifts apart.

Two message kinds:
  - state message  (topic: sensym/build/state) — one robot's current pose,
    state, and progress. Published every cycle.
  - flag message   (topic: sensym/build/flag)  — a defect or assist request
    (section 6). Published only when something needs the other robot's
    attention.
"""

import time

# Extends the brainstorm doc's suggested "state" enum — unchanged from
# section 4.
VALID_STATES = {
    "idle",
    "moving_to_pick",
    "moving_to_place",
    "placing",
    "placed_clear",
    "inspecting",
    "paused",
    "requesting_assist",
}

# Section 6 flag kinds. Not a closed set — flag_defect() accepts any
# string — but these are the ones the coordinator itself reacts to.
FLAG_MISALIGNED_BLOCK = "misaligned_block"
FLAG_COLLISION_RISK   = "collision_risk"
FLAG_ASSIST_REQUEST   = "assist_request"


def build_state_message(robot_id, state, pose, blocks_placed=0,
                         current_action=None, detected_structure_top=None,
                         flags=None):
    """
    Build one state-update message.

    pose: {"x": float, "y": float, "heading_deg": float} — ground-plane
        position + heading. Ground-plane axes are up to the caller's
        convention as long as both robots use the same one; the Unity
        dev-only get_pose query returns (x, z, yaw_deg) for the sim
        robot's ground-plane position, which is what the sim side should
        pass as (x, y, heading_deg) here.
    blocks_placed: monotonically increasing count of blocks this robot has
        placed on its side. This is the number wait_for_pacing_clearance()
        compares between robots for the bounded-slack check (section 5) —
        it's a small addition beyond the doc's original schema sketch,
        because bounded slack needs *something* numeric and monotonic to
        compare, and "count of blocks I've placed" is simpler and more
        robust than trying to diff position along the build axis.
    current_action: {"block_id": ..., "target_position": {...}} or None.
    detected_structure_top: {"x", "y", "z"} or None — this robot's camera
        reading of the current top/edge of the structure on its side.
    flags: list of strings, e.g. ["collision_risk"]. Usually empty —
        one-off problems go through build_flag_message() on the flag
        topic instead, since a flag has to survive being overwritten by
        the next cycle's state update, which flags-on-state wouldn't.

    Raises ValueError on an unrecognized state, since a typo'd state name
    would otherwise silently fail every coordinator check that compares
    against VALID_STATES.
    """
    if state not in VALID_STATES:
        raise ValueError(f"unknown state {state!r}; expected one of {sorted(VALID_STATES)}")
    return {
        "robot_id": robot_id,
        "timestamp": time.time(),
        "pose": dict(pose),
        "state": state,
        "blocks_placed": int(blocks_placed),
        "current_action": current_action or {},
        "detected_structure_top": detected_structure_top,
        "flags": list(flags) if flags else [],
    }


def build_flag_message(robot_id, flag, location=None, note=""):
    """
    Build one defect/assist-request message (section 6).

    flag: a short string, e.g. "misaligned_block" (see FLAG_* constants
        above for the ones the coordinator recognizes — anything else is
        still delivered, just not auto-acted-on).
    location: {"x", "y", "z"} of the thing being flagged, or None.
    note: free-text detail for a human reading the MQTT log.
    """
    return {
        "robot_id": robot_id,
        "timestamp": time.time(),
        "flag": flag,
        "location": dict(location) if location else None,
        "note": note,
    }
