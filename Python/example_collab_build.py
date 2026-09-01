"""
example_collab_build.py — Skeleton for the two-robot collaborative build
(collaborative-build-brainstorm.md).

This is a runnable template, not a finished build script: detect_material(),
detect_structure_top(), pick_and_place_block(), and run_qc_scan() are stubs
that print what they'd do — fill them in with real perception/arm control
(jetbot_nav, jetson_utils, SCSCtrl) once the coordination flow below is
validated. The coordination calls (publish_state / wait_for_* / flag_defect)
are the actual deliverable here and need no changes to go from stub to real.

Same file, both robots, both environments — nothing here is sim-only or
robot-specific:

    # Robot A, simulated:
    ROBOT_ID=robot_a python example_collab_build.py

    # Robot B, simulated (second terminal):
    ROBOT_ID=robot_b python example_collab_build.py

    # Robot A, real hardware:
    ROBOT_ID=robot_a MQTT_BROKER_HOST=192.168.1.50 python example_collab_build.py

    # Robot B, real hardware:
    ROBOT_ID=robot_b MQTT_BROKER_HOST=192.168.1.50 python example_collab_build.py

ROBOT_ID picks the MQTT identity everywhere, and additionally picks which
simulated truck/arm this process drives (see sim_robot_id.py) when running
against Unity — that second part has no effect on real hardware, where
there's exactly one robot and jetbot/SCSCtrl talk to it directly.
"""

import time

from jetbot import Robot
from robot_coord import Coordinator
from robot_coord.messages import FLAG_MISALIGNED_BLOCK

TARGET_BLOCK_COUNT = 5          # blocks this robot places on its side
MEETING_POINT = {"x": 0.0, "y": 0.0}   # midline both robots build toward — measure/agree on this per workspace
MAX_SLACK = 1                   # bounded slack (brainstorm doc section 5) — start near-lockstep
EXCLUSION_RADIUS = 0.5          # metres from MEETING_POINT treated as "the seam"


def get_my_pose():
    """
    STUB: return this robot's current {"x", "y", "heading_deg"}.

    Real implementation reads from whatever localization the project
    settles on (brainstorm doc section 4: AprilTags/ArUco + odometry, most
    likely). Returning a fixed point here so the coordination flow below
    is runnable before that's wired up.
    """
    return {"x": 0.0, "y": 0.0, "heading_deg": 0.0}


def detect_material():
    """STUB: locate the next block to pick up via the AI camera. Phase 1."""
    print("[build] (stub) scanning for material...")
    return {"block_id": "blk_stub", "position": {"x": 0.0, "y": 0.0, "z": 0.0}}


def detect_structure_top():
    """
    STUB: read the current top/edge of the structure on this robot's side.
    Phase 2 — this is what makes placement adaptive instead of a hardcoded
    coordinate (brainstorm doc section 7): read the real structure state
    every cycle, not an assumption of where the last block landed.
    """
    print("[build] (stub) detecting current structure top...")
    return {"x": 0.0, "y": 0.0, "z": 0.0}


def pick_and_place_block(robot: Robot, block, target):
    """STUB: drive to block, pick it up, drive to target, place it."""
    print(f"[build] (stub) placing {block['block_id']} at {target}")
    robot.stop()


def run_qc_scan():
    """STUB: Phase 3 — scan the finished structure from this robot's side."""
    print("[build] (stub) running QC scan...")
    return []   # list of defects, e.g. [{"kind": "misaligned_block", "location": {...}}]


def build_loop(robot: Robot, coord: Coordinator):
    blocks_placed = 0

    coord.publish_state("idle", pose=get_my_pose(), blocks_placed=blocks_placed)

    while blocks_placed < TARGET_BLOCK_COUNT:
        next_count = blocks_placed + 1

        # Bounded slack: don't start a block that would put us more than
        # MAX_SLACK ahead of the other robot.
        coord.publish_state("moving_to_pick", pose=get_my_pose(), blocks_placed=blocks_placed)
        if not coord.wait_for_pacing_clearance(next_count):
            print("[build] WARNING: pacing wait timed out — other robot may be stalled")

        block = detect_material()
        target = detect_structure_top()

        coord.publish_state(
            "moving_to_place", pose=get_my_pose(), blocks_placed=blocks_placed,
            current_action={"block_id": block["block_id"], "target_position": target},
            detected_structure_top=target,
        )

        # Exclusion zone: don't enter the meeting-point buffer while the
        # other robot is in it (only matters once get_my_pose() reflects
        # where we're actually about to move).
        if not coord.wait_for_zone_clearance(get_my_pose()):
            print("[build] WARNING: zone wait timed out — proceeding cautiously")

        # Placement clearance: don't approach while the other robot hasn't
        # signaled it's clear of the shared workspace.
        coord.publish_state("placing", pose=get_my_pose(), blocks_placed=blocks_placed,
                             current_action={"block_id": block["block_id"], "target_position": target})
        if not coord.wait_for_placement_clear():
            print("[build] WARNING: placement-clear wait timed out — proceeding cautiously")

        pick_and_place_block(robot, block, target)
        blocks_placed = next_count

        coord.publish_state("placed_clear", pose=get_my_pose(), blocks_placed=blocks_placed)
        print(f"[build] placed {blocks_placed}/{TARGET_BLOCK_COUNT}")

    print("[build] my side complete — waiting for other robot before QC")
    coord.publish_state("idle", pose=get_my_pose(), blocks_placed=blocks_placed)


def qc_loop(coord: Coordinator):
    coord.publish_state("inspecting", pose=get_my_pose(), blocks_placed=TARGET_BLOCK_COUNT)
    defects = run_qc_scan()
    for d in defects:
        print(f"[build] flagging defect: {d}")
        coord.flag_defect(d.get("kind", FLAG_MISALIGNED_BLOCK), location=d.get("location"))
    coord.publish_state("idle", pose=get_my_pose(), blocks_placed=TARGET_BLOCK_COUNT)
    print(f"[build] QC complete — {len(defects)} defect(s) flagged")


def main():
    robot = Robot()
    # solo_ok=False (default) means every wait_for_*() call below blocks
    # for real until the other robot's process is also running and
    # publishing — that's the point, for a real two-robot run. To exercise
    # this script alone without a partner, pass solo_ok=True here.
    coord = Coordinator(meeting_point=MEETING_POINT, max_slack=MAX_SLACK,
                         exclusion_radius=EXCLUSION_RADIUS)

    print(f"[build] connecting as {coord.robot_id}...")
    coord.connect()

    try:
        build_loop(robot, coord)
        qc_loop(coord)
    except KeyboardInterrupt:
        print("\n[build] interrupted")
    finally:
        robot.stop()
        coord.publish_state("paused", pose=get_my_pose())
        coord.disconnect()


if __name__ == "__main__":
    main()
