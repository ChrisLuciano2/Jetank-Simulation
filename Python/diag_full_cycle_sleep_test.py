"""Same as diag_full_cycle_instrumented.py, but tests a different
hypothesis: instead of querying get_block_pose/get_arm_state (which this
session found only ever succeeded, never reproducing Section 1.10/1.13's
all-six-candidates-fail failure), this patches reach_and_grand with a
version that is otherwise IDENTICAL to the real production
arm_ops.reach_and_grab() -- same TTLServo calls, same settle constants --
but inserts a plain time.sleep() of about the same duration the
instrumented version's extra queries actually took (measured live:
~2ms/query, ~12ms total for 6 queries per attempt; using 0.015s at each of
the two insertion points here for a bit of margin).

If this ALSO never fails, the "only instrumented runs succeed" pattern
isn't really about elapsed wall-clock time (12-30ms is tiny next to the
0.6-1.5s settle windows already in place) -- something else about running
queries (e.g. forcing synchronization with Unity's own frame loop via the
blocking TCP round-trip, rather than the raw duration) would be the more
likely explanation, and this test is what tells the difference.

If this DOES still fail sometimes, that's evidence the extra time itself
(regardless of source) is what matters, which would point toward nudging
GRAB_SETTLE_S/MOVE_SETTLE_S up a bit more as the real fix instead.
"""
import time

import jetson_utils
from jetbot import Robot
from jetbot_nav import visual_scan
from jetbot_nav.target_seek import SeekingGapFollowController
from robot_coord import Coordinator

import jetbot_nav.arm_ops as arm_ops
import sim_client, sim_robot_id
from autonomous_tower_build import build_tower, MAX_SLACK, EXCLUSION_RADIUS_M

EXTRA_SLEEP_S = 0.015  # ~ measured per-query overhead with margin


def sleep_instead_reach_and_grab(base_yaw_deg: float) -> bool:
    from SCSCtrl import TTLServo

    TTLServo.servoAngleCtrl(1, base_yaw_deg, 1, 300)
    time.sleep(0.5)

    for shoulder_deg, elbow_deg in arm_ops.FLOOR_REACH_CANDIDATES:
        TTLServo.servoAngleCtrl(2, shoulder_deg, 1, 300)
        TTLServo.servoAngleCtrl(3, elbow_deg, 1, 300)
        time.sleep(arm_ops.MOVE_SETTLE_S)

        time.sleep(EXTRA_SLEEP_S)   # stands in for the instrumented version's pre-close queries

        TTLServo.servoAngleCtrl(4, -100, 1, 200)
        time.sleep(arm_ops.GRAB_SETTLE_S)

        time.sleep(EXTRA_SLEEP_S)   # stands in for the instrumented version's post-close queries

        resp = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
        if bool(resp.get("is_holding")):
            print(f"  [sleep-test] GRAB CONFIRMED shoulder={shoulder_deg} elbow={elbow_deg}")
            return True

        TTLServo.servoAngleCtrl(4, 100, 1, 200)
        time.sleep(0.3)

    print("  [sleep-test] ALL CANDIDATES FAILED")
    return False


arm_ops.reach_and_grab = sleep_instead_reach_and_grab


def main():
    robot = Robot()
    camera = jetson_utils.videoSource("csi://0")
    geom = visual_scan.sim_jetank()
    controller = SeekingGapFollowController(max_range=12.0)

    coord = Coordinator(meeting_point={"x": 0.0, "y": 0.0}, max_slack=MAX_SLACK,
                         exclusion_radius=EXCLUSION_RADIUS_M, solo_ok=True)
    print(f"[build] connecting as {coord.robot_id}...")
    coord.connect()

    try:
        build_tower(robot, camera, geom, controller, coord)
    except KeyboardInterrupt:
        print("\n[build] interrupted")
    finally:
        robot.stop()
        coord.disconnect()


if __name__ == "__main__":
    main()
