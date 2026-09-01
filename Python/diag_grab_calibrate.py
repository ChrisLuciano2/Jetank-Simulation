"""Empirical grab calibration: drive the robot to real grab range against a
real block (same navigate_to_bearing + creep_to_target pipeline
autonomous_tower_build.py uses), then sweep Y_AT_FLOOR_MM/REACH_PICKUP_MM
combos directly against a live reach_and_grab() attempt, checking
get_arm_state().is_holding after each -- no theorizing about coordinate
frames, just "does it actually grab."
"""
import time

import jetson_utils
from jetbot import Robot
from jetbot_nav import arm_ops, gap_follow, visual_scan
from jetbot_nav.target_seek import SeekingGapFollowController

from autonomous_tower_build import (
    navigate_to_bearing, creep_to_target, sense_and_sort_blocks,
    PICKUP_ARRIVAL_DISTANCE_M,
)
from SCSCtrl import TTLServo
import sim_client
import sim_robot_id


def is_holding():
    resp = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
    return bool(resp.get("is_holding"))


def main():
    robot = Robot()
    camera = jetson_utils.videoSource("csi://0")
    geom = visual_scan.sim_jetank()
    controller = SeekingGapFollowController(max_range=12.0)

    sensed = sense_and_sort_blocks(camera, geom)
    if not sensed:
        print("[calibrate] no block sensed -- aborting")
        return
    target = sensed[0]
    print(f"[calibrate] targeting block: bearing={target['bearing_deg']:.1f} "
          f"distance={target['distance_m']:.2f} width={target['width_m']:.3f} "
          f"height={target['height_m']:.3f}")

    if not navigate_to_bearing(robot, controller, camera, geom, target["bearing_deg"]):
        print("[calibrate] navigate_to_bearing failed -- aborting")
        return

    if not creep_to_target(robot, camera, geom, color="red",
                            arrival_distance=PICKUP_ARRIVAL_DISTANCE_M,
                            expected_bearing_deg=target["bearing_deg"]):
        print("[calibrate] creep_to_target failed -- aborting")
        return

    robot.stop()
    print("[calibrate] in position, arm state before any attempt:")
    resp = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
    print(resp)

    # Sweep reach (x_input) and height (y_input) jointly.
    best = None
    for reach in [90, 120, 150, 180, 200]:
        for y in [-170, -140, -110, -80, -60, -30, 0, 30, 60]:
            TTLServo.servoAngleCtrl(1, 0.0, 1, 300)
            time.sleep(0.3)
            TTLServo.xyInput(reach, y)
            time.sleep(0.8)
            TTLServo.servoAngleCtrl(4, -100, 1, 200)
            time.sleep(0.6)
            holding = is_holding()
            resp = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
            ee = resp.get("end_effector", {})
            print(f"reach={reach:>4.0f} y_input={y:>5.0f} -> holding={holding} "
                  f"end_effector=(x={ee.get('x',0):.3f}, y={ee.get('y',0):.3f}, z={ee.get('z',0):.3f})")
            # Re-open gripper before the next attempt regardless of outcome.
            TTLServo.servoAngleCtrl(4, 100, 1, 200)
            time.sleep(0.4)
            if holding:
                best = (reach, y)
                print(f"*** GRAB SUCCEEDED at reach={reach}, y_input={y} ***")
                robot.stop()
                return

    print(f"\nNo combination grabbed. best={best}")


if __name__ == "__main__":
    main()
