"""Validate that a raw-joint floor-reach grab works at base_yaw_deg=0 --
the fixed value arm_ops.reach_and_grab() actually calls with in production
(chassis rotation + creep's bearing gate does the aiming, not arm yaw).
diag_floor_reach_calibrate.py's pass1 found success at yaw=-20 first, but
that doesn't confirm yaw=0 works -- this checks that specifically, and
narrows in on a good default shoulder/elbow for arm_ops.py's constants.
"""
import time

import jetson_utils
from jetbot import Robot
from jetbot_nav import visual_scan
from jetbot_nav.target_seek import SeekingGapFollowController
from autonomous_tower_build import (
    navigate_to_bearing, creep_to_target, sense_and_sort_blocks_with_search,
    PICKUP_ARRIVAL_DISTANCE_M,
)
import sim_client, sim_robot_id

ROBOT_ID = sim_robot_id.ARM_ID


def set_joint(i, a):
    sim_client.send_command({"command": "arm_set_joint", "robot_id": ROBOT_ID, "joint_index": i, "angle": a})


def set_gripper(a):
    sim_client.send_command({"command": "arm_set_gripper", "robot_id": ROBOT_ID, "gripper_amount": a})


def state():
    return sim_client.send_query({"command": "get_arm_state", "robot_id": ROBOT_ID})


robot = Robot()
camera = jetson_utils.videoSource("csi://0")
geom = visual_scan.sim_jetank()
controller = SeekingGapFollowController(max_range=12.0)

sensed = sense_and_sort_blocks_with_search(robot, camera, geom)
print("sensed:", sensed)
target = sensed[0]

ok1 = navigate_to_bearing(robot, controller, camera, geom, target["bearing_deg"])
print("navigate_to_bearing:", ok1)
ok2 = creep_to_target(robot, camera, geom, color="red",
                       arrival_distance=PICKUP_ARRIVAL_DISTANCE_M,
                       expected_bearing_deg=target["bearing_deg"])
print("creep_to_target:", ok2)
robot.stop()

pose = sim_client.send_query({"command": "get_pose", "robot_id": sim_robot_id.TRUCK_ID})
print("chassis pose after creep:", pose)

set_joint(0, 0)
time.sleep(0.4)

results = []
for shoulder in [65, 70, 75, 80, 85]:
    for elbow in [80, 90, 100, 110, 120, 130]:
        set_joint(1, shoulder)
        set_joint(2, elbow)
        time.sleep(0.5)
        set_gripper(0.0)
        time.sleep(0.5)
        resp = state()
        holding = bool(resp.get("is_holding"))
        gp = resp.get("gripper_position", {})
        print(f"yaw=0 shoulder={shoulder:>4} elbow={elbow:>4} -> holding={holding} "
              f"gripper=(x={gp.get('x',0):.3f}, y={gp.get('y',0):.3f}, z={gp.get('z',0):.3f})")
        set_gripper(1.0)
        time.sleep(0.3)
        if holding:
            results.append((shoulder, elbow))

print("\nworking (shoulder, elbow) combos at yaw=0:", results)
