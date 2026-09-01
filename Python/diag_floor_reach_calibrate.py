"""Empirically find shoulder/elbow angles that let raw-joint control (not
xyInput(), which cannot compute the needed elbow angles -- see HANDOFF.md)
reach a real, correctly-sized block after the full navigate+creep pipeline
gets the chassis into position.
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

# NOTE: as of this run, RoboticArmController.TryGrab()'s OverlapSphere check
# was switched from GetEndEffectorPosition() (j5_wrist -- matches what
# xyInput()'s IK model believes the tip is, per that method's own doc
# comment) to the new GetGripperPosition() (midpoint of gripperLeft/
# gripperRight, ~120-290mm further out -- where the fingers physically are).
# Every prior sweep in this file checked physical proximity from the WRONG
# point (j5_wrist), which could silently reject a real, physical grab. Rerun
# the exact same combinations first before widening the search -- if the
# reference-point fix alone resolves this, no search widening is needed at
# all. get_arm_state now also reports gripper_position directly (the real
# physical point), logged below alongside the old end_effector for
# comparison.
def sweep(label, yaws, shoulders, elbows, wrist_pitches=(0,)):
    best = None
    for yaw in yaws:
        set_joint(0, yaw)
        time.sleep(0.4)
        for shoulder in shoulders:
            for elbow in elbows:
                for wrist in wrist_pitches:
                    set_joint(1, shoulder)
                    set_joint(2, elbow)
                    set_joint(4, wrist)
                    time.sleep(0.5)
                    set_gripper(0.0)
                    time.sleep(0.5)
                    resp = state()
                    holding = bool(resp.get("is_holding"))
                    ee = resp.get("end_effector", {})
                    gp = resp.get("gripper_position", {})
                    print(f"[{label}] yaw={yaw:>4} shoulder={shoulder:>4} elbow={elbow:>4} "
                          f"wrist={wrist:>4} -> holding={holding} "
                          f"ee=(x={ee.get('x',0):.3f}, y={ee.get('y',0):.3f}, z={ee.get('z',0):.3f}) "
                          f"gripper=(x={gp.get('x',0):.3f}, y={gp.get('y',0):.3f}, z={gp.get('z',0):.3f})")
                    set_gripper(1.0)
                    time.sleep(0.3)
                    if holding:
                        best = (yaw, shoulder, elbow, wrist)
                        print(f"*** GRAB SUCCEEDED: yaw={yaw}, shoulder={shoulder}, "
                              f"elbow={elbow}, wrist={wrist} ***")
                        return best
    return best


# Pass 1: the exact combinations the pre-fix sweep already tried (105 of
# them, all "failed" against the wrong reference point).
best = sweep("pass1-original", [-20, -10, 0, 10, 20], [75, 80, 85],
             [90, 100, 110, 120, 130])

# Pass 2 (only if pass 1 still comes up empty): widen shoulder down toward
# its floor and elbow across its full hard-limit range, plus wrist pitch --
# tilting the wrist down can bring the fingers closer to the floor without
# needing as extreme a shoulder/elbow bend.
if best is None:
    print("\npass1 found nothing, widening search (pass2)...\n")
    best = sweep("pass2-widened", [-15, 0, 15], [40, 55, 65, 75, 85],
                 [-30, 0, 30, 60, 90, 120, 130], wrist_pitches=[-60, -30, 0, 30])

print("\nbest =", best)
