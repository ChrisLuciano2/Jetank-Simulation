"""Follow-up to diag_grab_calibrate.py: that script proved xyInput()'s 2-link
IK (TTLServo.linkageLenA/B = 90/160mm) never computes joint angles anywhere
near the region that actually reaches the floor -- diag_arm_joint_sweep.py
separately proved raw joint angles around shoulder=85, elbow=90-130 CAN reach
down to ~0.33-0.45m, right in block territory. This script assumes the robot
is already creeped into grab position (does NOT re-navigate -- run
diag_grab_calibrate.py first, or make sure the sim is already in that state)
and sweeps raw shoulder/elbow joint angles directly via arm_set_joint,
checking is_holding after each real gripper close.
"""
import time

import sim_client
import sim_robot_id

ROBOT_ID = sim_robot_id.ARM_ID


def set_joint(index, angle):
    sim_client.send_command({
        "command": "arm_set_joint",
        "robot_id": ROBOT_ID,
        "joint_index": index,
        "angle": angle,
    })


def set_gripper(amount):
    sim_client.send_command({
        "command": "arm_set_gripper",
        "robot_id": ROBOT_ID,
        "gripper_amount": amount,
    })


def state():
    resp = sim_client.send_query({"command": "get_arm_state", "robot_id": ROBOT_ID})
    return resp


set_joint(0, 0.0)  # base yaw
time.sleep(0.3)

best = None
for shoulder in [60, 70, 75, 80, 85]:
    for elbow in [70, 80, 90, 100, 110, 120, 130]:
        set_joint(1, shoulder)
        set_joint(2, elbow)
        time.sleep(0.6)
        set_gripper(0.0)  # closed
        time.sleep(0.6)
        resp = state()
        holding = bool(resp.get("is_holding"))
        ee = resp.get("end_effector", {})
        print(f"shoulder={shoulder:>4.0f} elbow={elbow:>4.0f} -> holding={holding} "
              f"end_effector=(x={ee.get('x',0):.3f}, y={ee.get('y',0):.3f}, z={ee.get('z',0):.3f})")
        set_gripper(1.0)  # reopen for next attempt
        time.sleep(0.4)
        if holding:
            best = (shoulder, elbow)
            print(f"*** GRAB SUCCEEDED at shoulder={shoulder}, elbow={elbow} ***")

if best:
    print(f"\nBest working combo: shoulder={best[0]}, elbow={best[1]}")
else:
    print("\nNo combination grabbed.")
