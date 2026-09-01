"""Diagnostic: run reach_and_grab()'s exact motion, then print the real
end-effector position/joint angles via get_arm_state, plus the known block
Y=0 floor plane, to see how far off Y_AT_FLOOR_MM actually is."""
import time

import sim_client
import sim_robot_id
from SCSCtrl import TTLServo
from jetbot_nav import arm_ops


def dump(label):
    resp = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
    print(f"--- {label} ---")
    print(resp)


dump("before")

TTLServo.servoAngleCtrl(1, 0.0, 1, 300)
time.sleep(0.5)
TTLServo.xyInput(arm_ops.REACH_PICKUP_MM, arm_ops.Y_AT_FLOOR_MM)
time.sleep(arm_ops.MOVE_SETTLE_S)
dump("after xyInput (before gripper close)")

TTLServo.servoAngleCtrl(4, -100, 1, 200)
time.sleep(arm_ops.GRAB_SETTLE_S)
dump("after gripper close")
