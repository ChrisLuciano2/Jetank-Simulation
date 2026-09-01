"""Empirical sweep: try several y_input values to xyInput() at a fixed reach,
and print the real Unity end-effector height for each, to find which value
actually brings the gripper down to floor level (blocks sit at y=~0.3-0.5)."""
import time

import sim_client
import sim_robot_id
from SCSCtrl import TTLServo

REACH_MM = 150.0


def state():
    resp = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
    return resp["joint_angles"], resp["end_effector"]


TTLServo.servoAngleCtrl(1, 0.0, 1, 300)
time.sleep(0.3)

for y in [-170, -100, -60, 0, 60, 100, 170]:
    TTLServo.xyInput(REACH_MM, y)
    time.sleep(1.0)
    angles, ee = state()
    print(f"y_input={y:>6.0f} -> joints(shoulder,elbow)=({angles[1]:.1f},{angles[2]:.1f}) "
          f"end_effector=(x={ee['x']:.3f}, y={ee['y']:.3f}, z={ee['z']:.3f})")
