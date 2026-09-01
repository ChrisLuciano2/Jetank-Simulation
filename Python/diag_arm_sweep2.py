"""Second sweep: vary BOTH x (reach) and y to see if a different combination
drives the shoulder closer to its actual floor limit (-20 deg) and reaches
lower than the first sweep's best (~1.07m)."""
import time

import sim_client
import sim_robot_id
from SCSCtrl import TTLServo

def state():
    resp = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
    return resp["joint_angles"], resp["end_effector"]

TTLServo.servoAngleCtrl(1, 0.0, 1, 300)
time.sleep(0.3)

for x in [90, 120, 150, 180, 200]:
    for y in [-170, -100, 100, 170]:
        TTLServo.xyInput(x, y)
        time.sleep(0.6)
        angles, ee = state()
        print(f"x={x:>4.0f} y={y:>5.0f} -> shoulder,elbow=({angles[1]:>6.1f},{angles[2]:>6.1f}) "
              f"ee=(y={ee['y']:.3f}, z={ee['z']:.3f})")
