"""Empirical sweep: bypass xyInput()'s 2-link IK entirely and command raw
shoulder/elbow joint angles (via the "arm_set_joint" network command), reading
back the real Unity end-effector position via get_arm_state for each.

Purpose: TTLServo.xyInput()'s IK only models linkageLenA=90mm (shoulder->elbow)
and linkageLenB=160mm (elbow->forearm pivot), but the scene's actual chain
adds a further ~120mm rigid segment (forearm pivot -> wrist pivot -> end
effector, at 0 + 120mm) that is never independently actuated (wrist
pitch/tool roll stay at 0 throughout xyInput commands). If that extra segment
is real and un-modeled, forward-kinematics driven directly by raw joint
angles should be able to reach much lower than xyInput ever manages -- this
sweep checks that hypothesis directly, independent of any IK bug.
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


def state():
    resp = sim_client.send_query({"command": "get_arm_state", "robot_id": ROBOT_ID})
    return resp["joint_angles"], resp["end_effector"]


# joint 0 = base yaw, 1 = shoulder, 2 = elbow (see RoboticArmController joint map)
set_joint(0, 0.0)
time.sleep(0.3)

best = None
for shoulder in [-20, -10, 0, 20, 40, 60, 85]:
    for elbow in [-130, -90, -60, -30, 0, 30, 60, 90, 130]:
        set_joint(1, shoulder)
        set_joint(2, elbow)
        time.sleep(0.5)
        angles, ee = state()
        h = ee['y']
        if best is None or h < best[0]:
            best = (h, shoulder, elbow, ee)
        print(f"shoulder={shoulder:>5.0f} elbow={elbow:>5.0f} -> "
              f"actual=({angles[1]:.1f},{angles[2]:.1f}) "
              f"end_effector=(x={ee['x']:.3f}, y={ee['y']:.3f}, z={ee['z']:.3f})")

print()
print(f"LOWEST height found: y={best[0]:.3f} at shoulder={best[1]}, elbow={best[2]}, ee={best[3]}")
