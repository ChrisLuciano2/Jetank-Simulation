"""Same as diag_full_cycle.py (runs the real build_tower() end-to-end via
Coordinator(solo_ok=True)) EXCEPT jetbot_nav.arm_ops.reach_and_grab is
monkey-patched with an instrumented version that logs get_block_pose
(ground truth, both blocks) and get_arm_state's gripper_position before and
after every FLOOR_REACH_CANDIDATES attempt.

Exists because diag_grab_pose_check.py (hand-run outside build_tower())
never reproduced Section 1.10/1.13's "all six candidates fail" mystery
despite several tries this session, while a real diag_full_cycle.py run
failed that way once WITHOUT this instrumentation attached. This patches
the exact call path autonomous_tower_build.py actually uses, so if it
fails again, the failure and the diagnostics happen in the same run
instead of hoping a separate hand-run script reproduces the same
conditions.
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

BLOCK_IDS = ["block_A_large", "block_B_small"]
_original_reach_and_grab = arm_ops.reach_and_grab


def block_poses():
    return {bid: sim_client.send_query({"command": "get_block_pose", "block_id": bid})
            for bid in BLOCK_IDS}


def fmt_xyz(d):
    if not d or d.get("status") != "ok":
        return f"<no data: {d}>"
    return f"(x={d.get('x', float('nan')):.4f}, y={d.get('y', float('nan')):.4f}, z={d.get('z', float('nan')):.4f})"


def instrumented_reach_and_grab(base_yaw_deg: float) -> bool:
    from SCSCtrl import TTLServo

    print(f"\n[INSTRUMENTED reach_and_grab] base_yaw_deg={base_yaw_deg}")
    TTLServo.servoAngleCtrl(1, base_yaw_deg, 1, 300)
    time.sleep(0.5)

    print("  block ground truth BEFORE any attempt:")
    before = block_poses()
    for bid, p in before.items():
        print(f"    {bid}: {fmt_xyz(p)}")

    for i, (shoulder_deg, elbow_deg) in enumerate(arm_ops.FLOOR_REACH_CANDIDATES):
        print(f"  --- attempt {i}: shoulder={shoulder_deg} elbow={elbow_deg} ---")

        TTLServo.servoAngleCtrl(2, shoulder_deg, 1, 300)
        TTLServo.servoAngleCtrl(3, elbow_deg, 1, 300)
        time.sleep(arm_ops.MOVE_SETTLE_S)

        pre_state = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
        pre_joints = pre_state.get("joint_angles", [])
        pre_gripper = pre_state.get("gripper_position", {})
        pre_blocks = block_poses()
        arrived_shoulder = pre_joints[1] if len(pre_joints) > 1 else float("nan")
        arrived_elbow = pre_joints[2] if len(pre_joints) > 2 else float("nan")
        print(f"    pre-close joint_angles: shoulder={arrived_shoulder:.2f} elbow={arrived_elbow:.2f} "
              f"(commanded shoulder={shoulder_deg} elbow={elbow_deg})")
        print(f"    pre-close gripper_position: {fmt_xyz(pre_gripper)}")
        for bid, p in pre_blocks.items():
            print(f"    pre-close {bid}: {fmt_xyz(p)}")

        TTLServo.servoAngleCtrl(4, -100, 1, 200)
        time.sleep(arm_ops.GRAB_SETTLE_S)

        post_state = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
        holding = bool(post_state.get("is_holding"))
        post_gripper = post_state.get("gripper_position", {})
        post_blocks = block_poses()
        print(f"    post-close is_holding={holding} gripper_position: {fmt_xyz(post_gripper)}")
        for bid, p in post_blocks.items():
            print(f"    post-close {bid}: {fmt_xyz(p)}")
            pb = pre_blocks.get(bid, {})
            if pb.get("status") == "ok" and p.get("status") == "ok":
                dx = p["x"] - pb["x"]; dy = p["y"] - pb["y"]; dz = p["z"] - pb["z"]
                dist = (dx**2 + dy**2 + dz**2) ** 0.5
                if dist > 0.001:
                    print(f"      [moved {dist*1000:.1f} mm during this attempt]")

        if holding:
            print(f"  *** GRAB CONFIRMED on attempt {i} ***")
            return True

        TTLServo.servoAngleCtrl(4, 100, 1, 200)
        time.sleep(0.3)

    print("  *** ALL CANDIDATES FAILED -- full diagnostic dump ***")
    chassis_pose = sim_client.send_query({"command": "get_pose", "robot_id": sim_robot_id.TRUCK_ID})
    print(f"    chassis pose at failure: {chassis_pose}")
    final_state = sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})
    print(f"    final arm state: {final_state}")
    final_blocks = block_poses()
    for bid, p in final_blocks.items():
        print(f"    final {bid}: {fmt_xyz(p)}")
    return False


arm_ops.reach_and_grab = instrumented_reach_and_grab


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
