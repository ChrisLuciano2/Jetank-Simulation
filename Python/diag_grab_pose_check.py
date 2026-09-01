"""Diagnostic: after the real sense -> navigate -> creep pipeline arrives at
a block, reimplement arm_ops.reach_and_grab()'s FLOOR_REACH_CANDIDATES loop
by hand -- with get_block_pose (diagnostic-only ground truth, never used by
jetbot_nav/ itself) and get_arm_state's gripper_position queried before AND
after every attempt.

This exists to answer HANDOFF.md Section 1.10's open question directly
instead of guessing between its two live hypotheses:
  (a) the block isn't physically where telemetry/creep-arrival implies, or
  (b) repeated gripper close/open cycling nudges the block out from under
      later attempts in the same loop.

If the block's ground-truth position is stable across all attempts and
close to the true reach in world space, but is_holding still never goes
true, that points at (a) being wrong and something else (grab-radius
tolerance, timing) being the real culprit. If the block's position visibly
shifts between attempts, that confirms (b).
"""
import time

import jetson_utils
from jetbot import Robot
from jetbot_nav import visual_scan
from jetbot_nav.target_seek import SeekingGapFollowController
from SCSCtrl import TTLServo
from autonomous_tower_build import (
    navigate_to_bearing, creep_to_target, sense_and_sort_blocks_with_search,
    PICKUP_ARRIVAL_DISTANCE_M,
)
from jetbot_nav.arm_ops import FLOOR_REACH_CANDIDATES, MOVE_SETTLE_S, GRAB_SETTLE_S
import sim_client, sim_robot_id

BLOCK_IDS = ["block_A_large", "block_B_small"]


def block_poses():
    return {bid: sim_client.send_query({"command": "get_block_pose", "block_id": bid})
            for bid in BLOCK_IDS}


def arm_state():
    return sim_client.send_query({"command": "get_arm_state", "robot_id": sim_robot_id.ARM_ID})


def fmt_xyz(d):
    return f"(x={d.get('x', float('nan')):.4f}, y={d.get('y', float('nan')):.4f}, z={d.get('z', float('nan')):.4f})"


robot = Robot()
camera = jetson_utils.videoSource("csi://0")
geom = visual_scan.sim_jetank()
controller = SeekingGapFollowController(max_range=12.0)

sensed = sense_and_sort_blocks_with_search(robot, camera, geom)
print("sensed:", sensed)
if not sensed:
    print("[diag] nothing sensed -- aborting")
    raise SystemExit(1)
target = sensed[0]

ok1 = navigate_to_bearing(robot, controller, camera, geom, target["bearing_deg"])
print("navigate_to_bearing:", ok1)
ok2 = creep_to_target(robot, camera, geom, color="red",
                       arrival_distance=PICKUP_ARRIVAL_DISTANCE_M,
                       expected_bearing_deg=target["bearing_deg"])
print("creep_to_target:", ok2)
robot.stop()

if not ok2:
    print("[diag] creep failed to arrive -- aborting grab-loop diagnostics")
    raise SystemExit(1)

chassis_pose = sim_client.send_query({"command": "get_pose", "robot_id": sim_robot_id.TRUCK_ID})
print("chassis pose after creep:", chassis_pose)

print("\n--- BLOCK GROUND TRUTH BEFORE ANY GRAB ATTEMPT ---")
before = block_poses()
for bid, p in before.items():
    print(f"  {bid}: {p}")

TTLServo.servoAngleCtrl(1, 0.0, 1, 300)
time.sleep(0.5)

for i, (shoulder_deg, elbow_deg) in enumerate(FLOOR_REACH_CANDIDATES):
    print(f"\n=== attempt {i}: shoulder={shoulder_deg} elbow={elbow_deg} ===")

    TTLServo.servoAngleCtrl(2, shoulder_deg, 1, 300)
    TTLServo.servoAngleCtrl(3, elbow_deg, 1, 300)
    time.sleep(MOVE_SETTLE_S)

    pre_state = arm_state()
    pre_gripper = pre_state.get("gripper_position", {})
    pre_blocks = block_poses()
    print(f"  pre-close gripper_position: {fmt_xyz(pre_gripper)}")
    for bid, p in pre_blocks.items():
        if p.get("status") == "ok":
            print(f"  pre-close {bid} ground truth: {fmt_xyz(p)}")

    TTLServo.servoAngleCtrl(4, -100, 1, 200)
    time.sleep(GRAB_SETTLE_S)

    post_state = arm_state()
    holding = bool(post_state.get("is_holding"))
    post_gripper = post_state.get("gripper_position", {})
    post_blocks = block_poses()
    print(f"  post-close is_holding={holding} gripper_position: {fmt_xyz(post_gripper)}")
    for bid, p in post_blocks.items():
        if p.get("status") == "ok":
            moved = ""
            pb = pre_blocks.get(bid, {})
            if pb.get("status") == "ok":
                dx = p["x"] - pb["x"]; dy = p["y"] - pb["y"]; dz = p["z"] - pb["z"]
                dist = (dx**2 + dy**2 + dz**2) ** 0.5
                moved = f"  [moved {dist*1000:.1f} mm since pre-close]"
            print(f"  post-close {bid} ground truth: {fmt_xyz(p)}{moved}")

    if holding:
        print(f"  *** GRAB CONFIRMED on attempt {i} ***")
        break

    TTLServo.servoAngleCtrl(4, 100, 1, 200)
    time.sleep(0.3)
else:
    print("\n*** ALL CANDIDATES FAILED ***")

print("\n--- BLOCK GROUND TRUTH AFTER FULL LOOP ---")
after = block_poses()
for bid, p in after.items():
    print(f"  {bid}: {p}")
    b = before.get(bid, {})
    if b.get("status") == "ok" and p.get("status") == "ok":
        dx = p["x"] - b["x"]; dy = p["y"] - b["y"]; dz = p["z"] - b["z"]
        dist = (dx**2 + dy**2 + dz**2) ** 0.5
        print(f"    net displacement since before loop: {dist*1000:.1f} mm")
