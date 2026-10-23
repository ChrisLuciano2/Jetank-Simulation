"""Run the real build_tower() (like diag_full_cycle.py) and, when it finishes,
print the TRUE position of both blocks from the sim (get_block_pose -- dev
only) and the horizontal offset of the top block from the bottom one.

"On top" means horizontal offset ~0 and height difference ~ the bottom
block's height. Block A is the large block (bottom), B the small one.
"""
import math

import jetson_utils
from jetbot import Robot
from jetbot_nav import visual_scan
from jetbot_nav.target_seek import SeekingGapFollowController
from robot_coord import Coordinator

import sim_client
from autonomous_tower_build import build_tower, MAX_SLACK, EXCLUSION_RADIUS_M

A, B = "block_A_large", "block_B_small"


def pose(bid):
    r = sim_client.send_query({"command": "get_block_pose", "block_id": bid})
    return r if r and r.get("status") == "ok" else None


def report():
    a, b = pose(A), pose(B)
    if not a or not b:
        print(f"[stack] could not read block poses: A={a} B={b}")
        return
    dx, dz, dy = b["x"] - a["x"], b["z"] - a["z"], b["y"] - a["y"]
    print(f"[stack] A (bottom) x={a['x']:.4f} y={a['y']:.4f} z={a['z']:.4f}")
    print(f"[stack] B (top)    x={b['x']:.4f} y={b['y']:.4f} z={b['z']:.4f}")
    print(f"[stack] B relative to A: dx={dx*1000:+.1f} mm  dz={dz*1000:+.1f} mm  "
          f"horizontal={math.hypot(dx, dz)*1000:.1f} mm  dy={dy*1000:+.1f} mm")


def main():
    robot = Robot()
    camera = jetson_utils.videoSource("csi://0")
    geom = visual_scan.sim_jetank()
    controller = SeekingGapFollowController(max_range=12.0)
    coord = Coordinator(meeting_point={"x": 0.0, "y": 0.0}, max_slack=MAX_SLACK,
                        exclusion_radius=EXCLUSION_RADIUS_M, solo_ok=True)
    coord.connect()
    try:
        build_tower(robot, camera, geom, controller, coord)
    except KeyboardInterrupt:
        print("\n[build] interrupted")
    finally:
        robot.stop()
        report()
        coord.disconnect()


if __name__ == "__main__":
    main()