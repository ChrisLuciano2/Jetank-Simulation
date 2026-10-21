"""diag_full_cycle.py plus ground truth during the marker approach.

Wraps autonomous_tower_build.navigate_to_bearing so that, whenever it is
called with track_color set (the walk to the build marker), a background
thread logs the chassis's TRUE world pose once a second via sim_truth
(dev-only), next to the [nav] lines the production code prints. Also logs
each time the gripper releases, so you can compare placement positions.

The marker's world position is not queried; infer it from where block 1 was
placed (the gripper_position printed at that placement), then check whether
the logged robot positions converge on it or wander off.

    py diag_full_cycle_marker.py
"""
import threading
import time

import jetson_utils
from jetbot import Robot
from jetbot_nav import visual_scan
from jetbot_nav.target_seek import SeekingGapFollowController
from robot_coord import Coordinator

import sim_truth
import autonomous_tower_build as atb

_orig_nav = atb.navigate_to_bearing


def _truth_logger(stop):
    t0 = time.time()
    while not stop.is_set():
        p = sim_truth.get_pose()
        if p:
            print(f"[truth t={time.time()-t0:5.1f}s] robot x={p['x']:.2f} z={p['z']:.2f} yaw={p['yaw_deg']:.0f}")
        stop.wait(1.0)


def nav_with_truth(*args, **kwargs):
    if not kwargs.get("track_color"):
        return _orig_nav(*args, **kwargs)
    p = sim_truth.get_pose()
    print(f"[truth] marker approach START robot x={p['x']:.2f} z={p['z']:.2f}" if p else "[truth] no pose")
    stop = threading.Event()
    th = threading.Thread(target=_truth_logger, args=(stop,), daemon=True)
    th.start()
    try:
        return _orig_nav(*args, **kwargs)
    finally:
        stop.set()
        th.join()
        p = sim_truth.get_pose()
        print(f"[truth] marker approach END robot x={p['x']:.2f} z={p['z']:.2f}" if p else "[truth] no pose")


atb.navigate_to_bearing = nav_with_truth


def main():
    robot = Robot()
    camera = jetson_utils.videoSource("csi://0")
    geom = visual_scan.sim_jetank()
    controller = SeekingGapFollowController(max_range=12.0)
    coord = Coordinator(meeting_point={"x": 0.0, "y": 0.0}, max_slack=atb.MAX_SLACK,
                         exclusion_radius=atb.EXCLUSION_RADIUS_M, solo_ok=True)
    coord.connect()
    try:
        atb.build_tower(robot, camera, geom, controller, coord)
    except KeyboardInterrupt:
        print("\n[build] interrupted")
    finally:
        robot.stop()
        coord.disconnect()


if __name__ == "__main__":
    main()