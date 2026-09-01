"""Instrumented replica of creep_to_target()'s per-tick matching logic,
printing every candidate's bearing, ray-based distance, and floor-
projection distance, to see exactly why a candidate does or doesn't get
accepted -- rather than only seeing the final held/frozen values in the
CSV telemetry.
"""
import time

import jetson_utils
from jetbot import Robot
from jetbot_nav import perception, visual_scan
from jetbot_nav.target_seek import bearing_from_blob, estimate_target_distance
from autonomous_tower_build import (
    _blob_floor_distance, _creep_steer, CREEP_MAX_DISTANCE_JUMP_M,
    navigate_to_bearing, sense_and_sort_blocks_with_search,
)
from jetbot_nav.target_seek import SeekingGapFollowController
import sim_client, sim_robot_id

robot = Robot()
camera = jetson_utils.videoSource("csi://0")
geom = visual_scan.sim_jetank()
controller = SeekingGapFollowController(max_range=12.0)


def capture():
    img = camera.Capture()
    return jetson_utils.cudaToNumpy(img)


sensed = sense_and_sort_blocks_with_search(robot, camera, geom)
print("sensed:", sensed)
target = sensed[0]
navigate_to_bearing(robot, controller, camera, geom, target["bearing_deg"])

last_bearing = target["bearing_deg"]
last_distance = None

for tick in range(20):
    frame = capture()
    blobs = perception.detect_color_blobs(frame, "red")
    scan = visual_scan.free_space_scan(frame, geom)
    candidates = [(bearing_from_blob(b, geom.hfov_deg, image_width=geom.width), b) for b in blobs]

    print(f"--- tick {tick}: last_bearing={last_bearing}, last_distance={last_distance} ---")
    for bearing, blob in candidates:
        ray_dist = estimate_target_distance(bearing, scan)
        floor_dist = _blob_floor_distance(blob, geom)
        print(f"  candidate bearing={bearing:.1f} ray_dist={ray_dist} floor_dist={floor_dist} "
              f"bbox=({blob['x1']:.0f},{blob['y1']:.0f})-({blob['x2']:.0f},{blob['y2']:.0f})")

    scored = [(b, blob, estimate_target_distance(b, scan)) for b, blob in candidates]
    valid = [s for s in scored if s[2] is not None]

    if last_distance is None:
        if candidates:
            bearing, blob = min(candidates, key=lambda c: abs(c[0] - last_bearing))
            dist = estimate_target_distance(bearing, scan) or _blob_floor_distance(blob, geom)
        else:
            bearing, blob, dist = None, None, None
    elif valid:
        bearing, blob, dist = min(valid, key=lambda s: abs(s[2] - last_distance))
        rejected = abs(dist - last_distance) > CREEP_MAX_DISTANCE_JUMP_M
        print(f"  -> ray match: bearing={bearing:.1f} dist={dist:.3f} rejected={rejected}")
        if rejected:
            bearing, blob, dist = None, None, None
    elif candidates:
        fb, fblob = min(candidates, key=lambda c: abs(c[0] - last_bearing))
        fdist = _blob_floor_distance(fblob, geom)
        print(f"  -> fallback candidate: bearing={fb:.1f} floor_dist={fdist}")
        if fdist is not None and abs(fdist - last_distance) <= CREEP_MAX_DISTANCE_JUMP_M:
            bearing, blob, dist = fb, fblob, fdist
        else:
            bearing, blob, dist = None, None, None
            print(f"  -> fallback REJECTED (jump={None if fdist is None else abs(fdist-last_distance):.3f})")
    else:
        bearing, blob, dist = None, None, None

    if bearing is not None:
        last_bearing, last_distance = bearing, dist
        l, r = _creep_steer(bearing)
        robot.set_motors(l, r)
    else:
        print("  -> LOST this tick")
        robot.set_motors(0, 0)

    time.sleep(0.1)  # matches POLL_HZ=10.0 in autonomous_tower_build.py

robot.stop()
