"""
validate_course_keep — live closed-loop course keeping, graded against truth.

MOVES THE ROBOT. Needs Unity playing.

WHY THIS EXISTS
────────────────────────────────────────────────────────────────
drive_keeping_course() prints the course error it BELIEVES it has, which
is produced by the estimator under test. When that estimator is wrong the
run reports plausible numbers the whole way into a wall — which is
exactly what the first live run did, reporting steady headings while
VisualGyro was railed at its correlation limit inventing 28 deg steps.

So this harness runs the same control loop but logs Unity's ground-truth
pose beside every estimate, and scores the run on the truth. The headline
number is not "did it think it was on course" but "was it".

WHAT IT REPORTS
    - estimator error vs truth, per tick and worst-case
    - final heading error (the thing course keeping exists to fix)
    - final LATERAL offset (the thing it explicitly does not fix — see
      course_keep's known-limitation note; heading alone can look perfect
      while the robot ends up in the next room over)
    - how often each heading source answered, and how often the estimator
      went UNKNOWN rather than guessing

USAGE
    py -3.8 validate_course_keep.py                 # 30 s from (0, 0, yaw 0)
    py -3.8 validate_course_keep.py --duration 45
    py -3.8 validate_course_keep.py --start -3,4,0  # x,z,yaw
"""

import sys
import time

import sim_client
import sim_truth
from jetbot_nav import course_keep, visual_scan

# ─── Options ─────────────────────────────────────────────────────────────────

duration = 30.0
start = (0.0, 0.0, 0.0)          # x, z, yaw_deg

for i, a in enumerate(sys.argv):
    if a == "--duration" and i + 1 < len(sys.argv):
        duration = float(sys.argv[i + 1])
    if a == "--start" and i + 1 < len(sys.argv):
        parts = [float(v) for v in sys.argv[i + 1].split(",")]
        start = (parts[0], parts[1], parts[2])

POLL_HZ = 10.0

# Refuse to start boxed in. A run that begins with an obstacle already
# inside the camera's blind radius tests nothing except the blind radius,
# and it produces exactly the BACKUP thrash that makes a log unreadable.
MIN_START_CLEARANCE = 1.5

# ─── Connect ─────────────────────────────────────────────────────────────────

if not sim_client.connect():
    print("Could not connect to Unity. Is the sim playing?")
    sys.exit(1)

probe = sim_truth.get_pose()
if probe is None:
    print("get_pose query failed. Unity is playing, but the C# side does not\n"
          "answer 'get_pose' — Stop play mode, let Unity recompile, then Play\n"
          "again. Unity does not recompile while playing.")
    sim_client.disconnect()
    sys.exit(1)

# ─── Place the robot at a known, repeatable start ────────────────────────────

print(f"placing robot at x={start[0]} z={start[1]} yaw={start[2]}")
sim_client.send_command({"command": "set_position",
                         "x": start[0], "y": 0.0, "z": start[1]})
sim_client.send_command({"command": "set_rotation", "rotation_y": start[2]})
time.sleep(0.5)

import jetson_utils

camera = jetson_utils.videoSource("csi://0")
img = camera.Capture()
if img is None:
    print("No camera frame.")
    sim_client.disconnect()
    sys.exit(1)
frame = jetson_utils.cudaToNumpy(img)

geom = visual_scan.sim_jetank(width=frame.shape[1], height=frame.shape[0])
scan0 = visual_scan.free_space_scan(frame, geom, n_rays=13)
nearest0 = min(scan0["distances"])
print(f"start clearance: nearest obstacle {nearest0:.2f} units")
if nearest0 < MIN_START_CLEARANCE:
    print(f"ABORT: less than {MIN_START_CLEARANCE} units of clearance at the "
          f"start pose.\nPick another --start; this one only exercises the "
          f"blind zone.")
    sim_client.disconnect()
    sys.exit(1)

origin = sim_truth.get_pose()
course_yaw = origin["yaw_deg"]
print(f"course set: yaw {course_yaw:.2f} deg\n")

# ─── Run ─────────────────────────────────────────────────────────────────────

keeper = course_keep.CourseKeeper(geom)
if not keeper.set_course(frame):
    print("WARNING: could not lock onto the view — too little texture. The run "
          "will degrade to plain obstacle avoidance and prove nothing about "
          "course keeping.")

from jetbot import Robot

robot = Robot()
interval = 1.0 / POLL_HZ
deadline = time.time() + duration

ticks = []
try:
    while time.time() < deadline:
        img = camera.Capture()
        if img is None:
            time.sleep(interval)
            continue
        frame = jetson_utils.cudaToNumpy(img)

        scan = visual_scan.free_space_scan(frame, geom, n_rays=13)
        left, right = keeper.step(scan["distances"], frame=frame)
        robot.set_motors(left, right)

        pose = sim_truth.get_pose()
        true_err = sim_truth.yaw_error(pose["yaw_deg"], course_yaw)
        d = keeper.debug()
        est = d["course_error_deg"]

        ticks.append({
            "state": d["state"],
            "course_state": d["course_state"],
            "est": est,
            "true": true_err,
            "src": "lock" if d["drift_free"] else ("gyro" if est is not None
                                                   else "none"),
            "nearest": min(scan["distances"]),
        })
        time.sleep(interval)
finally:
    robot.stop()

final = sim_truth.get_pose()
sim_client.disconnect()

# ─── Score ───────────────────────────────────────────────────────────────────

print(f"\n{len(ticks)} ticks in {duration:.0f}s "
      f"({len(ticks) / duration:.1f} Hz)\n")


def histogram(key):
    counts = {}
    for t in ticks:
        counts[t[key]] = counts.get(t[key], 0) + 1
    return "  ".join(f"{k}={v}" for k, v in sorted(counts.items()))


print("gap_follow state :", histogram("state"))
print("course state     :", histogram("course_state"))
print("heading source   :", histogram("src"))

measured = [t for t in ticks if t["est"] is not None]
if measured:
    devs = [abs(t["est"] - t["true"]) for t in measured]
    worst = max(devs)
    print(f"\nestimator vs truth: mean {sum(devs) / len(devs):.2f} deg, "
          f"worst {worst:.2f} deg, over {len(measured)}/{len(ticks)} ticks "
          f"where an estimate existed")
else:
    print("\nestimator never produced a heading at all")

final_err = sim_truth.yaw_error(final["yaw_deg"], course_yaw)
forward, lateral = sim_truth.displacement(final, origin)

print(f"\nFINAL (ground truth)")
print(f"  heading error : {final_err:+.2f} deg")
print(f"  travelled     : {forward:+.2f} forward, {lateral:+.2f} lateral")

# The verdict deliberately splits "held its course" from "the estimator was
# honest", because a run can pass one and fail the other and they need
# different fixes.
ok_course = abs(final_err) <= 10.0
ok_honest = (not measured) or max(abs(t["est"] - t["true"])
                                  for t in measured) <= 10.0
print(f"\n  held course (|err| <= 10 deg)      : "
      f"{'PASS' if ok_course else 'FAIL'}")
print(f"  estimator honest (<= 10 deg error) : "
      f"{'PASS' if ok_honest else 'FAIL'}")
sys.exit(0 if (ok_course and ok_honest) else 1)
