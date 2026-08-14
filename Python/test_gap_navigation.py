"""
test_gap_navigation.py — live test for the scan-based gap follower.

USAGE
    py test_gap_navigation.py                 drive 10s
    py test_gap_navigation.py 30              drive 30s
    py test_gap_navigation.py 30 run1.csv     drive 30s AND record telemetry
    py test_gap_navigation.py --replay run1.csv   re-run a recording OFFLINE

SETUP (Unity)
    1. The truck has the UPDATED ProximitySensor attached (the N-ray scan
       version). In the Inspector set: Max Range = 12, Scan Fov = 120,
       Ray Count = 13. Older scenes have the previous values serialized —
       the script defaults alone will NOT update an existing scene.
    2. With the scene running you should see a yellow fan of 13 debug rays
       in the Scene view — if you still see only 3 rays, the old component
       is active.
    3. Obstacles need Colliders, Is Trigger unchecked.

TROUBLESHOOTING ORDER when something misbehaves:
    1. py test_gap_logic.py         — offline closed-loop scenarios (no Unity)
    2. record a live run with a CSV path, then --replay it offline
    3. only then start changing constants

Runs identically on the physical robot once it answers get_proximity_scan
(servo-swept ultrasonic or downsampled lidar — see gap_follow.py docstring).
"""

import sys

from jetbot_nav import gap_follow

args = sys.argv[1:]

if args and args[0] == "--replay":
    if len(args) < 2:
        print("usage: py test_gap_navigation.py --replay run1.csv")
        sys.exit(1)
    gap_follow.replay_log(args[1])
    sys.exit(0)

duration = float(args[0]) if len(args) >= 1 else 10.0
log_path = args[1] if len(args) >= 2 else None

print(f"[test_gap_navigation] Driving with gap following for {duration}s"
      + (f", logging to {log_path}" if log_path else ""))
gap_follow.drive_with_gap_following(duration=duration, log_path=log_path)
print("[test_gap_navigation] Done")
