"""
test_navigation.py — live obstacle-avoidance test against the simulation.

USAGE
    py test_navigation.py                 drive 10s
    py test_navigation.py 30              drive 30s
    py test_navigation.py 30 run1.csv     drive 30s AND record telemetry
    py test_navigation.py --replay run1.csv   re-run a recorded run OFFLINE
                                              (no Unity needed) to debug it

SETUP (Unity)
    1. The truck GameObject has ProximitySensor attached, obstacles have
       Colliders (any shape; "Is Trigger" unchecked).
    2. IMPORTANT: set the ProximitySensor's Max Range to 12 in the Inspector.
       Older scenes have 5.0 serialized, which is SHORTER than the Python
       slow-down distance — the run will still work (Python auto-clamps and
       prints a warning) but with reduced look-ahead.
    3. Press Play, then run this script.

TROUBLESHOOTING WORKFLOW
    - Something behaved oddly during a live run? Record it with a CSV path,
      then step through the exact same sensor data offline with --replay.
    - Logic-only regressions are covered by test_avoidance_logic.py, which
      needs no Unity at all — run that FIRST when navigation misbehaves.

Runs identically on the physical Jetson (same file, same command) as long
as the robot exposes an equivalent get_proximity() sensor array.
"""

import sys

from jetbot_nav import navigation

args = sys.argv[1:]

if args and args[0] == "--replay":
    if len(args) < 2:
        print("usage: py test_navigation.py --replay run1.csv")
        sys.exit(1)
    navigation.replay_log(args[1])
    sys.exit(0)

duration = float(args[0]) if len(args) >= 1 else 10.0
log_path = args[1] if len(args) >= 2 else None

print(f"[test_navigation] Driving with obstacle avoidance for {duration}s"
      + (f", logging to {log_path}" if log_path else ""))
navigation.drive_with_avoidance(duration=duration, log_path=log_path)
print("[test_navigation] Done")