"""
test_navigation.py — quick manual test for jetbot_nav.navigation.

Run against the simulation:
    1. Press Play in Unity. Make sure the truck has ProximitySensor
       attached, and obstacles in the scene have Colliders.
    2. cd SenSym-Robot/Python
    3. py test_navigation.py           (drives for 10s by default)
    4. py test_navigation.py 20        (drives for 20s)

Run on the physical Jetson: identical file, identical command — the
robot's own get_proximity()-equivalent sensor array feeds the same
avoidance logic.
"""

import sys

from jetbot_nav import navigation

duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

print(f"[test_navigation] Driving with obstacle avoidance for {duration}s")
navigation.drive_with_avoidance(duration=duration)
print("[test_navigation] Done")
