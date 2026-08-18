"""
test_camera_nav.py — the closed-loop scenarios, run through a CAMERA.

test_gap_logic.py exercises gap_follow against an ideal 120 deg lidar fan
with no minimum range. The JETANK has no such sensor. This file re-runs
the identical scenarios with test_gap_logic.SENSOR swapped to the camera
model — ~75 deg of ground FOV and a ~0.82-unit near blind zone, matching
jetbot_nav.visual_scan.sim_jetank() — so the cost of real sensing is
measured rather than assumed.

Two things get harder, and they are not tuning problems:

  1. NO PERIPHERAL VISION. gap_follow's corridor clearance decides an
     obstacle is "beside us, not in our way" by projecting each ray's hit
     to a lateral offset. With 120 deg it could actually see beside
     itself; at 75 deg the rays that used to supply that evidence do not
     exist, so a wall alongside the robot is invisible rather than
     known-harmless.
  2. NEAR BLIND ZONE. Below min_range an obstacle reads as OPEN FLOOR.
     Approaching something, its base eventually drops out of the frame
     and forward clearance JUMPS from "nearly touching" to "totally
     clear" — so the controller accelerates into whatever it was about
     to avoid. A range sensor cannot produce this reading at all.

    py -3.8 test_camera_nav.py        (summary)
    py -3.8 test_camera_nav.py -v     (per-tick trace on failures)
"""

import sys

import test_gap_logic as G

# Swap the sensor BEFORE any scenario builds a MiniSim or a controller —
# both read this global at construction time.
G.SENSOR = G.CAMERA
G._results = []

SCENARIOS = [
    G.scenario_open_field,
    G.scenario_wall_parallel,
    G.scenario_wall_head_on,
    G.scenario_thin_pole,
    G.scenario_dead_end,
    G.scenario_doorway,
    G.scenario_single_cube,
]

print(f"sensor: {G.CAMERA.name} — {G.CAMERA.fov_deg:.0f} deg fov, "
      f"{G.CAMERA.ray_count} rays, blind inside {G.CAMERA.min_range} units")
print("(compare: test_gap_logic.py runs the same scenarios at "
      f"{G.LIDAR.fov_deg:.0f} deg with no blind zone)\n")

for scenario in SCENARIOS:
    scenario()

passed = sum(G._results)
total = len(G._results)
print(f"\n{passed}/{total} checks passed under the camera model")
sys.exit(0 if passed == total else 1)
