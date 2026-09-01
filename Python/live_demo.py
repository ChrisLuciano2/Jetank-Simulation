"""
live_demo.py — Watchable proof that robot_coord actually paces two robots
against each other, driving the two simulated JETANK trucks in Unity.

Not the real build logic (that's example_collab_build.py, with perception
stubs to fill in) — this drives straight toward a shared meeting point in
fixed discrete steps so the *pacing itself* is the thing on screen: robot_b
is deliberately slower, so you should see robot_a advance one step, then
stop and visibly wait — not just steamroll to the middle — until robot_b
catches up, because MAX_SLACK caps how far ahead it's allowed to get.

Run two copies, one per terminal, with Unity already in Play mode
(TcpServer only listens on 5555 while playing) and a broker reachable at
MQTT_BROKER_HOST (defaults to 127.0.0.1):

    ROBOT_ID=robot_a py -3.12 live_demo.py
    ROBOT_ID=robot_b py -3.12 live_demo.py

Uses sim_client directly (not jetbot.Robot()) for the "goto" command —
that's Unity-sim-only convenience, not part of the real JetBot API, which
is why this script is a demo, not something that also runs on hardware.
"""

import time

import sim_client
import sim_robot_id
from robot_coord import Coordinator

TOTAL_STEPS = 5
MEETING_X = 2.0          # halfway between JETANK (x=0) and JETANK (1) (x=4)
MAX_SLACK = 1
EXCLUSION_RADIUS = 1.0   # generous, so the pause is easy to see on screen

_START_X = 0.0 if sim_robot_id.TRUCK_ID == "truck_01" else 4.0
_DIRECTION = 1.0 if sim_robot_id.TRUCK_ID == "truck_01" else -1.0
_STEP_SIZE = abs(MEETING_X - _START_X) / TOTAL_STEPS
_STEP_DELAY = 4.0 if sim_robot_id.TRUCK_ID == "truck_01" else 1.2  # robot_a is the deliberately slow one


def goto(x, z=0.0):
    sim_client.send_command({
        "command": "goto",
        "robot_id": sim_robot_id.TRUCK_ID,
        "x": x,
        "z": z,
    })


def main():
    coord = Coordinator(meeting_point={"x": MEETING_X, "y": 0.0}, max_slack=MAX_SLACK,
                         exclusion_radius=EXCLUSION_RADIUS, wait_timeout=30.0)

    print(f"[demo] {coord.robot_id} ({sim_robot_id.TRUCK_ID}) connecting to MQTT...")
    coord.connect()

    x = _START_X
    goto(x)
    coord.publish_state("idle", pose={"x": x, "y": 0.0, "heading_deg": 0.0}, blocks_placed=0)

    for step in range(1, TOTAL_STEPS + 1):
        print(f"[demo] {coord.robot_id} step {step}/{TOTAL_STEPS}: "
              f"requesting pacing clearance (blocks_placed={step})...")
        t0 = time.time()
        ok = coord.wait_for_pacing_clearance(step)
        waited = time.time() - t0
        if waited > 0.5:
            print(f"[demo] {coord.robot_id} WAITED {waited:.1f}s for the other robot to catch up")
        if not ok:
            print(f"[demo] {coord.robot_id} pacing wait timed out, proceeding anyway (demo only)")

        x += _STEP_SIZE * _DIRECTION
        coord.publish_state("moving_to_place", pose={"x": x, "y": 0.0, "heading_deg": 0.0},
                             blocks_placed=step - 1)

        if not coord.wait_for_zone_clearance({"x": x, "y": 0.0}):
            print(f"[demo] {coord.robot_id} zone wait timed out, proceeding anyway (demo only)")

        print(f"[demo] {coord.robot_id} driving to x={x:.2f}")
        goto(x)
        time.sleep(_STEP_DELAY)

        coord.publish_state("placed_clear", pose={"x": x, "y": 0.0, "heading_deg": 0.0},
                             blocks_placed=step)
        print(f"[demo] {coord.robot_id} placed_clear at step {step}, x={x:.2f}")
        time.sleep(0.3)

    print(f"[demo] {coord.robot_id} done — final x={x:.2f}")
    coord.publish_state("idle", pose={"x": x, "y": 0.0, "heading_deg": 0.0}, blocks_placed=TOTAL_STEPS)
    coord.disconnect()


if __name__ == "__main__":
    main()
