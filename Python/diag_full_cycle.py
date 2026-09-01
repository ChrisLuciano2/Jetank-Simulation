"""End-to-end verification: run autonomous_tower_build.build_tower() for
real, exactly as main() does, except with Coordinator(solo_ok=True) so a
one-robot test run doesn't block on a nonexistent partner's pacing/zone
signals -- this is purely a coordination-layer dev convenience (see
robot_coord.coordinator's own docstring), not a change to the sense/
navigate/grab/place pipeline itself, which is exactly what main() runs.
"""
import jetson_utils
from jetbot import Robot
from jetbot_nav import visual_scan
from jetbot_nav.target_seek import SeekingGapFollowController
from robot_coord import Coordinator

from autonomous_tower_build import build_tower, MAX_SLACK, EXCLUSION_RADIUS_M


def main():
    robot = Robot()
    camera = jetson_utils.videoSource("csi://0")
    geom = visual_scan.sim_jetank()
    controller = SeekingGapFollowController(max_range=12.0)

    coord = Coordinator(meeting_point={"x": 0.0, "y": 0.0}, max_slack=MAX_SLACK,
                         exclusion_radius=EXCLUSION_RADIUS_M, solo_ok=True)
    print(f"[build] connecting as {coord.robot_id}...")
    coord.connect()

    try:
        build_tower(robot, camera, geom, controller, coord)
    except KeyboardInterrupt:
        print("\n[build] interrupted")
    finally:
        robot.stop()
        coord.disconnect()


if __name__ == "__main__":
    main()
