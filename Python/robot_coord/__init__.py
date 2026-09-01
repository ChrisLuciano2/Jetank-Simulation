"""
robot_coord — Inter-robot coordination layer for the two-robot collaborative
build (see collaborative-build-brainstorm.md at the repo root).

Like jetbot_nav, this is **not** a hardware shim. It contains no sim-specific
code and never talks to Unity's TCP ports (5555/5556) — those only exist in
simulation. Coordination happens over MQTT, directly between the two robot
processes, so the exact same script runs unchanged whether the robot
underneath is the Unity sim or a physical JetTank.

    from robot_coord import Coordinator

    coord = Coordinator(meeting_point={"x": 1.5, "y": 0.0})
    coord.connect()
    coord.publish_state("idle", pose=my_pose)

See example_collab_build.py at the Python/ root for a full usage skeleton.
"""

from .coordinator import Coordinator
from .world_model import SharedWorldModel
from . import config, messages

__all__ = ["Coordinator", "SharedWorldModel", "config", "messages"]
