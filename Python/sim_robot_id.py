"""
sim_robot_id — Resolves which simulated robot this process controls.

The shim modules (jetbot/robot.py, SCSCtrl/TTLServo.py) import this to
pick their Unity robot_id, instead of hardcoding "truck_01"/"arm_01".
That hardcoding was fine for one robot; it silently breaks two-robot
testing, since both Python processes would drive the same simulated
truck. Reading ROBOT_ID instead lets two processes on the same machine
each drive a different simulated robot:

    ROBOT_ID=robot_a python build_script.py   # drives truck_01 / arm_01
    ROBOT_ID=robot_b python build_script.py   # drives truck_02 / arm_02

Unset ROBOT_ID defaults to robot_a, so every existing single-robot
script/test keeps working unchanged.

This module is sim-only — the shims that use it are themselves sim-only
and get shadowed by the real jetbot/SCSCtrl libraries on hardware, where
there's exactly one robot and no concept of robot_id at the hardware API
level. On the real robots, ROBOT_ID is read separately by
robot_coord.config, purely for the MQTT coordination layer.

Only robot_a/robot_b are mapped because the Unity scene only wires up two
robots (see Robots/Assets/Scenes/MainScene.unity — JETANK and JETANK_B).
Add a case here, plus a matching robot in the scene, before using a third.
"""

import os

_LOGICAL_ID = os.environ.get("ROBOT_ID", "robot_a")

_TRUCK_IDS = {"robot_a": "truck_01", "robot_b": "truck_02"}
_ARM_IDS   = {"robot_a": "arm_01",   "robot_b": "arm_02"}

if _LOGICAL_ID not in _TRUCK_IDS:
    raise ValueError(
        f"ROBOT_ID={_LOGICAL_ID!r} not recognized by the simulator; "
        f"expected one of {sorted(_TRUCK_IDS)}"
    )

TRUCK_ID = _TRUCK_IDS[_LOGICAL_ID]
ARM_ID   = _ARM_IDS[_LOGICAL_ID]
