"""
sim_truth — ground truth from the simulator. DEV ONLY.

WHY THIS IS NOT IN jetbot_nav
────────────────────────────────────────────────────────────────
Everything under jetbot_nav/ is deployment code: it runs unchanged on the
Jetson, where the only sensor is a camera. The real JETANK has no
encoders, no IMU, and no way at all to answer "where am I". Any
navigation logic that reached for this module would pass in simulation
and fail on hardware, which is the worst possible failure ordering.

So this lives outside the package, and the rule is one line long:
tests and validation harnesses may import it; jetbot_nav may not.

WHAT IT IS FOR
────────────────────────────────────────────────────────────────
Grading. A live navigation run reports its own course error, measured by
the very estimator under test — if that estimator is wrong, the run
reports success while driving into a wall. That is not a hypothetical:
the first live course-keeping run reported plausible headings the whole
way through while the visual gyro was railed at its correlation search
limit, fabricating +/-28 deg steps. Ground truth is how that gets caught.

Needs Unity playing, and a `get_pose` query on the C# side
(SimQueryServer -> ProximitySensor.GetCachedPose).

USAGE
    import sim_truth
    sim_truth.get_pose()              -> {"x":.., "y":.., "z":.., "yaw_deg":..}
    sim_truth.yaw_error(now, ref)     -> signed degrees, wrapped to (-180, 180]
"""

import sim_client


def get_pose():
    """
    Ground-truth robot pose, or None if the query failed.

    Returns {"x", "y", "z", "yaw_deg"}. yaw_deg is Unity's eulerAngles.y,
    so it reads 0-360 exactly as the Inspector shows it. Use yaw_error()
    to compare two of them rather than subtracting directly — see there
    for why.
    """
    resp = sim_client.send_query({"command": "get_pose"})
    if not resp or resp.get("status") != "ok":
        return None
    return {
        "x": float(resp["x"]),
        "y": float(resp["y"]),
        "z": float(resp["z"]),
        "yaw_deg": float(resp["yaw_deg"]),
    }


def yaw_error(yaw_deg: float, reference_deg: float) -> float:
    """
    Signed yaw difference in degrees, wrapped to (-180, 180].

    Positive means `yaw_deg` is to the RIGHT of `reference_deg`, matching
    jetbot_nav's convention throughout (Unity's eulerAngles.y also
    increases when turning right, so no negation is needed).

    Wrapping is the whole point of this function. Raw Unity yaw is 0-360,
    so a robot that starts at 2 deg and drifts 4 deg left reads 358, and a
    naive subtraction calls that a 356 deg error instead of -4. That kind
    of mistake looks like a catastrophic failure in a results table and
    sends you debugging the wrong thing.
    """
    return (yaw_deg - reference_deg + 180.0) % 360.0 - 180.0


def displacement(pose, reference_pose) -> tuple:
    """
    (forward, lateral) travel in the reference pose's frame, in world units.

    Lateral is the number Phase 4 flagged as unfixed: course keeping
    restores DIRECTION, so a successful run ends parallel to the original
    course but can be well off to one side of it. Reporting only heading
    error would hide that entirely.
    """
    import math
    dx = pose["x"] - reference_pose["x"]
    dz = pose["z"] - reference_pose["z"]
    a = math.radians(reference_pose["yaw_deg"])
    # Unity yaw is clockwise from +Z, so forward is (sin a, cos a) and the
    # right-hand normal is (cos a, -sin a).
    forward = dx * math.sin(a) + dz * math.cos(a)
    lateral = dx * math.cos(a) - dz * math.sin(a)
    return forward, lateral
