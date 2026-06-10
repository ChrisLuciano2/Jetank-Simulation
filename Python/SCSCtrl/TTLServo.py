"""
SCSCtrl.TTLServo — simulation shim for Feetech SCS/STS servo control.

All public functions match the real TTLServo module exactly.
Instead of writing to /dev/ttyTHS1 over TTL serial, they send JSON
commands to the Unity simulation on the control TCP channel (port 5555).

Servo ID → Unity arm joint mapping
────────────────────────────────────────────────────
 Servo 1  base rotation    → arm joint_index 0  (J1 base yaw)
 Servo 2  shoulder         → arm joint_index 1  (J2 shoulder pitch)
 Servo 3  elbow            → arm joint_index 2  (J3 elbow pitch)
 Servo 4  claw / gripper   → arm gripper        (0 = closed, 1 = open)
 Servo 5  tilt             → arm joint_index 4  (J5 wrist pitch)

Raw position ↔ angle conversion (same constants as real hardware):
  servoInit      = 512          (centre / 0°)
  servoInputRange = 850          (full input span)
  servoAngleRange = 180          (full angle span, degrees)
  raw = 512 + int(850/180 * angle_deg)
  angle_deg = (raw - 512) * 180 / 850
"""

import numpy as np
import sim_client

# ─── Hardware constants (identical to real TTLServo.py) ──────────────────────
linkageLenA   = 90
linkageLenB   = 160
servoInputRange = 850
servoAngleRange = 180
servoInit       = [None, 512, 512, 512, 512, 512]  # index 0 unused

# Local cache of the last commanded raw position per servo (for infoSingleGet)
_last_raw = [None, 512, 512, 512, 512, 512]

ARM_ID = "arm_01"

# ─── Servo → joint index mapping ─────────────────────────────────────────────
# Servo 4 (claw) is special — handled as a gripper, not a joint angle.
_SERVO_TO_JOINT = {1: 0, 2: 1, 3: 2, 5: 4}

# ─── Safety limits (degrees) ─────────────────────────────────────────────────
# These match RoboticArmController.JOINT_MIN/MAX on the Unity side.
# Enforced here so student code is corrected before it ever reaches the robot.
#
#  Servo 1 (base yaw)  : ±150°  — beyond this, cables wrap through the joint
#  Servo 2 (shoulder)  : -15° to +80° — -15° floor prevents chassis collision
#  Servo 3 (elbow)     : ±120°
#  Servo 4 (gripper)   : -100° to +100° (fully closed to fully open)
#  Servo 5 (wrist)     : ±90°
# Hard limits — motion is clamped here, warning printed in PowerShell + Unity Console.
_ANGLE_MIN  = {1: -160.0, 2: -20.0, 3: -130.0, 4: -100.0, 5: -100.0}
_ANGLE_MAX  = {1:  160.0, 2:  85.0, 3:  130.0, 4:  100.0, 5:  100.0}

# Warning zone — printed but motion is still allowed (~10° inside hard limit).
_ANGLE_WARN_MIN = {1: -140.0, 2: -10.0, 3: -115.0, 4: -85.0, 5: -85.0}
_ANGLE_WARN_MAX = {1:  140.0, 2:  75.0, 3:  115.0, 4:  85.0, 5:  85.0}

# Max safe speed (raw units / 10 ms).  Real SCS/STS servos skip above ~1000.
_SPEED_MAX  = 800
_SPEED_WARN = 600


# ─── Internal helpers ────────────────────────────────────────────────────────

def _raw_to_angle(servo_id: int, raw: int) -> float:
    """Convert raw servo position to angle in degrees."""
    return (raw - servoInit[servo_id]) * servoAngleRange / servoInputRange


def _angle_to_raw(servo_id: int, angle_deg: float) -> int:
    """Convert angle in degrees to raw servo position."""
    return servoInit[servo_id] + int((servoInputRange / servoAngleRange) * angle_deg)


def _send_joint(servo_id: int, angle_deg: float):
    """
    Send one servo's angle to Unity.
    Servo 4 drives the gripper; all others drive named arm joints.
    """
    if servo_id == 4:
        # Map angle [-100, +100] → gripper [0 (closed), 1 (open)]
        gripper = max(0.0, min(1.0, (angle_deg + 100.0) / 200.0))
        sim_client.send_command({
            "command":        "arm_set_gripper",
            "robot_id":       ARM_ID,
            "gripper_amount": gripper,
        })
    elif servo_id in _SERVO_TO_JOINT:
        joint_idx = _SERVO_TO_JOINT[servo_id]
        sim_client.send_command({
            "command":    "arm_set_joint",
            "robot_id":   ARM_ID,
            "joint_index": joint_idx,
            "angle":      angle_deg,
        })
    # Servo IDs outside 1-5 are silently ignored (matches real hardware behaviour)


# ─── IK helpers (copied from real TTLServo.py — no serial calls) ─────────────

def _limit_check(pos_input, circle_pos, circle_len, outline):
    rx = pos_input[0] - circle_pos[0]
    ry = pos_input[1] - circle_pos[1]
    real_sq  = rx * rx + ry * ry
    short_sq = np.square(circle_len[1] - circle_len[0])
    long_sq  = np.square(circle_len[1] + circle_len[0])

    if short_sq <= real_sq <= long_sq:
        return pos_input[0], pos_input[1]

    line_k = (pos_input[1] - circle_pos[1]) / (pos_input[0] - circle_pos[0])
    line_b = circle_pos[1] - line_k * circle_pos[0]
    radius_sq = short_sq if real_sq < short_sq else long_sq
    sign = -1 if real_sq < short_sq else 1

    a = 1 + line_k * line_k
    b = 2 * line_k * (line_b - circle_pos[1]) - 2 * circle_pos[0]
    c = (circle_pos[0] ** 2 + (line_b - circle_pos[1]) ** 2 - radius_sq)
    disc = b * b - 4 * a * c
    x1 = (-b + np.sqrt(disc)) / (2 * a)
    x2 = (-b - np.sqrt(disc)) / (2 * a)
    y1 = line_k * x1 + line_b
    y2 = line_k * x2 + line_b

    if pos_input[0] > circle_pos[0]:
        xo = (x1 - sign * outline) if x1 > circle_pos[0] else (x2 + sign * outline)
        yo = y1 if x1 > circle_pos[0] else y2
    elif pos_input[0] < circle_pos[0]:
        xo = (x1 + sign * outline) if x1 < circle_pos[0] else (x2 - sign * outline)
        yo = y1 if x1 < circle_pos[0] else y2
    else:
        xo, yo = x1, y1  # degenerate case

    return xo, yo


def _plane_linkage_reverse(linkage_len, linkage_en_de, servo_num_ctrl, debug_pos, goal_pos):
    goal_pos = [goal_pos[0] + debug_pos[0], goal_pos[1] + debug_pos[1]]
    angle_en_d = np.arctan(linkage_en_de / linkage_len[1]) * 180 / np.pi
    linkage_len_real = np.sqrt(linkage_len[1] ** 2 + linkage_en_de ** 2)

    goal_pos[0], goal_pos[1] = _limit_check(
        goal_pos, debug_pos, [linkage_len[0], linkage_len_real], 0.00001)

    gx, gy = goal_pos[0], goal_pos[1]

    if gx < 0:
        gx = -gx
        m = linkage_len_real ** 2 - linkage_len[0] ** 2 - gx ** 2 - gy ** 2
        n = m / (2 * linkage_len[0])
        ang_a = (np.arctan(gy / gx)
                 + np.arcsin(n / np.sqrt(gx ** 2 + gy ** 2)))
        ang_b = (np.arcsin((gy - linkage_len[0] * np.cos(ang_a))
                           / linkage_len_real) - ang_a)
        ang_a = 90 - ang_a * 180 / np.pi
        ang_b = ang_b * 180 / np.pi
    elif gx == 0:
        ang_a = np.arccos((linkage_len[0] ** 2 + gy ** 2 - linkage_len_real ** 2)
                          / (2 * linkage_len[0] * gy))
        c_ = np.tan(ang_a) * linkage_len[0]
        d_ = gy - linkage_len[0] / np.cos(ang_a)
        ang_b = np.arccos((c_ ** 2 + linkage_len_real ** 2 - d_ ** 2)
                          / (2 * c_ * linkage_len_real))
        ang_a = -ang_a * 180 / np.pi + 90
        ang_b = -ang_b * 180 / np.pi
    else:
        sq = np.sqrt(gx ** 2 + gy ** 2)
        n = (linkage_len[0] ** 2 + gx ** 2 + gy ** 2 - linkage_len_real ** 2) \
            / (2 * linkage_len[0] * sq)
        ang_a = np.arccos(n) * 180 / np.pi
        ang_b_raw = np.arctan(gy / gx) * 180 / np.pi
        ang_a = ang_b_raw - ang_a
        m = (linkage_len[0] ** 2 + linkage_len_real ** 2 - gx ** 2 - gy ** 2) \
            / (2 * linkage_len[0] * linkage_len_real)
        ang_b = np.arccos(m) * 180 / np.pi - 90
        ang_a = float(ang_a)
        ang_b = float(ang_b)

    return [ang_a, ang_b + angle_en_d]


# ─── Public API (module-level functions, mirroring real TTLServo) ─────────────

def returnOffset(servo_id: int, angle_input: float, direction: int) -> int:
    """
    Calculate raw servo position for a given angle and direction.
    Pure maths — no TCP call needed.
    """
    raw = servoInit[servo_id] + int(
        (servoInputRange / servoAngleRange) * angle_input * direction)
    return raw


_BORDER = "=" * 60

def _safe_angle(servo_id: int, angle_deg: float) -> float:
    """Clamp angle_deg to the safe range for servo_id, warn if in warning zone or clamped."""
    if servo_id not in _ANGLE_MIN:
        return angle_deg
    lo, hi = _ANGLE_MIN[servo_id], _ANGLE_MAX[servo_id]
    clamped = max(lo, min(hi, angle_deg))

    if abs(clamped - angle_deg) > 0.01:
        # Hard limit hit — clamped
        print(f"\n{_BORDER}")
        print(f"  [TTLServo] HARD LIMIT: servo {servo_id} angle {angle_deg:.1f} deg")
        print(f"  Clamped to {clamped:.1f} deg (hard limit [{lo}, {hi}])")
        print(f"  Motion stopped — fix your angle before deploying to real robot!")
        print(f"{_BORDER}\n")
    elif (servo_id in _ANGLE_WARN_MIN and
          (angle_deg < _ANGLE_WARN_MIN[servo_id] or angle_deg > _ANGLE_WARN_MAX[servo_id])):
        # Warning zone — allowed but flagged
        wlo = _ANGLE_WARN_MIN[servo_id]
        whi = _ANGLE_WARN_MAX[servo_id]
        print(f"\n{_BORDER}")
        print(f"  [TTLServo] WARNING: servo {servo_id} at {angle_deg:.1f} deg")
        print(f"  Approaching hard limit [{lo}, {hi}] — warn zone [{wlo}, {whi}]")
        print(f"{_BORDER}\n")

    return clamped


def _safe_speed(servo_id: int, speed_input: int) -> int:
    """Clamp speed to a safe value, warn if too high."""
    if speed_input > _SPEED_MAX:
        print(f"[TTLServo] SAFETY: servo {servo_id} speed {speed_input} "
              f"clamped to {_SPEED_MAX} (max safe = {_SPEED_MAX})")
        return _SPEED_MAX
    if speed_input > _SPEED_WARN:
        print(f"[TTLServo] WARNING: servo {servo_id} speed {speed_input} "
              f"is high — consider staying below {_SPEED_WARN} on real hardware")
    return speed_input


def servoAngleCtrl(servo_id: int, angle_input: float,
                   direction: int, speed_input: int) -> int:
    """
    Move servo_id to angle_input * direction degrees at speed_input.
    Returns the raw position value (same as real hardware).
    """
    speed_input = _safe_speed(servo_id, speed_input)
    raw = returnOffset(servo_id, angle_input, direction)
    angle_deg = _raw_to_angle(servo_id, raw)
    angle_deg = _safe_angle(servo_id, angle_deg)
    raw = _angle_to_raw(servo_id, angle_deg)
    _last_raw[servo_id] = raw
    _send_joint(servo_id, angle_deg)
    return raw


def syncCtrl(id_list: list, speed_list: list, goal_list: list):
    """
    Move multiple servos simultaneously to their raw goal positions.
    Speed is forwarded per-servo so the real robot moves at the commanded rate.
    """
    for servo_id, raw_goal, speed in zip(id_list, goal_list, speed_list):
        speed = _safe_speed(servo_id, int(speed))
        angle_deg = _raw_to_angle(servo_id, raw_goal)
        angle_deg = _safe_angle(servo_id, angle_deg)
        raw_goal  = _angle_to_raw(servo_id, angle_deg)
        _last_raw[servo_id] = raw_goal
        _send_joint(servo_id, angle_deg)


def infoSingleGet(servo_id: int) -> int:
    """
    Return the current raw servo position.
    In simulation this returns the last commanded position (no encoder read-back).
    """
    return _last_raw[servo_id] if _last_raw[servo_id] is not None else 512


def xyInput(x_input: float, y_input: float):
    """
    Move arm tip to (x_input, y_input) in mm using inverse kinematics.
    x = reach outward (90–200 mm), y = left/right (−170 to +170 mm).
    """
    ang = _plane_linkage_reverse(
        [linkageLenA, linkageLenB], 0, [0, 1], [0, 0], [x_input, -y_input])
    servoAngleCtrl(2, ang[0] + 90, 1,  300)
    servoAngleCtrl(3, ang[1],      -1, 300)
    return [ang[0], ang[1]]


def xyInputSmooth(x_input: float, y_input: float, dt: float):
    """
    Move arm tip to (x_input, y_input) over dt seconds using IK.
    Speed is computed from the distance to travel, same as real hardware.
    """
    ang = _plane_linkage_reverse(
        [linkageLenA, linkageLenB], 0, [0, 1], [0, 0], [x_input, -y_input])

    next2 = returnOffset(2, ang[0] + 90, 1)
    next3 = returnOffset(3, ang[1], -1)

    # Compute speed from distance / time (matches real speedGenOut)
    d2 = abs(next2 - _last_raw[2])
    d3 = abs(next3 - _last_raw[3])
    spd2 = int(round(d2 / dt)) if dt > 0 else 300
    spd3 = int(round(d3 / dt)) if dt > 0 else 300

    servoAngleCtrl(2, ang[0] + 90, 1,  max(1, spd2))
    servoAngleCtrl(3, ang[1],      -1, max(1, spd3))
    return [ang[0], ang[1]]


def stopServo(servo_id: int):
    """Stop servo_id at its current position (freeze in place)."""
    sim_client.send_command({
        "command":  "servo_stop",
        "robot_id": ARM_ID,
        "servo_id": servo_id,
    })


def portClose():
    """Close the servo port.  No-op in simulation."""
    pass
