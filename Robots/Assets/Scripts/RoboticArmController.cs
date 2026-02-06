using UnityEngine;

/// <summary>
/// RoboticArmController — Attach to the root "base_plate" GameObject after importing the .glb/.obj.
/// Controls a 6-DOF robotic arm + gripper. Set joint angles via properties or SetJointAngles().
///
/// SETUP (after import):
///   1. The .glb import creates the correct parent-child hierarchy automatically.
///      For .obj: run RoboticArmAssembler.AssembleFromOBJ() once to build hierarchy.
///   2. Attach this script to the "base_plate" root object.
///   3. Click "Auto-Assign Joints" in the Inspector context menu, or assign manually.
///
/// JOINT MAP:
///   J1 = j1_turntable    (Y-axis rotation, base yaw)
///   J2 = j2_shoulder     (X-axis rotation, shoulder pitch)
///   J3 = j3_upper_arm    (X-axis rotation, elbow pitch)
///   J4 = j4_forearm      (Y-axis rotation, wrist roll)
///   J5 = j5_wrist        (X-axis rotation, wrist pitch)
///   J6 = j6_end_effector (Y-axis rotation, tool roll)
/// </summary>
public class RoboticArmController : MonoBehaviour
{
    [Header("Joint Transforms (auto-assigned or drag manually)")]
    public Transform j1_turntable;
    public Transform j2_shoulder;
    public Transform j3_upperArm;
    public Transform j4_forearm;
    public Transform j5_wrist;
    public Transform j6_endEffector;
    public Transform gripperLeft;
    public Transform gripperRight;

    [Header("Joint Angles (degrees)")]
    [Range(-180f, 180f)] public float J1_BaseYaw = 0f;
    [Range(-90f, 90f)]   public float J2_ShoulderPitch = 0f;
    [Range(-135f, 135f)] public float J3_ElbowPitch = 0f;
    [Range(-180f, 180f)] public float J4_WristRoll = 0f;
    [Range(-120f, 120f)] public float J5_WristPitch = 0f;
    [Range(-180f, 180f)] public float J6_ToolRoll = 0f;

    [Header("Gripper")]
    [Range(0f, 1f)] public float GripperOpenAmount = 0.5f;

    [Header("Motion Settings")]
    public float jointSpeed = 90f;  // degrees per second for smooth motion
    public bool useSmoothMotion = true;

    // Internal targets for smooth motion
    private float[] _targetAngles = new float[6];
    private float[] _currentAngles = new float[6];
    private float _targetGrip = 0.5f;
    private float _currentGrip = 0.5f;

    // Gripper geometry
    private float _gripMaxOffset = 0.022f;
    private Vector3 _gripLeftBase, _gripRightBase;
    private bool _initialized = false;

    void Start()
    {
        AutoAssignJoints();
        if (gripperLeft != null) _gripLeftBase = gripperLeft.localPosition;
        if (gripperRight != null) _gripRightBase = gripperRight.localPosition;
        _currentAngles = new float[] { J1_BaseYaw, J2_ShoulderPitch, J3_ElbowPitch,
                                        J4_WristRoll, J5_WristPitch, J6_ToolRoll };
        _targetAngles = (float[])_currentAngles.Clone();
        _currentGrip = GripperOpenAmount;
        _initialized = true;
    }

    void Update()
    {
        if (!_initialized) return;

        _targetAngles[0] = J1_BaseYaw;
        _targetAngles[1] = J2_ShoulderPitch;
        _targetAngles[2] = J3_ElbowPitch;
        _targetAngles[3] = J4_WristRoll;
        _targetAngles[4] = J5_WristPitch;
        _targetAngles[5] = J6_ToolRoll;
        _targetGrip = GripperOpenAmount;

        if (useSmoothMotion)
        {
            float step = jointSpeed * Time.deltaTime;
            for (int i = 0; i < 6; i++)
                _currentAngles[i] = Mathf.MoveTowards(_currentAngles[i], _targetAngles[i], step);
            _currentGrip = Mathf.MoveTowards(_currentGrip, _targetGrip, step * 0.02f);
        }
        else
        {
            _currentAngles = (float[])_targetAngles.Clone();
            _currentGrip = _targetGrip;
        }

        ApplyJointRotations();
        ApplyGripper();
    }

    void ApplyJointRotations()
    {
        if (j1_turntable != null)
            j1_turntable.localRotation = Quaternion.Euler(0, _currentAngles[0], 0);
        if (j2_shoulder != null)
            j2_shoulder.localRotation = Quaternion.Euler(_currentAngles[1], 0, 0);
        if (j3_upperArm != null)
            j3_upperArm.localRotation = Quaternion.Euler(_currentAngles[2], 0, 0);
        if (j4_forearm != null)
            j4_forearm.localRotation = Quaternion.Euler(0, _currentAngles[3], 0);
        if (j5_wrist != null)
            j5_wrist.localRotation = Quaternion.Euler(_currentAngles[4], 0, 0);
        if (j6_endEffector != null)
            j6_endEffector.localRotation = Quaternion.Euler(0, _currentAngles[5], 0);
    }

    void ApplyGripper()
    {
        float offset = Mathf.Lerp(0, _gripMaxOffset, _currentGrip);
        if (gripperLeft != null)
            gripperLeft.localPosition = _gripLeftBase + new Vector3(-offset, 0, 0);
        if (gripperRight != null)
            gripperRight.localPosition = _gripRightBase + new Vector3(offset, 0, 0);
    }

    // ─── PUBLIC API ──────────────────────────

    /// <summary>Set all 6 joint angles at once (degrees).</summary>
    public void SetJointAngles(float j1, float j2, float j3, float j4, float j5, float j6)
    {
        J1_BaseYaw = Mathf.Clamp(j1, -180f, 180f);
        J2_ShoulderPitch = Mathf.Clamp(j2, -90f, 90f);
        J3_ElbowPitch = Mathf.Clamp(j3, -135f, 135f);
        J4_WristRoll = Mathf.Clamp(j4, -180f, 180f);
        J5_WristPitch = Mathf.Clamp(j5, -120f, 120f);
        J6_ToolRoll = Mathf.Clamp(j6, -180f, 180f);
    }

    /// <summary>Set individual joint by index (0-5).</summary>
    public void SetJoint(int index, float angleDegrees)
    {
        switch (index)
        {
            case 0: J1_BaseYaw = angleDegrees; break;
            case 1: J2_ShoulderPitch = angleDegrees; break;
            case 2: J3_ElbowPitch = angleDegrees; break;
            case 3: J4_WristRoll = angleDegrees; break;
            case 4: J5_WristPitch = angleDegrees; break;
            case 5: J6_ToolRoll = angleDegrees; break;
        }
    }

    /// <summary>Get current joint angles as array.</summary>
    public float[] GetJointAngles()
    {
        return new float[] { J1_BaseYaw, J2_ShoulderPitch, J3_ElbowPitch,
                             J4_WristRoll, J5_WristPitch, J6_ToolRoll };
    }

    /// <summary>Open/close gripper: 0 = closed, 1 = fully open.</summary>
    public void SetGripper(float openAmount)
    {
        GripperOpenAmount = Mathf.Clamp01(openAmount);
    }

    /// <summary>Move to home position (all zeros).</summary>
    public void GoHome()
    {
        SetJointAngles(0, 0, 0, 0, 0, 0);
        SetGripper(0.5f);
    }

    /// <summary>Get the world position of the end effector tip.</summary>
    public Vector3 GetEndEffectorPosition()
    {
        if (j6_endEffector != null)
            return j6_endEffector.position;
        return transform.position;
    }

    /// <summary>Get the world rotation of the end effector.</summary>
    public Quaternion GetEndEffectorRotation()
    {
        if (j6_endEffector != null)
            return j6_endEffector.rotation;
        return transform.rotation;
    }

    // ─── PRESET POSES ───────────────────────

    /// <summary>Move arm to a pickup-ready pose.</summary>
    public void PosePickupReady()
    {
        SetJointAngles(0, -30, 60, 0, -30, 0);
        SetGripper(1f);
    }

    /// <summary>Move arm to a folded/parked pose.</summary>
    public void PoseParked()
    {
        SetJointAngles(0, -80, 130, 0, -50, 0);
        SetGripper(0f);
    }

    /// <summary>Move arm to reach forward.</summary>
    public void PoseReachForward()
    {
        SetJointAngles(0, 0, 0, 0, 0, 0);
        SetGripper(0.5f);
    }

    // ─── AUTO SETUP ─────────────────────────

    [ContextMenu("Auto-Assign Joints")]
    public void AutoAssignJoints()
    {
        j1_turntable = FindInChildren("j1_turntable");
        j2_shoulder = FindInChildren("j2_shoulder");
        j3_upperArm = FindInChildren("j3_upper_arm");
        j4_forearm = FindInChildren("j4_forearm");
        j5_wrist = FindInChildren("j5_wrist");
        j6_endEffector = FindInChildren("j6_end_effector");
        gripperLeft = FindInChildren("gripper_left");
        gripperRight = FindInChildren("gripper_right");

        if (j1_turntable == null) Debug.LogWarning("RoboticArm: Could not find j1_turntable");
    }

    Transform FindInChildren(string name)
    {
        foreach (Transform t in GetComponentsInChildren<Transform>(true))
        {
            if (t.name.ToLower().Replace("-","_") == name.ToLower().Replace("-","_"))
                return t;
        }
        return null;
    }
}
