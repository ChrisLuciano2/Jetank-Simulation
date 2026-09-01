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

    [Header("Grab Settings")]
    [Tooltip("How close a GraspableBlock's center must be to the end effector to be picked up when the gripper closes.")]
    [SerializeField] private float grabRadius = 0.6f;
    [Tooltip("Gripper openAmount at or below this counts as \"closed\" for grab purposes.")]
    [SerializeField] private float closedThreshold = 0.15f;
    [Tooltip("Gripper openAmount at or above this counts as \"open\" for release purposes.")]
    [SerializeField] private float openThreshold = 0.85f;

    // Internal targets for smooth motion
    private float[] _targetAngles = new float[6];
    private float[] _currentAngles = new float[6];
    private float _targetGrip = 0.5f;
    private float _currentGrip = 0.5f;

    private Transform _heldObject;
    private bool _wasGripClosed;

    // ── Thread-safe state snapshot ──────────────────────────────────────────
    //
    // SimQueryServer's get_arm_state runs on a background thread (see its own
    // class docs), and Unity's Transform/Component APIs may only be touched
    // from the main thread — GetEndEffectorPosition() etc. throw if called
    // from there. So, same pattern as SimCamera/ProximitySensor: publish an
    // immutable snapshot by reference once per Update() on the main thread,
    // and have the query path read only that snapshot, never a live Transform.
    public class ArmStateSnapshot
    {
        public readonly float[] JointAngles;
        public readonly float GripperAmount;
        public readonly bool IsHoldingSnapshot;
        public readonly string HeldObjectNameSnapshot;
        public readonly Vector3 EndEffectorPosition;
        public readonly Vector3 GripperPosition;

        public ArmStateSnapshot(float[] jointAngles, float gripperAmount, bool isHolding,
                                 string heldObjectName, Vector3 endEffectorPosition,
                                 Vector3 gripperPosition)
        {
            JointAngles = jointAngles;
            GripperAmount = gripperAmount;
            IsHoldingSnapshot = isHolding;
            HeldObjectNameSnapshot = heldObjectName;
            EndEffectorPosition = endEffectorPosition;
            GripperPosition = gripperPosition;
        }
    }

    private volatile ArmStateSnapshot _cachedState;

    /// <summary>Background-thread-safe snapshot of arm state, for SimQueryServer.</summary>
    public ArmStateSnapshot GetCachedState() => _cachedState;

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
        UpdateGrab();

        // A held block rides the end effector rigidly (parented in
        // TryGrab), so there is nothing further to do here for its
        // position — Unity's transform hierarchy handles that every frame
        // on its own.

        _cachedState = new ArmStateSnapshot(
            (float[])_currentAngles.Clone(),
            _currentGrip,
            _heldObject != null,
            _heldObject != null ? _heldObject.name : null,
            GetEndEffectorPosition(),
            GetGripperPosition());
    }

    // ─── GRAB / RELEASE ──────────────────────

    /// <summary>True if the gripper currently has a block attached.</summary>
    public bool IsHolding => _heldObject != null;

    /// <summary>Name of the currently-held block, or null if none.</summary>
    public string HeldObjectName => _heldObject != null ? _heldObject.name : null;

    private void UpdateGrab()
    {
        bool isClosed = _currentGrip <= closedThreshold;
        bool isOpen   = _currentGrip >= openThreshold;

        if (isClosed && !_wasGripClosed && _heldObject == null)
        {
            TryGrab();
        }
        else if (isOpen && _heldObject != null)
        {
            Release();
        }
        _wasGripClosed = isClosed;
    }

    private void TryGrab()
    {
        // Deliberately GetGripperPosition(), not GetEndEffectorPosition(): the
        // latter is j5_wrist, chosen to match what xyInput()'s IK model
        // believes the tip is (see GetEndEffectorPosition()'s doc comment) --
        // correct for keeping xyInput()-driven reach_and_grab() consistent
        // with its own calibration, but NOT where the gripper physically is.
        // Raw joint control (bypassing xyInput() entirely, e.g. arm_ops.py's
        // future floor-reach workaround or diagnostic sweeps) can put the
        // real fingers wherever it wants, ~120-290mm beyond j5_wrist -- using
        // j5_wrist here would silently reject a real, physical grab whenever
        // that gap matters. This is a physical proximity check, so it must
        // use the physical point.
        Vector3 effectorPos = GetGripperPosition();
        Collider[] hits = Physics.OverlapSphere(effectorPos, grabRadius);

        Transform closest = null;
        float closestDist = float.MaxValue;
        foreach (Collider c in hits)
        {
            GraspableBlock block = c.GetComponentInParent<GraspableBlock>();
            if (block == null) continue;

            float d = Vector3.Distance(c.transform.position, effectorPos);
            if (d < closestDist)
            {
                closestDist = d;
                closest = block.transform;
            }
        }

        if (closest != null)
        {
            _heldObject = closest;
            Transform anchor = j6_endEffector != null ? j6_endEffector : transform;
            _heldObject.SetParent(anchor, worldPositionStays: true);
            Debug.Log($"[RoboticArm] Grabbed '{_heldObject.name}' " +
                      $"({closestDist:F2}m from end effector)");
        }
    }

    private void Release()
    {
        if (_heldObject == null) return;
        Debug.Log($"[RoboticArm] Released '{_heldObject.name}' at {_heldObject.position}");
        _heldObject.SetParent(null, worldPositionStays: true);
        _heldObject = null;
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

    /// <summary>Set all 6 joint angles at once (degrees) with safety clamping.</summary>
    public void SetJointAngles(float j1, float j2, float j3, float j4, float j5, float j6)
    {
        SetJoint(0, j1);
        SetJoint(1, j2);
        SetJoint(2, j3);
        SetJoint(3, j4);
        SetJoint(4, j5);
        SetJoint(5, j6);
    }

    // Hard limits — the arm will not move past these under any circumstances.
    // J2 floor -20°: below this the upper arm approaches chassis level.
    // J1 ±160°: beyond this cables begin wrapping through the base joint.
    public static readonly float[] JOINT_MIN  = { -160f, -20f, -130f, -180f, -100f, -180f };
    public static readonly float[] JOINT_MAX  = {  160f,  85f,  130f,  180f,  100f,  180f };

    // Warning zone — logged when angle enters this band before the hard limit.
    // Gives ~10° of notice so students can see the approach in the Console.
    public static readonly float[] WARN_MIN   = { -140f, -10f, -115f, -165f,  -85f, -165f };
    public static readonly float[] WARN_MAX   = {  140f,  75f,  115f,  165f,   85f,  165f };

    /// <summary>Set individual joint by index (0-5) with safety clamping.</summary>
    public void SetJoint(int index, float angleDegrees)
    {
        if (index < 0 || index > 5) return;

        float clamped = Mathf.Clamp(angleDegrees, JOINT_MIN[index], JOINT_MAX[index]);

        // Hard-limit hit
        if (!Mathf.Approximately(clamped, angleDegrees))
        {
            string msg = $"Joint {index} HARD LIMIT: {angleDegrees:F1}° clamped to {clamped:F1}° " +
                         $"(limit [{JOINT_MIN[index]}, {JOINT_MAX[index]}])";
            Debug.LogError($"[RoboticArm] 🛑 {msg}");
            SafetyMonitor.QueueWarning(msg);
        }
        // Warning zone (approaching limit)
        else if (clamped < WARN_MIN[index] || clamped > WARN_MAX[index])
        {
            string msg = $"Joint {index} APPROACHING LIMIT: {clamped:F1}° " +
                         $"(warn zone [{WARN_MIN[index]}, {WARN_MAX[index]}], " +
                         $"hard limit [{JOINT_MIN[index]}, {JOINT_MAX[index]}])";
            Debug.LogWarning($"[RoboticArm] ⚠ {msg}");
            SafetyMonitor.QueueWarning(msg);
        }

        switch (index)
        {
            case 0: J1_BaseYaw       = clamped; break;
            case 1: J2_ShoulderPitch = clamped; break;
            case 2: J3_ElbowPitch    = clamped; break;
            case 3: J4_WristRoll     = clamped; break;
            case 4: J5_WristPitch    = clamped; break;
            case 5: J6_ToolRoll      = clamped; break;
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

    /// <summary>
    /// Get the world position of the arm's grab/reach reference point.
    ///
    /// This is j5_wrist, not j6_endEffector. TTLServo.xyInput()'s 2-link IK
    /// (linkageLenA=90mm shoulder->elbow, linkageLenB=160mm elbow->forearm
    /// pivot) treats the forearm pivot as the tip of its model -- it has no
    /// knowledge of the further ~120mm rigid segment from there out to
    /// j6_endEffector (wrist pitch stays at 0 throughout xyInput commands,
    /// so that segment is real but never independently actuated). Reporting
    /// j6_endEffector here made grab detection expect the IK to place the
    /// tip somewhere the IK's own math can never compute, since
    /// linkageLenA/B are real hardware measurements shared with
    /// TTLServo.py and must not be changed to paper over a sim-only
    /// modeling gap. j5_wrist is where the IK model's own math believes
    /// the tip is (co-located with j4_forearm -- zero offset between them),
    /// so using it here makes GetEndEffectorPosition() consistent with what
    /// xyInput() actually computes.
    /// </summary>
    public Vector3 GetEndEffectorPosition()
    {
        if (j5_wrist != null)
            return j5_wrist.position;
        if (j6_endEffector != null)
            return j6_endEffector.position;
        return transform.position;
    }

    /// <summary>
    /// Get the world position of the physical gripper contact point -- where
    /// a held block's collider actually needs to be for the fingers to close
    /// around it. This is the midpoint of gripperLeft/gripperRight (both
    /// children of j6_endEffector, offset further out from it -- confirmed
    /// in the scene file, e.g. gripperLeft's local position is (-0.055,
    /// 0.17, 0) relative to j6_endEffector, which itself sits 0.12 beyond
    /// j5_wrist). Distinct from GetEndEffectorPosition() (j5_wrist), which
    /// exists to match what xyInput()'s IK model believes the tip is, not
    /// where the real gripper is. Physical grab checks (TryGrab()) must use
    /// this point, not that one.
    /// </summary>
    public Vector3 GetGripperPosition()
    {
        if (gripperLeft != null && gripperRight != null)
            return (gripperLeft.position + gripperRight.position) * 0.5f;
        if (j6_endEffector != null)
            return j6_endEffector.position;
        return GetEndEffectorPosition();
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
