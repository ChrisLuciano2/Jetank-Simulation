using System.Collections.Concurrent;
using UnityEngine;
using RobotSimulator;

/// <summary>
/// SafetyMonitor — attach to the JETANK root GameObject.
///
/// Two responsibilities:
///   1. Runtime checks each frame (ground clearance, tip-over, cable-wrap,
///      shoulder floor) — logs to Unity Console.
///   2. Static warning queue — RoboticArmController and this script enqueue
///      warnings here; SimQueryServer drains the queue so Python's background
///      polling thread can print them to PowerShell as well.
/// </summary>
[RequireComponent(typeof(TruckController))]
public class SafetyMonitor : MonoBehaviour
{
    // ── Static queue (thread-safe) — readable by SimQueryServer ──────────────
    private static readonly ConcurrentQueue<string> _warningQueue
        = new ConcurrentQueue<string>();

    /// <summary>
    /// Enqueue a warning from any script (RoboticArmController, etc.).
    /// SimQueryServer drains this on get_safety_warnings queries.
    /// </summary>
    public static void QueueWarning(string message) => _warningQueue.Enqueue(message);

    /// <summary>Drain and return all pending warnings (called by SimQueryServer).</summary>
    public static string[] DrainWarnings()
    {
        var list = new System.Collections.Generic.List<string>();
        while (_warningQueue.TryDequeue(out string w)) list.Add(w);
        return list.ToArray();
    }

    // ── Inspector settings ────────────────────────────────────────────────────

    [Header("Ground clearance")]
    [SerializeField] private float groundClearanceMin = 0.05f;

    [Header("Tip-over detection")]
    [SerializeField] private float driveSpeedThreshold = 0.55f;
    [SerializeField] private float armExtendThreshold  = 1.2f;

    [Header("Cable wrap — base joint J1")]
    [SerializeField] private float cableWrapWarnAngle  = 140f;   // warn zone
    [SerializeField] private float cableWrapHardAngle  = 160f;   // hard limit

    [Header("Shoulder floor — J2")]
    [SerializeField] private float shoulderWarnAngle   = -10f;   // warn zone
    [SerializeField] private float shoulderHardAngle   = -20f;   // hard limit

    // ── Per-warning cooldown so Console isn't flooded ─────────────────────────
    private float _warnCooldown = 2.0f;
    private enum W { Ground, TipOver, CableWarn, CableHard, ShoulderWarn, ShoulderHard, COUNT }
    private float[] _lastWarn;

    private RoboticArmController _arm;
    private Vector3 _lastPos;
    private float   _speed;

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    private void Start()
    {
        _arm      = GetComponentInChildren<RoboticArmController>();
        _lastWarn = new float[(int)W.COUNT];
        for (int i = 0; i < _lastWarn.Length; i++) _lastWarn[i] = -999f;
        _lastPos  = transform.position;

        if (_arm == null)
            Debug.LogWarning("[SafetyMonitor] No RoboticArmController found.");
    }

    private void LateUpdate()
    {
        _speed   = (transform.position - _lastPos).magnitude / Mathf.Max(Time.deltaTime, 0.001f);
        _lastPos = transform.position;
    }

    private void Update()
    {
        if (_arm == null) return;
        CheckGroundClearance();
        CheckTipOver();
        CheckCableWrap();
        CheckShoulder();
    }

    // ── Runtime checks ────────────────────────────────────────────────────────

    private void CheckGroundClearance()
    {
        float tipY = _arm.GetEndEffectorPosition().y;
        if (tipY < groundClearanceMin)
            Fire(W.Ground, false,
                $"End-effector {tipY * 100f:F0} cm above ground " +
                $"(min {groundClearanceMin * 100f:F0} cm) — " +
                "gripper will hit the floor on the real robot.");
    }

    private void CheckTipOver()
    {
        Vector3 local = transform.InverseTransformPoint(_arm.GetEndEffectorPosition());
        float reach = Mathf.Sqrt(local.x * local.x + local.z * local.z);
        if (reach > armExtendThreshold && _speed > driveSpeedThreshold)
            Fire(W.TipOver, false,
                $"Arm reach={reach:F2} m, drive speed={_speed:F2} m/s — " +
                "high speed with arm extended can tip the real JETANK.");
    }

    private void CheckCableWrap()
    {
        float yaw = _arm.GetJointAngles()[0];
        if (Mathf.Abs(yaw) >= cableWrapHardAngle)
            Fire(W.CableHard, true,
                $"Base J1={yaw:F1}° — HARD LIMIT ±{cableWrapHardAngle}° reached. " +
                "Cables will tear on the real robot beyond this point.");
        else if (Mathf.Abs(yaw) >= cableWrapWarnAngle)
            Fire(W.CableWarn, false,
                $"Base J1={yaw:F1}° — approaching cable-wrap limit ±{cableWrapHardAngle}°.");
    }

    private void CheckShoulder()
    {
        float sh = _arm.GetJointAngles()[1];
        if (sh <= shoulderHardAngle)
            Fire(W.ShoulderHard, true,
                $"Shoulder J2={sh:F1}° — HARD LIMIT {shoulderHardAngle}° reached. " +
                "Upper arm will collide with chassis on the real robot.");
        else if (sh <= shoulderWarnAngle)
            Fire(W.ShoulderWarn, false,
                $"Shoulder J2={sh:F1}° — approaching chassis-collision zone " +
                $"(hard limit {shoulderHardAngle}°).");
    }

    // ── Helper ────────────────────────────────────────────────────────────────

    private void Fire(W id, bool isError, string message)
    {
        float now = Time.time;
        if (now - _lastWarn[(int)id] < _warnCooldown) return;
        _lastWarn[(int)id] = now;

        string tag = isError ? "🛑 SAFETY ERROR" : "⚠ SAFETY WARNING";
        if (isError)
            Debug.LogError($"[SafetyMonitor] {tag}: {message}");
        else
            Debug.LogWarning($"[SafetyMonitor] {tag}: {message}");

        // Enqueue so Python's polling thread prints it to PowerShell too
        QueueWarning($"{tag}: {message}");
    }
}
