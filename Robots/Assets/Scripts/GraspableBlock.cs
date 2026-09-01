using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Marker + metadata for objects RoboticArmController's gripper can
/// physically pick up in simulation.
///
/// This is deliberately separate from the "DetectableObject" tag used for
/// camera detection: several non-block props (Door_Left, Door_Right, Pole)
/// share that tag for vision purposes but obviously should not be
/// liftable. A block can carry both — the tag makes it visible to
/// detect_objects/perception, this component makes it liftable — without
/// Unity's one-tag-per-GameObject limit forcing a choice between them.
///
/// Attach to any block's root GameObject (the one with the Collider) that
/// should be grabbable.
///
/// IMPORTANT: this component decides only WHETHER the gripper can
/// mechanically close around something and carry it — a simulation
/// approximation for the real gripper's physical grip. It does not decide
/// or report block SIZE for the "largest on bottom" ordering; that comes
/// from the camera in Python (jetbot_nav.block_sensing), the same way it
/// would have to on real hardware. Keeping those two concerns apart matters:
/// if this component also reported size, the whole sorting problem the
/// robots are supposed to solve for themselves would just be pre-answered
/// in the scene file.
/// </summary>
public class GraspableBlock : MonoBehaviour
{
    [Tooltip("Optional human-readable id, useful only in Console logs.")]
    public string blockId;

    // ── Dev-only ground truth ────────────────────────────────────────────
    //
    // Same pattern/rationale as ProximitySensor's per-robot pose registry:
    // a background-thread-safe snapshot, published by reference swap once
    // per Update() on the main thread (Transform reads are main-thread-only
    // in Unity), so SimQueryServer's get_block_pose can grade a real,
    // physical grab attempt against ground truth instead of trusting
    // sensed/telemetry values alone. Diagnostic-only, same as get_pose —
    // nothing in jetbot_nav may read this on real hardware, which has no
    // way to know a block's true position independent of its own camera.
    private static readonly Dictionary<string, GraspableBlock> _byBlockId = new Dictionary<string, GraspableBlock>();

    public static GraspableBlock Get(string blockId) =>
        _byBlockId.TryGetValue(blockId ?? "", out var b) ? b : null;

    // float[3], not Vector3 -- Vector3 is a struct, and a volatile field
    // can't be a struct type (CS0677). An array is a reference type, so the
    // same publish-by-swap-reference pattern ProximitySensor's _pose uses
    // works here too.
    private volatile float[] _cachedPosition;
    public Vector3 GetCachedPosition()
    {
        float[] p = _cachedPosition;
        return new Vector3(p[0], p[1], p[2]);
    }

    private void Awake()
    {
        if (!string.IsNullOrEmpty(blockId))
        {
            if (_byBlockId.ContainsKey(blockId))
                Debug.LogWarning($"[GraspableBlock] Duplicate blockId '{blockId}' — " +
                                  "get_block_pose will only reach one of them.");
            else
                _byBlockId[blockId] = this;
        }
        Vector3 pos = transform.position;
        _cachedPosition = new float[] { pos.x, pos.y, pos.z };
    }

    private void Update()
    {
        Vector3 pos = transform.position;
        _cachedPosition = new float[] { pos.x, pos.y, pos.z };
    }

    private void OnDestroy()
    {
        if (!string.IsNullOrEmpty(blockId) &&
            _byBlockId.TryGetValue(blockId, out var mine) && mine == this)
            _byBlockId.Remove(blockId);
    }
}
