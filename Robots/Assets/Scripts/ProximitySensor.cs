using UnityEngine;

/// <summary>
/// ProximitySensor — attach to the truck (robot) GameObject.
///
/// Casts THREE rays each Update(): front-left, front-center, front-right
/// (local +Z is forward, per the truck's orientation). This mimics a
/// small array of ultrasonic/IR sensors a real JetBot could carry —
/// enough directional info for Python to decide not just "stop" but
/// "which way has more room."
///
/// SETUP:
///   1. Attach this script to the same GameObject as TruckController.
///   2. Adjust maxRange / sideAngleDegrees / rayOriginOffset as needed.
///   3. Obstacles need Colliders (not just Renderers) to be detected.
///
/// SimQueryServer reads cached values from a background thread, same
/// pattern as SimCamera.
/// </summary>
public class ProximitySensor : MonoBehaviour
{
    [Header("Sensor settings")]
    [Tooltip("Maximum distance any ray can report (meters)")]
    [SerializeField] private float maxRange = 5.0f;

    [Tooltip("Angle (degrees) of the left/right rays from forward")]
    [SerializeField] private float sideAngleDegrees = 30f;

    [Tooltip("Local-space offset for each ray's starting point, e.g. front bumper height")]
    [SerializeField] private Vector3 rayOriginOffset = new Vector3(0f, 0.2f, 0f);

    [Tooltip("Layers the rays should hit. Defaults to everything.")]
    [SerializeField] private LayerMask hitLayers = ~0;

    public static ProximitySensor Instance { get; private set; }

    // Cached results — written on main thread, read on background thread
    private volatile float _cachedLeft;
    private volatile float _cachedCenter;
    private volatile float _cachedRight;

    private void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(this); return; }
        Instance = this;
        _cachedLeft = _cachedCenter = _cachedRight = maxRange;
    }

    private void Update()
    {
        Vector3 origin = transform.TransformPoint(rayOriginOffset);

        Vector3 fwdCenter = transform.forward;
        Vector3 fwdLeft = Quaternion.AngleAxis(-sideAngleDegrees, transform.up) * fwdCenter;
        Vector3 fwdRight = Quaternion.AngleAxis(sideAngleDegrees, transform.up) * fwdCenter;

        _cachedCenter = CastRay(origin, fwdCenter);
        _cachedLeft = CastRay(origin, fwdLeft);
        _cachedRight = CastRay(origin, fwdRight);
    }

    private float CastRay(Vector3 origin, Vector3 direction)
    {
        float dist = maxRange;
        if (Physics.Raycast(origin, direction, out RaycastHit hit, maxRange, hitLayers))
        {
            dist = hit.distance;
        }
        Debug.DrawRay(origin, direction * dist, Color.yellow);
        return dist;
    }

    /// <summary>Called from SimQueryServer's background thread.</summary>
    public (float left, float center, float right) GetCachedDistances()
        => (_cachedLeft, _cachedCenter, _cachedRight);

    public float MaxRange => maxRange;
}