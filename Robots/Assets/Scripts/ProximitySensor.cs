using UnityEngine;

/// <summary>
/// ProximitySensor - DEVELOPMENT GROUND TRUTH ONLY. Not a deployment path.
///
/// Casts an ARC of rays each Update() across scanFovDegrees centered on
/// the robot's forward direction (local +Z) and answers
/// get_proximity_scan with the result.
///
/// THIS IS NOT HARDWARE THE ROBOT HAS. The physical JETANK has a camera
/// and nothing else - no lidar, no ultrasonic. Casting 13 rays
/// simultaneously every frame is a 2D lidar, and an earlier claim that a
/// servo-swept ultrasonic was an equivalent substitute does not hold up:
/// a swept sensor returns readings SEQUENTIALLY over ~0.5-2 s (during
/// which the robot has moved and rotated, while gap_follow's tick-based
/// debounce assumes one coherent instantaneous snapshot), and it sees a
/// wide echo cone rather than an infinitely thin ray.
///
/// The real sensing path is jetbot_nav.visual_scan, which derives the
/// same {angles, distances} shape from the RobotCamera image via
/// ground-plane projection, and therefore runs identically on the Jetson.
///
/// KEEP THIS COMPONENT for what it is genuinely good for: it reports
/// exact geometric truth, so it is the reference to check visual_scan
/// against. If the two disagree, the vision pipeline is miscalibrated
/// (or is hitting one of its documented blind spots - overhangs,
/// floor-coloured objects, non-flat ground). Useful precisely BECAUSE it
/// cheats. Never ship behaviour that depends on it.
///
/// SETUP:
///   1. Attach to the same GameObject as TruckController.
///   2. Obstacles need Colliders (not just Renderers), Is Trigger OFF.
///
/// SimQueryServer reads cached values from a background thread; the scan
/// array is republished by reference swap each frame so readers always
/// see a consistent snapshot.
/// </summary>
public class ProximitySensor : MonoBehaviour
{
    [Header("Scan settings")]
    [Tooltip("Maximum distance any ray can report (meters). MUST exceed Python's GAP_THRESHOLD or no ray can ever read as 'free'. Reported to Python as max_range.")]
    [SerializeField] private float maxRange = 12.0f;

    [Tooltip("Total field of view of the scan arc, centered on forward (degrees)")]
    [SerializeField] private float scanFovDegrees = 120f;

    [Tooltip("Number of rays across the arc (odd keeps one ray dead-center)")]
    [SerializeField] private int rayCount = 13;

    [Tooltip("Local-space offset for each ray's starting point, e.g. front bumper height")]
    [SerializeField] private Vector3 rayOriginOffset = new Vector3(0f, 0.2f, 0f);

    [Tooltip("Layers the rays should hit. Defaults to everything.")]
    [SerializeField] private LayerMask hitLayers = ~0;

    // Values jetbot_nav.gap_follow's constants assume. Going below these
    // doesn't error, it just quietly starves the controller of "free"
    // rays, so OnValidate() warns rather than letting it pass silently.
    private const float MIN_USEFUL_MAX_RANGE = 12.0f;
    private const float MIN_USEFUL_FOV = 120f;
    private const int MIN_USEFUL_RAY_COUNT = 13;

    public static ProximitySensor Instance { get; private set; }

    // Republished by reference swap each frame - safe to read from the
    // SimQueryServer background thread.
    private volatile float[] _scan;

    // Robot pose in world space: x, y, z, yaw-degrees. Same publish-by-swap
    // rule as _scan, and for the same reason - Unity transforms may only be
    // touched on the main thread, so the query server reads this snapshot
    // instead.
    //
    // This lives on ProximitySensor because it is the same KIND of thing: dev
    // -only ground truth with no counterpart on the real JETANK, which has no
    // encoders, no IMU and no way whatsoever to know where it is. Nothing in
    // the deployment path may read it. It exists so a live navigation run can
    // be graded against the truth rather than against the estimator's own
    // opinion of itself.
    private volatile float[] _pose;

    private void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(this); return; }
        Instance = this;

        var init = new float[rayCount];
        for (int i = 0; i < rayCount; i++) init[i] = maxRange;
        _scan = init;

        _pose = PoseSnapshot();
    }

    private void OnValidate()
    {
        rayCount = Mathf.Max(2, rayCount);

        if (maxRange < MIN_USEFUL_MAX_RANGE)
            Debug.LogWarning($"[ProximitySensor] maxRange={maxRange} is below " +
                $"{MIN_USEFUL_MAX_RANGE}, which jetbot_nav.gap_follow's " +
                "GAP_THRESHOLD assumes. Rays will read short and the controller " +
                "may never find a gap to steer through.", this);

        if (scanFovDegrees < MIN_USEFUL_FOV || rayCount < MIN_USEFUL_RAY_COUNT)
            Debug.LogWarning($"[ProximitySensor] scan is {scanFovDegrees} deg over " +
                $"{rayCount} rays; gap_follow is tuned for {MIN_USEFUL_FOV} deg " +
                $"over {MIN_USEFUL_RAY_COUNT}. Wider ray spacing reopens the " +
                "angular blind zones the fan exists to close.", this);
    }

    private void Update()
    {
        Vector3 origin = transform.TransformPoint(rayOriginOffset);
        float step = scanFovDegrees / (rayCount - 1);
        float start = -scanFovDegrees * 0.5f;

        var scan = new float[rayCount];
        for (int i = 0; i < rayCount; i++)
        {
            float angle = start + i * step;             // negative = left
            Vector3 dir = Quaternion.AngleAxis(angle, transform.up)
                          * transform.forward;
            scan[i] = CastRay(origin, dir);
        }
        _scan = scan;                                   // atomic publish
        _pose = PoseSnapshot();
    }

    /// <summary>
    /// World pose as {x, y, z, yawDegrees}. Yaw is Unity's eulerAngles.y, so
    /// it reads 0-360 exactly as the Inspector shows it; wrapping to a signed
    /// range is left to the caller doing the comparison. Turning right
    /// increases it, which already matches jetbot_nav's "positive = right".
    /// </summary>
    private float[] PoseSnapshot()
    {
        Vector3 p = transform.position;
        return new float[] { p.x, p.y, p.z, transform.eulerAngles.y };
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

    /// <summary>Full scan snapshot (background-thread safe).</summary>
    public float[] GetCachedScan() => _scan;

    /// <summary>
    /// World pose snapshot {x, y, z, yawDegrees} (background-thread safe).
    /// Dev-only ground truth - see the _pose field for why nothing in the
    /// deployment path may use this.
    /// </summary>
    public float[] GetCachedPose() => _pose;

    public float MaxRange => maxRange;
    public float ScanFovDegrees => scanFovDegrees;
    public int RayCount => rayCount;
}