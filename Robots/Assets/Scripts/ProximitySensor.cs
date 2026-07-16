using UnityEngine;

/// <summary>
/// ProximitySensor - attach to the truck (robot) GameObject.
///
/// Casts an ARC of rays each Update() across scanFovDegrees centered on
/// the robot's forward direction (local +Z) - a cheap 2D "lidar" scan.
/// This replaced the original 3-fixed-ray design (0 / +-30 deg), which
/// had blind zones wide enough that narrow objects slightly off-center
/// were invisible until contact, and whose single wall-side ray caused
/// direction-oscillation when driving parallel to walls.
///
/// PHYSICAL-HARDWARE PARITY: the real robot must provide an equivalent
/// scan - e.g. an ultrasonic/IR sensor swept by a servo on the JETANK's
/// TTL bus (driven via SCSCtrl), or a low-cost 2D lidar downsampled to
/// rayCount beams. The Python side (jetbot_nav.gap_follow) only needs
/// get_proximity_scan answered with the same shape. Builds limited to 3
/// fixed sensors can keep using the legacy get_proximity query, which
/// this component still answers (rays nearest to -30/0/+30 deg).
///
/// SETUP:
///   1. Attach to the same GameObject as TruckController.
///   2. Obstacles need Colliders (not just Renderers), Is Trigger OFF.
///
/// SCENE UPGRADE: older scenes serialized this component with maxRange=5,
/// scanFovDegrees=?, rayCount=3 (from the original 3-fixed-ray design).
/// Unity's Inspector preserves those serialized values over the script's
/// new defaults, which used to require a manual per-scene fix (Max Range
/// = 12, Scan Fov = 120, Ray Count = 13) - easy to miss and the failure
/// mode is silent (rays just come up short, no error). Awake() now
/// detects and self-corrects stale values instead, logging a warning so
/// it's visible but doesn't block play.
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

    // Known-good values for the current N-ray gap-following controller.
    // Anything below these came from a scene serialized under the old
    // 3-fixed-ray design and gets corrected at startup - see Awake().
    private const float MIN_SANE_MAX_RANGE = 12.0f;
    private const float MIN_SANE_FOV = 120f;
    private const int MIN_SANE_RAY_COUNT = 13;

    public static ProximitySensor Instance { get; private set; }

    // Republished by reference swap each frame - safe to read from the
    // SimQueryServer background thread.
    private volatile float[] _scan;

    // Legacy 3-value cache (rays nearest -30 / 0 / +30 deg).
    private volatile float _cachedLeft;
    private volatile float _cachedCenter;
    private volatile float _cachedRight;

    private void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(this); return; }
        Instance = this;

        // Self-correct stale values from a scene serialized under the old
        // 3-fixed-ray design (see class doc "SCENE UPGRADE"). This used to
        // be a manual Inspector step per scene; skipping it silently made
        // rays read maxRange too early, which is indistinguishable from
        // "nothing there" to the Python side (see gap_follow's GAP
        // detection) and was the root cause of one navigation bug already.
        if (maxRange < MIN_SANE_MAX_RANGE)
        {
            Debug.LogWarning($"[ProximitySensor] maxRange={maxRange} looks like a " +
                $"stale pre-scan-upgrade value (needs >= {MIN_SANE_MAX_RANGE} so " +
                "Python's GAP_THRESHOLD is reachable). Correcting to " +
                $"{MIN_SANE_MAX_RANGE} for this run - update the Inspector value " +
                "to silence this.");
            maxRange = MIN_SANE_MAX_RANGE;
        }
        if (scanFovDegrees < MIN_SANE_FOV)
        {
            Debug.LogWarning($"[ProximitySensor] scanFovDegrees={scanFovDegrees} looks " +
                $"stale. Correcting to {MIN_SANE_FOV} for this run - update the " +
                "Inspector value to silence this.");
            scanFovDegrees = MIN_SANE_FOV;
        }
        if (rayCount < MIN_SANE_RAY_COUNT)
        {
            Debug.LogWarning($"[ProximitySensor] rayCount={rayCount} looks like the old " +
                $"3-fixed-ray setup. Correcting to {MIN_SANE_RAY_COUNT} for this run - " +
                "update the Inspector value to silence this.");
            rayCount = MIN_SANE_RAY_COUNT;
        }

        var init = new float[rayCount];
        for (int i = 0; i < rayCount; i++) init[i] = maxRange;
        _scan = init;
        _cachedLeft = _cachedCenter = _cachedRight = maxRange;
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

        // Legacy 3-value view for the old get_proximity query.
        _cachedLeft = scan[NearestRay(-30f)];
        _cachedCenter = scan[NearestRay(0f)];
        _cachedRight = scan[NearestRay(30f)];
    }

    private int NearestRay(float angleDeg)
    {
        float step = scanFovDegrees / (rayCount - 1);
        float start = -scanFovDegrees * 0.5f;
        int i = Mathf.RoundToInt((angleDeg - start) / step);
        return Mathf.Clamp(i, 0, rayCount - 1);
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

    /// <summary>Legacy 3-value view (background-thread safe).</summary>
    public (float left, float center, float right) GetCachedDistances()
        => (_cachedLeft, _cachedCenter, _cachedRight);

    public float MaxRange => maxRange;
    public float ScanFovDegrees => scanFovDegrees;
    public int RayCount => rayCount;
}