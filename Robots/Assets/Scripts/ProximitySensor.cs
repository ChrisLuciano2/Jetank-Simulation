using UnityEngine;

/// <summary>
/// ProximitySensor - attach to the robot (JETANK / truck) GameObject.
///
/// Casts an ARC of rays each Update() across scanFovDegrees centered on
/// the robot's forward direction (local +Z) - a cheap 2D "lidar" scan.
/// A wide fan rather than a few fixed rays is deliberate: 30-degree
/// spacing leaves angular blind zones big enough that a narrow object
/// slightly off-center is invisible until contact, and a single
/// wall-side ray grazing a wall at a shallow angle reads "too close"
/// even while the robot is making fine progress alongside it.
///
/// PHYSICAL-HARDWARE PARITY: the real robot must provide an equivalent
/// scan - e.g. an ultrasonic/IR sensor swept by a servo on the JETANK's
/// TTL bus (driven via SCSCtrl), or a low-cost 2D lidar downsampled to
/// rayCount beams. The Python side (jetbot_nav.gap_follow) only needs
/// get_proximity_scan answered with the same shape.
///
/// SETUP:
///   1. Attach to the same GameObject as TruckController.
///   2. Obstacles need Colliders (not just Renderers), Is Trigger OFF.
///
/// The defaults below are matched to jetbot_nav.gap_follow's constants -
/// in particular maxRange MUST stay above Python's GAP_THRESHOLD, or no
/// ray can ever read as "free" and the controller never leaves its
/// recovery states. The failure is silent (rays just come up short, no
/// error), so OnValidate() flags a mismatch in the Inspector.
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

    private void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(this); return; }
        Instance = this;

        var init = new float[rayCount];
        for (int i = 0; i < rayCount; i++) init[i] = maxRange;
        _scan = init;
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

    public float MaxRange => maxRange;
    public float ScanFovDegrees => scanFovDegrees;
    public int RayCount => rayCount;
}