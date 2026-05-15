using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// SimCamera — attach to the scene's Main Camera GameObject.
///
/// Each Unity Update() this script:
///   1. Renders the camera view to a 640×480 RenderTexture.
///   2. Encodes the pixels as JPEG and stores them as a base-64 string.
///   3. Scans the scene for GameObjects tagged "DetectableObject" and
///      computes their screen-space bounding boxes.
///
/// SimQueryServer reads the cached results from a background thread.
/// Because the cache is written once per frame on the main thread and
/// read on the background thread, we use volatile string references
/// (single-word writes are atomic on modern .NET) which is safe enough
/// for this educational use case.
///
/// SETUP:
///   1. Attach this script to the Main Camera in your scene.
///   2. Tag objects you want the detector to "see" with "DetectableObject".
///      (Create the tag via Edit → Project Settings → Tags and Layers.)
///   3. Optionally set captureWidth / captureHeight (default 640×480).
///
/// The detection output format matches the real TensorRT postprocess()
/// expectations: each object is reported as pixel-space bounding box
/// [x1, y1, x2, y2] at 640×640 scale (the student's INPUT_SIZE).
/// </summary>
[RequireComponent(typeof(Camera))]
public class SimCamera : MonoBehaviour
{
    [Header("Capture settings")]
    [SerializeField] private int captureWidth  = 640;
    [SerializeField] private int captureHeight = 480;
    [SerializeField] private int jpegQuality   = 75;

    [Header("Detection settings")]
    [Tooltip("Tag objects that should be reported as 'cough_drop_container'")]
    [SerializeField] private string detectableTag   = "DetectableObject";
    [SerializeField] private string detectionClass  = "cough_drop_container";
    [SerializeField] private float  detectionConf   = 0.92f;

    public static SimCamera Instance { get; private set; }

    private Camera         _cam;
    private RenderTexture  _rt;
    private Texture2D      _tex;

    // Cached results — written on main thread, read on background thread
    private volatile string _cachedFrameB64   = null;
    private volatile string _cachedDetections = "{\"status\":\"ok\",\"objects\":[]}";

    // ── Unity lifecycle ───────────────────────────────────────────────────────

    private void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(this); return; }
        Instance = this;

        _cam = GetComponent<Camera>();
        _rt  = new RenderTexture(captureWidth, captureHeight, 24, RenderTextureFormat.ARGB32);
        _tex = new Texture2D(captureWidth, captureHeight, TextureFormat.RGB24, false);
    }

    private void Update()
    {
        CaptureFrame();
        UpdateDetections();
    }

    private void OnDestroy()
    {
        if (_rt  != null) { _rt.Release();  Destroy(_rt);  }
        if (_tex != null) { Destroy(_tex); }
    }

    // ── Frame capture ─────────────────────────────────────────────────────────

    private void CaptureFrame()
    {
        // Render camera into our RenderTexture
        RenderTexture prev = _cam.targetTexture;
        _cam.targetTexture = _rt;
        _cam.Render();
        _cam.targetTexture = prev;

        // Read pixels from GPU → CPU
        RenderTexture.active = _rt;
        _tex.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0, false);
        _tex.Apply();
        RenderTexture.active = null;

        // Encode to JPEG and base-64
        byte[] jpeg  = _tex.EncodeToJPG(jpegQuality);
        _cachedFrameB64 = System.Convert.ToBase64String(jpeg);
    }

    // ── Object detection ──────────────────────────────────────────────────────

    private void UpdateDetections()
    {
        // Find all tagged GameObjects in the scene (tag may not exist yet)
        GameObject[] targets;
        try { targets = GameObject.FindGameObjectsWithTag(detectableTag); }
        catch { targets = new GameObject[0]; }

        var sb = new StringBuilder();
        sb.Append("{\"status\":\"ok\",\"objects\":[");

        bool first = true;
        foreach (GameObject go in targets)
        {
            if (!IsVisibleToCamera(go, out Rect screenRect))
                continue;

            // Scale from capture resolution to 640×640 (detection space)
            float scaleX = 640f / captureWidth;
            float scaleY = 640f / captureHeight;

            float x1 = screenRect.xMin * scaleX;
            float y1 = screenRect.yMin * scaleY;
            float x2 = screenRect.xMax * scaleX;
            float y2 = screenRect.yMax * scaleY;

            if (!first) sb.Append(",");
            first = false;

            sb.Append("{");
            sb.Append($"\"class\":\"{detectionClass}\",");
            sb.Append($"\"x1\":{x1:F1},");
            sb.Append($"\"y1\":{y1:F1},");
            sb.Append($"\"x2\":{x2:F1},");
            sb.Append($"\"y2\":{y2:F1},");
            sb.Append($"\"conf\":{detectionConf:F2}");
            sb.Append("}");
        }

        sb.Append("]}");
        _cachedDetections = sb.ToString();
    }

    /// <summary>
    /// Returns true if the GameObject has a renderer that is within the
    /// camera's view frustum.  Fills screenRect with the object's AABB
    /// projected to screen space (origin = top-left, y-down).
    /// </summary>
    private bool IsVisibleToCamera(GameObject go, out Rect screenRect)
    {
        screenRect = Rect.zero;

        Renderer rend = go.GetComponentInChildren<Renderer>();
        if (rend == null || !rend.isVisible) return false;

        Bounds bounds = rend.bounds;

        // Project all 8 corners of the world-space AABB to screen space
        Vector3 min = bounds.min;
        Vector3 max = bounds.max;

        Vector3[] corners = new Vector3[8]
        {
            new Vector3(min.x, min.y, min.z),
            new Vector3(max.x, min.y, min.z),
            new Vector3(min.x, max.y, min.z),
            new Vector3(max.x, max.y, min.z),
            new Vector3(min.x, min.y, max.z),
            new Vector3(max.x, min.y, max.z),
            new Vector3(min.x, max.y, max.z),
            new Vector3(max.x, max.y, max.z),
        };

        float sx1 = float.MaxValue, sy1 = float.MaxValue;
        float sx2 = float.MinValue, sy2 = float.MinValue;
        bool anyInFront = false;

        foreach (Vector3 corner in corners)
        {
            Vector3 sp = _cam.WorldToScreenPoint(corner);
            if (sp.z <= 0) continue;   // behind camera
            anyInFront = true;

            // WorldToScreenPoint returns y from bottom; flip to top-left origin
            float screenY = captureHeight - sp.y;

            if (sp.x < sx1) sx1 = sp.x;
            if (sp.x > sx2) sx2 = sp.x;
            if (screenY < sy1) sy1 = screenY;
            if (screenY > sy2) sy2 = screenY;
        }

        if (!anyInFront) return false;

        // Clamp to image bounds
        sx1 = Mathf.Clamp(sx1, 0, captureWidth);
        sx2 = Mathf.Clamp(sx2, 0, captureWidth);
        sy1 = Mathf.Clamp(sy1, 0, captureHeight);
        sy2 = Mathf.Clamp(sy2, 0, captureHeight);

        if (sx2 - sx1 < 2 || sy2 - sy1 < 2) return false;  // too small

        screenRect = new Rect(sx1, sy1, sx2 - sx1, sy2 - sy1);
        return true;
    }

    // ── Public accessors (called from SimQueryServer background thread) ────────

    public string GetCachedFrameBase64()      => _cachedFrameB64;
    public string GetCachedDetectionsJson()   => _cachedDetections;
}
