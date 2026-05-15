using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using RobotSimulator;
using RobotSimulator.Communication;

/// <summary>
/// Tools → Build JETANK Robot
///
/// Removes the MiniCargoTruck and builds a proportional JETANK model from
/// Unity primitives.  Matches the real Waveshare JETANK layout:
///   - Tank chassis (~23 cm × 17 cm × 6.5 cm)
///   - Rubber track assemblies on each side
///   - 4 visible road wheels per side
///   - 5-servo arm (base yaw / shoulder / elbow / wrist / gripper)
///   - All joint names match RoboticArmController.AutoAssignJoints()
/// </summary>
public static class BuildJETANK
{
    // ── colour palette ────────────────────────────────────────────────────────
    static readonly Color COL_CHASSIS = new Color(0.10f, 0.16f, 0.42f); // dark blue
    static readonly Color COL_TRACK   = new Color(0.07f, 0.07f, 0.07f); // near-black rubber
    static readonly Color COL_WHEEL   = new Color(0.18f, 0.18f, 0.18f); // dark-grey plastic
    static readonly Color COL_TOP     = new Color(0.14f, 0.20f, 0.48f); // slightly lighter blue
    static readonly Color COL_ARM     = new Color(0.72f, 0.72f, 0.72f); // silver
    static readonly Color COL_JOINT   = new Color(0.55f, 0.55f, 0.60f); // slightly darker joint
    static readonly Color COL_GRIP    = new Color(0.92f, 0.42f, 0.04f); // orange gripper

    // ── scale constants (Unity units ≈ 10 cm each) ───────────────────────────
    // Chassis  230 mm L × 170 mm W × 65 mm H  →  2.3 × 1.7 × 0.65
    const float CW = 1.7f;   // chassis width  (X)
    const float CH = 0.65f;  // chassis height (Y)
    const float CL = 2.3f;   // chassis length (Z)
    const float TW = 0.30f;  // track width    (X)
    const float TH = 0.56f;  // track height   (Y)
    const float TL = 2.55f;  // track length   (Z, slightly longer than chassis)

    // Arm link lengths from real hardware (mm → units)
    // linkageLenA = 90 mm, linkageLenB = 160 mm
    const float LINK_A = 0.90f;  // shoulder → elbow
    const float LINK_B = 1.60f;  // elbow    → wrist

    [MenuItem("Tools/Build JETANK Robot")]
    public static void Build()
    {
        // ── 1. Remove old truck / arm ─────────────────────────────────────────
        RemoveNamed("Truck_01", "Truck", "Truck(Clone)", "JETANK");

        // ── 2. Build JETANK root ──────────────────────────────────────────────
        GameObject root = new GameObject("JETANK");
        // Place so the bottom of the tracks sits exactly on y = 0
        root.transform.position = new Vector3(0f, TH * 0.5f, 0f);

        // ── 3. Chassis body ───────────────────────────────────────────────────
        GameObject chassis = Box("Chassis_Body", root, Vector3.zero,
                                 new Vector3(CW, CH, CL), COL_CHASSIS);

        // Top plate (slightly raised, slightly lighter)
        Box("Top_Plate", root, new Vector3(0f, CH * 0.5f + 0.02f, 0f),
            new Vector3(CW - 0.05f, 0.05f, CL - 0.05f), COL_TOP);

        // ── 4. Tracks ─────────────────────────────────────────────────────────
        float tx = CW * 0.5f + TW * 0.5f + 0.02f;
        float ty = -(CH - TH) * 0.5f;   // tracks hang slightly lower than chassis
        Box("Track_Left",  root, new Vector3(-tx, ty, 0f), new Vector3(TW, TH, TL), COL_TRACK);
        Box("Track_Right", root, new Vector3( tx, ty, 0f), new Vector3(TW, TH, TL), COL_TRACK);

        // ── 5. Road wheels (4 per side) ───────────────────────────────────────
        float wy = ty - TH * 0.5f + 0.18f;        // wheel centre height
        float[] wz = { -0.88f, -0.28f, 0.28f, 0.88f };
        foreach (float wpos in wz)
        {
            Wheel("Wheel_L", root, new Vector3(-tx, wy, wpos));
            Wheel("Wheel_R", root, new Vector3( tx, wy, wpos));
        }

        // Drive sprocket (front) and idler (rear) — slightly larger
        Wheel("Sprocket_L", root, new Vector3(-tx, wy + 0.04f,  1.15f), 0.22f);
        Wheel("Sprocket_R", root, new Vector3( tx, wy + 0.04f,  1.15f), 0.22f);
        Wheel("Idler_L",    root, new Vector3(-tx, wy + 0.04f, -1.15f), 0.22f);
        Wheel("Idler_R",    root, new Vector3( tx, wy + 0.04f, -1.15f), 0.22f);

        // ── 6. Jetson Nano board silhouette (top centre-rear) ─────────────────
        Box("Jetson_Board", root,
            new Vector3(0f, CH * 0.5f + 0.045f, -0.4f),
            new Vector3(0.70f, 0.04f, 0.60f),
            new Color(0.05f, 0.35f, 0.05f));   // PCB green

        // ── 7. Robotic Arm ────────────────────────────────────────────────────
        // Arm is mounted front-centre on top of the chassis
        GameObject armRoot = new GameObject("RoboticArm");
        armRoot.transform.SetParent(root.transform);
        // Local position: top of chassis, slightly forward
        armRoot.transform.localPosition = new Vector3(0f, CH * 0.5f + 0.05f, 0.55f);
        armRoot.transform.localRotation = Quaternion.identity;
        armRoot.transform.localScale    = Vector3.one;

        BuildArm(armRoot);

        // Add arm controllers to the arm root
        RoboticArmController armCtrl = armRoot.AddComponent<RoboticArmController>();
        armRoot.AddComponent<RoboticArmNetworkController>();
        armCtrl.AutoAssignJoints();

        // ── 8. Add TruckController + SafetyMonitor to JETANK root ────────────
        root.AddComponent<TruckController>();
        root.AddComponent<SafetyMonitor>();

        // ── 9. Make sure TcpServer and SimQueryServer exist ───────────────────
        if (Object.FindFirstObjectByType<TcpServer>() == null)
            new GameObject("TcpServer").AddComponent<TcpServer>();
        if (Object.FindFirstObjectByType<SimQueryServer>() == null)
            new GameObject("SimQueryServer").AddComponent<SimQueryServer>();
        if (Camera.main != null && Camera.main.GetComponent<SimCamera>() == null)
            Camera.main.gameObject.AddComponent<SimCamera>();

        // ── 10. Ground plane ──────────────────────────────────────────────────
        if (GameObject.Find("Ground") == null)
        {
            GameObject gnd = GameObject.CreatePrimitive(PrimitiveType.Plane);
            gnd.name = "Ground";
            gnd.transform.localScale = new Vector3(5f, 1f, 5f);
            ApplyColor(gnd, new Color(0.28f, 0.42f, 0.28f));
        }

        // ── 11. Camera position ───────────────────────────────────────────────
        if (Camera.main != null)
        {
            Camera.main.transform.position = new Vector3(0f, 3.0f, -4.5f);
            Camera.main.transform.rotation = Quaternion.Euler(28f, 0f, 0f);
        }

        // ── 12. Player settings ───────────────────────────────────────────────
        PlayerSettings.runInBackground = true;

        // ── 13. Save ──────────────────────────────────────────────────────────
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo();

        Selection.activeGameObject = root;
        SceneView.FrameLastActiveSceneView();

        EditorUtility.DisplayDialog("JETANK Built!",
            "Your JETANK is in the scene.\n\n" +
            "Press Play, then run:\n" +
            "  python test_all.py 1\n\n" +
            "The arm joints are:\n" +
            "  Servo 1 → base yaw\n" +
            "  Servo 2 → shoulder\n" +
            "  Servo 3 → elbow\n" +
            "  Servo 4 → gripper\n" +
            "  Servo 5 → wrist",
            "OK");
    }

    // ── Arm builder ───────────────────────────────────────────────────────────

    static void BuildArm(GameObject armRoot)
    {
        // Base plate (visual only, no rotation)
        Box("Arm_BasePlate", armRoot, new Vector3(0f, 0.04f, 0f),
            new Vector3(0.30f, 0.08f, 0.30f), COL_JOINT);

        // j1_turntable — base yaw (rotates around Y)
        GameObject j1 = Joint("j1_turntable", armRoot, new Vector3(0f, 0.10f, 0f));
        // Turntable disc visual
        GameObject turntable = Cyl("j1_visual", j1, new Vector3(0f, 0.05f, 0f),
                                    new Vector3(0.20f, 0.10f, 0.20f), COL_JOINT);

        // j2_shoulder — shoulder pitch (rotates around X)
        GameObject j2 = Joint("j2_shoulder", j1, new Vector3(0f, 0.15f, 0f));
        // Upper arm link visual — extends from j2 upward
        Box("UpperArm_visual", j2,
            new Vector3(0f, LINK_A * 0.5f, 0f),
            new Vector3(0.10f, LINK_A, 0.10f), COL_ARM);

        // j3_upper_arm — elbow pitch (rotates around X)
        GameObject j3 = Joint("j3_upper_arm", j2, new Vector3(0f, LINK_A, 0f));
        // Elbow joint cap
        Cyl("Elbow_visual", j3, Vector3.zero,
            new Vector3(0.14f, 0.10f, 0.14f), COL_JOINT);
        // Forearm link visual
        Box("Forearm_visual", j3,
            new Vector3(0f, LINK_B * 0.5f, 0f),
            new Vector3(0.085f, LINK_B, 0.085f), COL_ARM);

        // j4_forearm — intermediate (no servo in real JETANK; still needed by controller)
        GameObject j4 = Joint("j4_forearm", j3, new Vector3(0f, LINK_B, 0f));

        // j5_wrist — wrist pitch (rotates around X)  [servo 5]
        GameObject j5 = Joint("j5_wrist", j4, Vector3.zero);
        Cyl("Wrist_visual", j5, Vector3.zero,
            new Vector3(0.12f, 0.08f, 0.12f), COL_JOINT);

        // j6_end_effector — tool mount
        GameObject j6 = Joint("j6_end_effector", j5, new Vector3(0f, 0.12f, 0f));
        Box("Palm_visual", j6,
            new Vector3(0f, 0.06f, 0f),
            new Vector3(0.10f, 0.10f, 0.08f), COL_ARM);

        // Gripper fingers  [servo 4 → gripper_left and gripper_right]
        // RoboticArmController separates them on the X axis
        GameObject gl = Box("gripper_left",  j6,
            new Vector3(-0.055f, 0.17f, 0f),
            new Vector3(0.04f, 0.14f, 0.04f), COL_GRIP);
        GameObject gr = Box("gripper_right", j6,
            new Vector3( 0.055f, 0.17f, 0f),
            new Vector3(0.04f, 0.14f, 0.04f), COL_GRIP);

        // Fingertip pads
        Box("Tip_L", j6, new Vector3(-0.055f, 0.245f, 0.012f),
            new Vector3(0.038f, 0.02f, 0.02f), new Color(0.1f, 0.1f, 0.1f));
        Box("Tip_R", j6, new Vector3( 0.055f, 0.245f, 0.012f),
            new Vector3(0.038f, 0.02f, 0.02f), new Color(0.1f, 0.1f, 0.1f));
    }

    // ── Primitive helpers ─────────────────────────────────────────────────────

    static GameObject Joint(string name, GameObject parent, Vector3 localPos)
    {
        var go = new GameObject(name);
        go.transform.SetParent(parent.transform);
        go.transform.localPosition = localPos;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale    = Vector3.one;
        return go;
    }

    static GameObject Box(string name, GameObject parent,
                          Vector3 localPos, Vector3 scale, Color col)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name;
        go.transform.SetParent(parent.transform);
        go.transform.localPosition = localPos;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale    = scale;
        ApplyColor(go, col);
        // Remove collider from arm/visual parts (only chassis needs collider)
        if (!name.StartsWith("Chassis") && !name.StartsWith("Ground"))
            Object.DestroyImmediate(go.GetComponent<Collider>());
        return go;
    }

    static GameObject Cyl(string name, GameObject parent,
                           Vector3 localPos, Vector3 scale, Color col)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        go.name = name;
        go.transform.SetParent(parent.transform);
        go.transform.localPosition = localPos;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale    = scale;
        ApplyColor(go, col);
        Object.DestroyImmediate(go.GetComponent<Collider>());
        return go;
    }

    static void Wheel(string name, GameObject parent, Vector3 localPos,
                      float radius = 0.18f)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        go.name = name;
        go.transform.SetParent(parent.transform);
        go.transform.localPosition = localPos;
        // Rotate so the cylinder axis runs along X (side-to-side)
        go.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
        go.transform.localScale    = new Vector3(radius * 2f, TW * 0.38f, radius * 2f);
        ApplyColor(go, COL_WHEEL);
        Object.DestroyImmediate(go.GetComponent<Collider>());
    }

    static void ApplyColor(GameObject go, Color col)
    {
        var rend = go.GetComponent<Renderer>();
        if (rend == null) return;
        var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        mat.SetColor("_BaseColor", col);
        mat.color = col;
        rend.material = mat;
    }

    static void RemoveNamed(params string[] names)
    {
        foreach (string n in names)
        {
            GameObject found = GameObject.Find(n);
            if (found != null)
            {
                Object.DestroyImmediate(found);
                Debug.Log($"[BuildJETANK] Removed: {n}");
            }
        }
    }
}
