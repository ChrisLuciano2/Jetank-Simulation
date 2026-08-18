using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using RobotSimulator;
using RobotSimulator.Communication;

namespace RobotSimulator.Editor
{
    public static class SceneSetup
    {
        [MenuItem("Tools/Setup Robot Simulator Scene")]
        public static void SetupScene()
        {
            bool changed = false;

            // ── 1. TcpServer (port 5555) ──────────────────────────────────────
            if (Object.FindFirstObjectByType<TcpServer>() == null)
            {
                new GameObject("TcpServer").AddComponent<TcpServer>();
                Debug.Log("[SceneSetup] Created TcpServer");
                changed = true;
            }

            // ── 2. SimQueryServer (port 5556) ─────────────────────────────────
            if (Object.FindFirstObjectByType<SimQueryServer>() == null)
            {
                new GameObject("SimQueryServer").AddComponent<SimQueryServer>();
                Debug.Log("[SceneSetup] Created SimQueryServer");
                changed = true;
            }

            // ── 3. Main Camera stays the HUMAN's overview ─────────────────────
            // SimCamera used to live here, which meant Python's "robot camera"
            // was a world-fixed spectator view that never moved with the robot.
            // Every camera-driven behaviour (jetbot_nav.visual_scan,
            // perception, target_seek) computes bearings relative to the
            // ROBOT, so the feed has to come from a robot-mounted camera —
            // see step 7. Strip any SimCamera left on the overview camera.
            Camera mainCam = Camera.main;
            if (mainCam != null)
            {
                var stale = mainCam.GetComponent<SimCamera>();
                if (stale != null)
                {
                    Object.DestroyImmediate(stale);
                    Debug.Log("[SceneSetup] Removed SimCamera from the overview " +
                              "Main Camera — it now lives on the robot (step 7)");
                    changed = true;
                }
                mainCam.transform.position = new Vector3(0f, 3.5f, -5f);
                mainCam.transform.rotation = Quaternion.Euler(20f, 0f, 0f);
            }

            // ── 4. Ground plane ───────────────────────────────────────────────
            if (GameObject.Find("Ground") == null)
            {
                GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
                ground.name = "Ground";
                ground.transform.position = Vector3.zero;
                ground.transform.localScale = new Vector3(5f, 1f, 5f);
                var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                mat.color = new Color(0.3f, 0.5f, 0.3f);
                ground.GetComponent<Renderer>().material = mat;
                Debug.Log("[SceneSetup] Created Ground plane");
                changed = true;
            }

            // ── 5. Truck ──────────────────────────────────────────────────────
            GameObject truckObj = null;
            TruckController existingTruck = Object.FindFirstObjectByType<TruckController>();

            if (existingTruck == null)
            {
                // Try the real truck prefab first
                string[] truckGuids = AssetDatabase.FindAssets("Truck t:Prefab", new[] { "Assets/MiniCargoTruck" });
                if (truckGuids.Length > 0)
                {
                    string path = AssetDatabase.GUIDToAssetPath(truckGuids[0]);
                    GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
                    if (prefab != null)
                    {
                        truckObj = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                        truckObj.name = "Truck_01";
                        Debug.Log($"[SceneSetup] Instantiated truck prefab: {path}");
                    }
                }

                // Fallback: blue box
                if (truckObj == null)
                {
                    truckObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    truckObj.name = "Truck_01";
                    truckObj.transform.localScale = new Vector3(1.2f, 0.6f, 2f);
                    var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                    mat.color = new Color(0.1f, 0.3f, 0.8f);
                    truckObj.GetComponent<Renderer>().material = mat;
                    Debug.Log("[SceneSetup] Truck prefab not found — created placeholder cube");
                }

                truckObj.transform.position = new Vector3(0f, 0f, 0f);

                if (truckObj.GetComponent<TruckController>() == null)
                    truckObj.AddComponent<TruckController>();

                changed = true;
            }
            else
            {
                truckObj = existingTruck.gameObject;
                Debug.Log("[SceneSetup] Truck already in scene");
            }

            // ── 6. Robotic Arm (mounted on truck) ─────────────────────────────
            // Only add if there isn't one already
            RoboticArmController existingArm = Object.FindFirstObjectByType<RoboticArmController>();
            if (existingArm == null && truckObj != null)
            {
                GameObject armObj = null;

                // Try loading the GLB model
                string[] armGuids = AssetDatabase.FindAssets("robotic_arm_6dof");
                if (armGuids.Length > 0)
                {
                    string armPath = AssetDatabase.GUIDToAssetPath(armGuids[0]);
                    GameObject armPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(armPath);
                    if (armPrefab != null)
                    {
                        armObj = (GameObject)PrefabUtility.InstantiatePrefab(armPrefab);
                        armObj.name = "RoboticArm";
                        Debug.Log($"[SceneSetup] Instantiated arm model: {armPath}");
                    }
                }

                // Fallback: thin yellow box as placeholder
                if (armObj == null)
                {
                    armObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    armObj.name = "RoboticArm";
                    armObj.transform.localScale = new Vector3(0.15f, 0.4f, 0.15f);
                    var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                    mat.color = Color.yellow;
                    armObj.GetComponent<Renderer>().material = mat;
                    Debug.Log("[SceneSetup] Arm model not found — created placeholder");
                }

                // Mount on top of truck center-front
                armObj.transform.SetParent(truckObj.transform);
                armObj.transform.localPosition = new Vector3(0f, 0.5f, 0.3f);
                armObj.transform.localRotation = Quaternion.identity;
                armObj.transform.localScale    = Vector3.one;

                // Add controllers
                RoboticArmController armCtrl = armObj.GetComponent<RoboticArmController>();
                if (armCtrl == null)
                    armCtrl = armObj.AddComponent<RoboticArmController>();

                armCtrl.AutoAssignJoints();

                if (armObj.GetComponent<RoboticArmNetworkController>() == null)
                    armObj.AddComponent<RoboticArmNetworkController>();

                changed = true;
            }
            else if (existingArm != null)
            {
                Debug.Log("[SceneSetup] Robotic arm already in scene");
            }

            // ── 7. Robot-mounted camera (the one Python actually sees) ────────
            if (truckObj != null && Object.FindFirstObjectByType<SimCamera>() == null)
            {
                CreateRobotCamera(truckObj);
                changed = true;
            }

            // ── 8. ProximitySensor on the truck ───────────────────────────────
            // Must sit on the SAME GameObject as TruckController: the rays are
            // cast along that transform's forward, so putting it on a parent or
            // a child with its own rotation silently skews every bearing the
            // Python side computes.
            if (truckObj != null && truckObj.GetComponent<ProximitySensor>() == null)
            {
                truckObj.AddComponent<ProximitySensor>();
                Debug.Log("[SceneSetup] Added ProximitySensor to " + truckObj.name);
                changed = true;
            }

            // ── 9. Obstacle course ────────────────────────────────────────────
            // Colliders are what the sensor rays actually hit, so these are
            // primitives (which come with one) rather than bare renderers.
            if (GameObject.Find(ObstacleRoot) == null)
            {
                CreateObstacleCourse();
                changed = true;
            }
            else
            {
                Debug.Log("[SceneSetup] Obstacle course already in scene");
            }

            // ── 10. Run In Background ─────────────────────────────────────────
            PlayerSettings.runInBackground = true;

            // ── 11. Save scene ────────────────────────────────────────────────
            if (changed)
            {
                EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
                EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo();
            }

            EditorUtility.DisplayDialog(
                "Scene Setup Complete",
                "Everything added:\n\n" +
                "  TcpServer         (port 5555)\n" +
                "  SimQueryServer    (port 5556)\n" +
                "  SimCamera         (on Main Camera)\n" +
                "  Truck_01          (TruckController)\n" +
                "  RoboticArm        (mounted on truck)\n" +
                "  RobotCamera       (SimCamera — Python's eye, on the robot)\n" +
                "  ProximitySensor   (debug ground truth only)\n" +
                "  Obstacles         (red DetectableObject props)\n" +
                "  Run In Background ON\n\n" +
                "Press Play, then run:\n" +
                "  py -3.8 test_all.py 1\n" +
                "  py -3.8 test_gap_navigation.py 30",
                "OK"
            );
        }

        // ── Robot camera ──────────────────────────────────────────────────────

        // These four numbers ARE the calibration. jetbot_nav.visual_scan turns
        // pixels into distances using the camera's height, tilt and FOV, so
        // they must match CameraGeometry on the Python side exactly —
        // visual_scan.SIM_JETANK mirrors them. Change one, change both, or
        // every reported distance is wrong by a constant factor while still
        // looking entirely plausible.
        public const float CamHeight = 0.80f;   // above the truck's origin
        public const float CamForward = 0.80f;  // toward the front bumper
        public const float CamTiltDeg = 20f;    // downward pitch
        public const float CamFovDeg = 48.8f;   // VERTICAL fov of an IMX219
                                                // (62.2 horizontal at 4:3)

        private static void CreateRobotCamera(GameObject truckObj)
        {
            var camObj = new GameObject("RobotCamera");
            camObj.transform.SetParent(truckObj.transform);
            camObj.transform.localPosition = new Vector3(0f, CamHeight, CamForward);
            camObj.transform.localRotation = Quaternion.Euler(CamTiltDeg, 0f, 0f);

            var cam = camObj.AddComponent<Camera>();
            cam.fieldOfView = CamFovDeg;

            // Disabled so it never draws to the game view — SimCamera calls
            // cam.Render() explicitly into its own RenderTexture, which works
            // fine on a disabled Camera and costs nothing the rest of the time.
            cam.enabled = false;

            camObj.AddComponent<SimCamera>();

            Debug.Log($"[SceneSetup] Created RobotCamera on {truckObj.name} " +
                      $"(height {CamHeight}, forward {CamForward}, tilt {CamTiltDeg} deg, " +
                      $"fov {CamFovDeg} deg) — keep jetbot_nav.visual_scan.SIM_JETANK in sync");
        }

        // ── Obstacle course ───────────────────────────────────────────────────

        private const string ObstacleRoot = "Obstacles";
        private const string DetectableTag = "DetectableObject";

        // Laid out ahead of the truck's spawn at the origin, deliberately
        // mixed: a thin pole (invisible to widely-spaced rays), a pair
        // forming a threadable doorway, and offset blocks to steer around.
        private static readonly (string name, Vector3 pos, Vector3 scale)[] Obstacles =
        {
            ("Pole",       new Vector3( 0.2f, 0.5f,  6f),  new Vector3(0.3f, 1f, 0.3f)),
            ("Door_Left",  new Vector3(-3f,   0.5f, 12f),  new Vector3(2f,   1f, 1f)),
            ("Door_Right", new Vector3( 3f,   0.5f, 12f),  new Vector3(2f,   1f, 1f)),
            ("Block_A",    new Vector3(-2f,   0.5f, 18f),  new Vector3(1.5f, 1f, 1.5f)),
            ("Block_B",    new Vector3( 2.5f, 0.5f, 22f),  new Vector3(1.5f, 1f, 1.5f)),
        };

        private static void CreateObstacleCourse()
        {
            var root = new GameObject(ObstacleRoot);

            // jetbot_nav.perception picks targets out by HSV hue, so the
            // colour here is load-bearing, not decoration — it must land
            // inside perception.COLOR_RANGES["red"].
            var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            mat.color = new Color(0.85f, 0.08f, 0.08f);

            bool tagExists = System.Array.IndexOf(
                UnityEditorInternal.InternalEditorUtility.tags, DetectableTag) >= 0;

            foreach (var (name, pos, scale) in Obstacles)
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.name = name;
                go.transform.SetParent(root.transform);
                go.transform.position   = pos;
                go.transform.localScale = scale;
                go.GetComponent<Renderer>().sharedMaterial = mat;

                if (tagExists) go.tag = DetectableTag;
            }

            if (!tagExists)
                Debug.LogWarning($"[SceneSetup] Tag '{DetectableTag}' does not exist, so " +
                    "the obstacles were left untagged and SimCamera's detect_objects will " +
                    "report nothing. Add it under Edit > Project Settings > Tags and Layers, " +
                    "then re-run this tool. (jetbot_nav.perception detects them by colour " +
                    "regardless, so gap-following and colour-seeking still work without it.)");

            Debug.Log($"[SceneSetup] Created obstacle course ({Obstacles.Length} props)");
        }
    }
}
