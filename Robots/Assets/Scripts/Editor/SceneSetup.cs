using System.IO;
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
            // Running this during Play is doubly futile, and it is an easy
            // trap because the Python harness needs the sim PLAYING to
            // connect — so the natural workflow is to leave it running and
            // invoke the tool alongside it. That silently fails twice over:
            //
            //   1. Unity does not recompile scripts while playing, so the
            //      menu item that runs is whatever was compiled BEFORE Play
            //      started, no matter what the file on disk says.
            //   2. Scene changes made during Play are discarded when Play
            //      stops, so even a correctly compiled run throws away its
            //      own work.
            //
            // Both are invisible: the tool appears to succeed and the scene
            // appears unchanged, which is indistinguishable from a bug in
            // the tool. Refuse instead.
            if (EditorApplication.isPlayingOrWillChangePlaymode)
            {
                EditorUtility.DisplayDialog(
                    "Stop Play Mode First",
                    "Unity will not recompile scripts while playing, and any "
                    + "scene changes made now are discarded when Play stops.\n\n"
                    + "  1. Press Stop\n"
                    + "  2. Wait for the recompile to finish\n"
                    + "  3. Run this tool again\n"
                    + "  4. Save the scene, then press Play\n\n"
                    + "The Console should show a \"running rev\" line when the "
                    + "tool actually executes. If it does not, Unity is still "
                    + "on a stale compile.",
                    "OK");
                Debug.LogWarning("[SceneSetup] Aborted: cannot set up the scene "
                    + "during Play mode. Stop, let scripts recompile, re-run.");
                return;
            }

            bool changed = false;

            // Version stamp. If this line is missing from the Console after
            // running the menu item, Unity is executing a STALE COMPILE of
            // this file and none of the steps below are the ones you are
            // reading. Bump it whenever this file changes meaningfully.
            Debug.Log("[SceneSetup] running rev 5 (matte surfaces)");

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
            // The SimCamera move itself happens in step 7, which owns all
            // camera placement — doing half of it here made step 7 depend on
            // Camera.main having been found, and a scene where that lookup
            // fails would silently keep the old world-fixed feed.
            Camera mainCam = Camera.main;
            if (mainCam != null)
            {
                mainCam.transform.position = new Vector3(0f, 3.5f, -5f);
                mainCam.transform.rotation = Quaternion.Euler(20f, 0f, 0f);
            }
            else
            {
                Debug.LogWarning("[SceneSetup] No camera tagged MainCamera — the "
                    + "human overview view was left alone. This does not affect "
                    + "the robot camera (step 7).");
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

            // The floor must be MATTE, and this runs whether or not the ground
            // was just created — a scene built by an earlier revision still has
            // the glossy one, and the symptom is subtle enough that nobody
            // would think to rebuild the scene over it.
            //
            // URP/Lit defaults to smoothness 0.5, so a large flat floor
            // reflects progressively more sky as the viewing angle gets
            // shallower. Measured down a single frame: the floor's hue ran 48
            // near the camera to 68 at the horizon while getting BRIGHTER
            // (val 174 -> 180). floor_mask's hue_tol is 12, so everything past
            // ~row 210 stopped counting as floor and the column walk reported
            // a phantom obstacle at 2.4 units on completely open ground —
            // capping the camera's useful range at ~2.5 units no matter what
            // max_range said, and leaving the robot backing away from nothing
            // for 69% of a drive.
            //
            // The brightening is why the shadow allowance could not save it:
            // that latitude is granted only to pixels DARKER than the floor,
            // and this drift goes the other way.
            var groundGo = GameObject.Find("Ground");
            if (groundGo != null && MakeMatte(groundGo.GetComponent<Renderer>()))
            {
                Debug.Log("[SceneSetup] Ground material set to matte "
                          + "(smoothness 0, no specular or environment reflections)");
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
            // Python's whole camera pipeline computes bearings and distances
            // relative to the ROBOT, so the feed has to come from a camera
            // that moves with it. Historically SimCamera sat on the
            // world-fixed overview camera, which produced confident nonsense
            // rather than an error — visual_scan reported the robot's own
            // chassis as an obstacle a metre ahead.
            //
            // The condition is "is there a SimCamera UNDER THE TRUCK", not
            // "is there a SimCamera anywhere": the latter is satisfied by the
            // very misconfiguration this is meant to repair, so the tool
            // would report success having changed nothing. Inactive objects
            // are included, since a disabled leftover still blocks the
            // singleton.
            if (truckObj != null)
            {
                var cams = Object.FindObjectsByType<SimCamera>(
                    FindObjectsInactive.Include, FindObjectsSortMode.None);

                SimCamera onRobot = null;
                foreach (var c in cams)
                    if (c.transform.IsChildOf(truckObj.transform)) { onRobot = c; break; }

                if (onRobot != null)
                {
                    Debug.Log($"[SceneSetup] Robot camera already present on "
                              + $"'{onRobot.gameObject.name}' — left as is");
                }
                else
                {
                    foreach (var c in cams)
                    {
                        Debug.Log($"[SceneSetup] Removing SimCamera from "
                                  + $"'{c.gameObject.name}' — it is not on the robot");
                        Object.DestroyImmediate(c);
                        changed = true;
                    }
                    CreateRobotCamera(truckObj);
                    changed = true;
                }
            }
            else
            {
                Debug.LogError("[SceneSetup] No TruckController in the scene, so "
                    + "there is nothing to mount the camera on. Python's camera "
                    + "pipeline will not work until this is fixed.");
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

            // ── 10. Arena walls ───────────────────────────────────────────────
            if (GameObject.Find(WallRoot) == null)
            {
                CreateWalls();
                changed = true;
            }
            else
            {
                // Walls from an earlier revision still carry the default
                // glossy material, and the step above will not rebuild them.
                var wallRoot = GameObject.Find(WallRoot);
                int retouched = 0;
                foreach (var r in wallRoot.GetComponentsInChildren<Renderer>())
                    if (MakeMatte(r)) retouched++;

                Debug.Log(retouched > 0
                    ? $"[SceneSetup] Arena walls already in scene; made {retouched} "
                      + "wall material(s) matte"
                    : "[SceneSetup] Arena walls already in scene");
                if (retouched > 0) changed = true;
            }

            // ── 11. Run In Background ─────────────────────────────────────────
            PlayerSettings.runInBackground = true;

            // ── 12. Save scene ────────────────────────────────────────────────
            if (changed)
            {
                EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
                EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo();
            }

            // Report what the scene ACTUALLY contains now, rather than a fixed
            // list of what the tool intended to do. A dialog that claims
            // success regardless is worse than no dialog: the previous version
            // said "RobotCamera — on the robot" while SimCamera was still
            // sitting on the world-fixed overview camera.
            var sim = Object.FindFirstObjectByType<SimCamera>();
            string camWhere;
            if (sim == null)
                camWhere = "MISSING — Python will get no frames";
            else if (truckObj != null && sim.transform.IsChildOf(truckObj.transform))
                camWhere = $"on '{sim.gameObject.name}' (correct: moves with the robot)";
            else
                camWhere = $"on '{sim.gameObject.name}' — NOT on the robot, "
                           + "distances and headings will be wrong";

            EditorUtility.DisplayDialog(
                "Scene Setup Complete",
                "Scene now contains:\n\n" +
                $"  TcpServer        {(Object.FindFirstObjectByType<TcpServer>() != null ? "yes" : "MISSING")}\n" +
                $"  SimQueryServer   {(Object.FindFirstObjectByType<SimQueryServer>() != null ? "yes" : "MISSING")}\n" +
                $"  Robot            {(truckObj != null ? truckObj.name : "MISSING")}\n" +
                $"  ProximitySensor  {(Object.FindFirstObjectByType<ProximitySensor>() != null ? "yes (dev ground truth)" : "MISSING")}\n" +
                $"  Obstacles        {(GameObject.Find(ObstacleRoot) != null ? "yes" : "MISSING")}\n" +
                $"  Arena walls      {(GameObject.Find(WallRoot) != null ? "yes (textured, for heading)" : "MISSING — heading will fail in featureless directions")}\n" +
                $"  SimCamera        {camWhere}\n\n" +
                "Press Play, then verify the camera pipeline with:\n" +
                "  py -3.8 validate_camera.py --save",
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

        // ── Arena walls ───────────────────────────────────────────────────────
        //
        // WHY THE SCENE NEEDS THESE AT ALL
        //
        // Camera-only heading works by correlating what the camera sees now
        // against what it saw before. With an open ground plane and Unity's
        // procedural sky, roughly half of all headings had NOTHING in frame to
        // correlate: measured across three positions, facing yaw 0/45/135 the
        // estimator was accurate to 0.6-1.8 deg, while facing 90/180/270 a real
        // 5 deg rotation measured as 0.0 deg, because the view correlated with
        // itself at 0.998. There was simply nothing to see.
        //
        // No gate can catch that. Correlation score, peak prominence,
        // peak-to-floor falloff and raw band structure were each measured
        // across the accurate and inaccurate populations and every one of them
        // OVERLAPS — a featureless view is indistinguishable from a robot that
        // did not turn. The fix has to be in the environment, and a real room
        // has features in every direction, so this is also the more faithful
        // scene.
        //
        // THREE CONSTRAINTS, ALL LOAD-BEARING
        //
        //   1. Not red. jetbot_nav.perception picks seek targets out by red
        //      hue, so a red wall would be a 48-unit-wide target.
        //   2. Not "dark and near the floor's hue". floor_mask deliberately
        //      forgives pixels that are BOTH darker than the floor and within
        //      shadow_hue_tol of it, because that is what a shadow looks like.
        //      A wall matching that description would be walked straight
        //      through as open ground. The blues here sit ~50 (OpenCV) away
        //      from the floor's hue, which clears that rule at any brightness.
        //   3. Horizontal variation, non-repeating. A uniformly coloured wall
        //      collapses to a flat column profile and fixes nothing — it is
        //      exactly as unhelpful as the empty sky it replaces. Evenly
        //      repeating stripes are worse than useless: periodic scenery
        //      produces a confident WRONG match, which is the failure the
        //      prominence gate exists to catch. Hence bands of randomised
        //      width and shade, stretched once across each wall rather than
        //      tiled.
        private const string WallRoot   = "ArenaWalls";
        private const string GeneratedDir = "Assets/Generated";
        private const float  WallHalfExtent = 24f;   // just inside the ground plane
        private const float  WallHeight     = 3f;
        private const float  WallThickness  = 0.5f;

        /// <summary>
        /// Strip a renderer's material of view-angle-dependent shading.
        /// Returns true if anything actually changed, so callers can decide
        /// whether the scene needs saving.
        ///
        /// Anything the camera pipeline segments by colour has to look the
        /// same from every angle, because floor_mask and perception both
        /// compare hue against a reference sampled somewhere else in the
        /// frame. A specular or sky-reflecting surface breaks that assumption
        /// quietly: the colour is still "right", just not the same right at
        /// both ends of the frame.
        /// </summary>
        private static bool MakeMatte(Renderer renderer)
        {
            if (renderer == null) return false;
            var mat = renderer.sharedMaterial;
            if (mat == null) return false;

            bool changed = false;

            if (mat.HasProperty("_Smoothness") && mat.GetFloat("_Smoothness") != 0f)
            {
                mat.SetFloat("_Smoothness", 0f);
                changed = true;
            }
            if (mat.HasProperty("_SpecularHighlights")
                && mat.GetFloat("_SpecularHighlights") != 0f)
            {
                mat.SetFloat("_SpecularHighlights", 0f);
                mat.EnableKeyword("_SPECULARHIGHLIGHTS_OFF");
                changed = true;
            }
            // The environment reflection is the bigger contributor here: it is
            // what puts sky colour onto a floor seen at a grazing angle.
            if (mat.HasProperty("_EnvironmentReflections")
                && mat.GetFloat("_EnvironmentReflections") != 0f)
            {
                mat.SetFloat("_EnvironmentReflections", 0f);
                mat.EnableKeyword("_ENVIRONMENTREFLECTIONS_OFF");
                changed = true;
            }

            if (changed) EditorUtility.SetDirty(mat);
            return changed;
        }

        private static void CreateWalls()
        {
            if (!AssetDatabase.IsValidFolder(GeneratedDir))
                AssetDatabase.CreateFolder("Assets", "Generated");

            var root = new GameObject(WallRoot);

            // (name, position, scale, texture seed). Distinct seeds matter:
            // four identical walls would make the four compass directions
            // indistinguishable, so a heading estimate could lock onto the
            // wrong one and be confidently 90 deg out.
            float span = WallHalfExtent * 2f;
            var walls = new (string name, Vector3 pos, Vector3 scale, int seed)[]
            {
                ("Wall_North", new Vector3(0f, WallHeight / 2f,  WallHalfExtent),
                               new Vector3(span, WallHeight, WallThickness), 11),
                ("Wall_South", new Vector3(0f, WallHeight / 2f, -WallHalfExtent),
                               new Vector3(span, WallHeight, WallThickness), 23),
                ("Wall_East",  new Vector3( WallHalfExtent, WallHeight / 2f, 0f),
                               new Vector3(WallThickness, WallHeight, span), 37),
                ("Wall_West",  new Vector3(-WallHalfExtent, WallHeight / 2f, 0f),
                               new Vector3(WallThickness, WallHeight, span), 53),
            };

            foreach (var (name, pos, scale, seed) in walls)
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.name = name;
                go.transform.SetParent(root.transform);
                go.transform.position   = pos;
                go.transform.localScale = scale;
                go.GetComponent<Renderer>().sharedMaterial = BuildWallMaterial(name, seed);
            }

            Debug.Log($"[SceneSetup] Created {walls.Length} textured arena walls " +
                      $"({span}x{span} units). Textures and materials written to " +
                      GeneratedDir);
        }

        /// <summary>
        /// Material for one wall, backed by a generated banded texture saved as
        /// a real asset.
        ///
        /// Saving to disk is not optional: a Texture2D built in memory by an
        /// editor script does not serialise with the scene, so the material
        /// would reference a dead object the next time the scene was opened and
        /// the walls would come back blank — reintroducing the exact problem
        /// they exist to solve, silently.
        /// </summary>
        private static Material BuildWallMaterial(string name, int seed)
        {
            string texPath = $"{GeneratedDir}/{name}_Texture.png";
            string matPath = $"{GeneratedDir}/{name}_Material.mat";

            File.WriteAllBytes(texPath, BuildBandedTexture(seed).EncodeToPNG());
            AssetDatabase.ImportAsset(texPath, ImportAssetOptions.ForceUpdate);
            var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(texPath);

            var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            mat.mainTexture = tex;
            // Stretch once across the wall. Any tiling here would make the
            // pattern periodic, which is the aliasing case that produces
            // confident wrong matches.
            mat.mainTextureScale = Vector2.one;

            // Matte for the same reason the floor is, and it matters more
            // here: heading is read by correlating the wall against a view of
            // the same wall from somewhere else. A specular sheen that moves
            // as the robot moves is a feature that slides independently of the
            // texture underneath it, which is noise injected into precisely
            // the signal these walls were added to provide.
            mat.SetFloat("_Smoothness", 0f);
            mat.SetFloat("_SpecularHighlights", 0f);
            mat.EnableKeyword("_SPECULARHIGHLIGHTS_OFF");
            mat.SetFloat("_EnvironmentReflections", 0f);
            mat.EnableKeyword("_ENVIRONMENTREFLECTIONS_OFF");

            AssetDatabase.CreateAsset(mat, matPath);
            return mat;
        }

        private static Texture2D BuildBandedTexture(int seed)
        {
            const int width  = 1024;
            const int height = 128;

            // Bright blues and violets: far in hue from both the red seek
            // targets and the green floor, so neither perception nor
            // floor_mask can mistake a wall for something it cares about.
            var palette = new[]
            {
                new Color(0.55f, 0.62f, 0.80f),
                new Color(0.70f, 0.74f, 0.86f),
                new Color(0.45f, 0.52f, 0.72f),
                new Color(0.62f, 0.58f, 0.78f),
                new Color(0.78f, 0.80f, 0.88f),
                new Color(0.38f, 0.45f, 0.66f),
            };

            var rng = new System.Random(seed);
            var tex = new Texture2D(width, height, TextureFormat.RGB24, false);
            var pixels = new Color[width * height];

            int x = 0;
            while (x < width)
            {
                int bandWidth = rng.Next(6, 49);          // varied, so the
                bandWidth = Mathf.Min(bandWidth, width - x);   // pattern never
                Color band = palette[rng.Next(palette.Length)]; // repeats
                float shade = 0.82f + (float)rng.NextDouble() * 0.36f;

                for (int bx = 0; bx < bandWidth; bx++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        // Slight vertical falloff so the wall reads as a
                        // surface rather than a flat colour chip. Only the
                        // horizontal variation matters to the estimator, which
                        // averages down the rows.
                        float v = shade * (0.88f + 0.12f * y / (height - 1f));
                        pixels[y * width + x + bx] = new Color(
                            Mathf.Clamp01(band.r * v),
                            Mathf.Clamp01(band.g * v),
                            Mathf.Clamp01(band.b * v));
                    }
                }
                x += bandWidth;
            }

            tex.SetPixels(pixels);
            tex.Apply();
            return tex;
        }

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
