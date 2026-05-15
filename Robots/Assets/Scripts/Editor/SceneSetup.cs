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

            // ── 3. SimCamera on Main Camera ───────────────────────────────────
            Camera mainCam = Camera.main;
            if (mainCam != null)
            {
                if (mainCam.GetComponent<SimCamera>() == null)
                {
                    mainCam.gameObject.AddComponent<SimCamera>();
                    Debug.Log("[SceneSetup] Added SimCamera to Main Camera");
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

            // ── 7. Run In Background ──────────────────────────────────────────
            PlayerSettings.runInBackground = true;

            // ── 8. Save scene ─────────────────────────────────────────────────
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
                "  Run In Background ON\n\n" +
                "Press Play, then run:\n" +
                "  python test_all.py 1",
                "OK"
            );
        }
    }
}
