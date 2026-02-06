using UnityEngine;
using UnityEditor;
using RobotSimulator;
using RobotSimulator.Communication;

namespace RobotSimulator.Editor
{
    /// <summary>
    /// Editor utility to set up the scene with required components.
    /// </summary>
    public class SceneSetup : EditorWindow
    {
        [MenuItem("Tools/Setup Robot Simulator Scene")]
        public static void SetupScene()
        {
            // 1. Find or create TcpServer
            TcpServer tcpServer = FindObjectOfType<TcpServer>();
            if (tcpServer == null)
            {
                GameObject serverObj = new GameObject("TcpServer");
                tcpServer = serverObj.AddComponent<TcpServer>();
                Debug.Log("[SceneSetup] Created TcpServer GameObject");
            }
            else
            {
                Debug.Log("[SceneSetup] TcpServer already exists");
            }

            // 2. Find or create Truck
            TruckController truck = FindObjectOfType<TruckController>();
            if (truck == null)
            {
                GameObject truckObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
                truckObj.name = "Truck_01";
                truckObj.transform.position = new Vector3(0, 0.5f, 0);
                truckObj.transform.localScale = new Vector3(2f, 1f, 3f);

                // Add TruckController
                truck = truckObj.AddComponent<TruckController>();

                // Set the material color to make it visible
                Renderer renderer = truckObj.GetComponent<Renderer>();
                if (renderer != null)
                {
                    renderer.material.color = Color.blue;
                }

                Debug.Log("[SceneSetup] Created Truck_01 GameObject with TruckController");
            }
            else
            {
                Debug.Log("[SceneSetup] Truck with TruckController already exists");
            }

            // 3. Position camera to view the truck
            Camera mainCamera = Camera.main;
            if (mainCamera != null)
            {
                mainCamera.transform.position = new Vector3(0, 5, -10);
                mainCamera.transform.rotation = Quaternion.Euler(20, 0, 0);
                Debug.Log("[SceneSetup] Positioned Main Camera");
            }

            // Mark scene as dirty so it prompts to save
            UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(
                UnityEditor.SceneManagement.EditorSceneManager.GetActiveScene()
            );

            Debug.Log("[SceneSetup] ✓ Scene setup complete! Press Play and run unity_client.py");
            EditorUtility.DisplayDialog(
                "Setup Complete",
                "Scene is now ready!\n\n1. Save the scene (Ctrl+S / Cmd+S)\n2. Press Play\n3. Run unity_client.py",
                "OK"
            );
        }
    }
}
