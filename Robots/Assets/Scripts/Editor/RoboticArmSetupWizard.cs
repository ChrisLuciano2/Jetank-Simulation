using UnityEngine;
using UnityEditor;

/// <summary>
/// Editor wizard to create a proper joint hierarchy for the robotic arm.
/// Use this when importing from .obj or other flat formats.
/// </summary>
public class RoboticArmSetupWizard : EditorWindow
{
    private GameObject armRoot;
    private GameObject visualMesh;

    [MenuItem("Tools/Robotic Arm/Setup Joint Hierarchy")]
    public static void ShowWindow()
    {
        GetWindow<RoboticArmSetupWizard>("Arm Setup Wizard");
    }

    void OnGUI()
    {
        GUILayout.Label("Robotic Arm Setup Wizard", EditorStyles.boldLabel);
        GUILayout.Space(10);

        EditorGUILayout.HelpBox(
            "This wizard creates a proper joint hierarchy for the robotic arm.\n\n" +
            "1. Select the root GameObject (e.g., 'robotic_arm_6dof')\n" +
            "2. Optionally select the visual mesh child (e.g., 'default')\n" +
            "3. Click 'Create Joint Hierarchy'",
            MessageType.Info
        );

        GUILayout.Space(10);

        armRoot = (GameObject)EditorGUILayout.ObjectField(
            "Arm Root GameObject",
            armRoot,
            typeof(GameObject),
            true
        );

        visualMesh = (GameObject)EditorGUILayout.ObjectField(
            "Visual Mesh (optional)",
            visualMesh,
            typeof(GameObject),
            true
        );

        GUILayout.Space(10);

        GUI.enabled = armRoot != null;
        if (GUILayout.Button("Create Joint Hierarchy", GUILayout.Height(30)))
        {
            CreateJointHierarchy();
        }
        GUI.enabled = true;

        GUILayout.Space(10);

        EditorGUILayout.HelpBox(
            "Note: This creates EMPTY joint GameObjects with approximate positions.\n" +
            "You'll need to adjust their positions to match your arm's actual geometry.",
            MessageType.Warning
        );
    }

    void CreateJointHierarchy()
    {
        if (armRoot == null)
        {
            EditorUtility.DisplayDialog("Error", "Please select the arm root GameObject.", "OK");
            return;
        }

        // Check if hierarchy already exists
        if (armRoot.transform.Find("j1_turntable") != null)
        {
            bool overwrite = EditorUtility.DisplayDialog(
                "Hierarchy Exists",
                "Joint hierarchy already exists. Overwrite?",
                "Yes", "Cancel"
            );
            if (!overwrite) return;

            // Clear existing hierarchy
            for (int i = armRoot.transform.childCount - 1; i >= 0; i--)
            {
                Transform child = armRoot.transform.GetChild(i);
                if (child.name.StartsWith("j"))
                {
                    DestroyImmediate(child.gameObject);
                }
            }
        }

        // Create the joint hierarchy
        // These positions are approximations - user will need to adjust

        Transform j1 = CreateJoint("j1_turntable", armRoot.transform, Vector3.zero);
        Transform j2 = CreateJoint("j2_shoulder", j1, new Vector3(0, 0.1f, 0));
        Transform j3 = CreateJoint("j3_upper_arm", j2, new Vector3(0, 0.15f, 0));
        Transform j4 = CreateJoint("j4_forearm", j3, new Vector3(0, 0.15f, 0));
        Transform j5 = CreateJoint("j5_wrist", j4, new Vector3(0, 0.1f, 0));
        Transform j6 = CreateJoint("j6_end_effector", j5, new Vector3(0, 0.05f, 0));

        // Create gripper
        Transform gripperLeft = CreateJoint("gripper_left", j6, new Vector3(-0.02f, 0.03f, 0));
        Transform gripperRight = CreateJoint("gripper_right", j6, new Vector3(0.02f, 0.03f, 0));

        // Move visual mesh to end effector (or keep at root)
        if (visualMesh != null && visualMesh != armRoot)
        {
            bool moveToRoot = EditorUtility.DisplayDialog(
                "Visual Mesh",
                "Keep the visual mesh at the root?\n\n" +
                "Yes = Mesh stays at root (entire arm moves together)\n" +
                "No = Mesh moves to end effector (for tool/gripper mesh)",
                "Yes (Root)", "No (End Effector)"
            );

            if (!moveToRoot)
            {
                visualMesh.transform.SetParent(j6);
                visualMesh.transform.localPosition = Vector3.zero;
            }
        }

        // Add RoboticArmController if not present
        if (armRoot.GetComponent<RoboticArmController>() == null)
        {
            armRoot.AddComponent<RoboticArmController>();
        }

        // Auto-assign joints
        RoboticArmController controller = armRoot.GetComponent<RoboticArmController>();
        if (controller != null)
        {
            controller.AutoAssignJoints();
        }

        EditorUtility.DisplayDialog(
            "Success",
            "Joint hierarchy created!\n\n" +
            "Next steps:\n" +
            "1. Adjust joint positions in Scene view to match your arm\n" +
            "2. Add RoboticArmNetworkController component\n" +
            "3. Test the arm movement",
            "OK"
        );

        Debug.Log("[RoboticArmSetupWizard] Joint hierarchy created successfully!");
    }

    Transform CreateJoint(string name, Transform parent, Vector3 localPosition)
    {
        GameObject joint = new GameObject(name);
        joint.transform.SetParent(parent);
        joint.transform.localPosition = localPosition;
        joint.transform.localRotation = Quaternion.identity;
        joint.transform.localScale = Vector3.one;

        // Add gizmo for visualization
        var gizmo = joint.AddComponent<JointGizmo>();

        return joint.transform;
    }
}

/// <summary>
/// Simple gizmo drawer for joint visualization in editor.
/// </summary>
public class JointGizmo : MonoBehaviour
{
    void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, 0.02f);

        // Draw line to parent
        if (transform.parent != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(transform.parent.position, transform.position);
        }

        // Draw coordinate axes
        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, transform.right * 0.05f);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(transform.position, transform.up * 0.05f);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(transform.position, transform.forward * 0.05f);
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(transform.position, 0.03f);
    }
}
