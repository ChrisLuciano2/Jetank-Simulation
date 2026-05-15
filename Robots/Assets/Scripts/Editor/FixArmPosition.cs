using UnityEngine;
using UnityEditor;

/// <summary>
/// Tools → Fix Arm Position
/// Finds the RoboticArm in the scene and repositions it
/// to sit on top of the truck using actual renderer bounds.
/// </summary>
public static class FixArmPosition
{
    [MenuItem("Tools/Fix Arm Position")]
    public static void Fix()
    {
        // Find the truck root (has TruckController)
        var truck = Object.FindFirstObjectByType<RobotSimulator.TruckController>();
        if (truck == null)
        {
            EditorUtility.DisplayDialog("Error", "No TruckController found in scene. Run scene setup first.", "OK");
            return;
        }

        // Find the arm (has RoboticArmController)
        var arm = Object.FindFirstObjectByType<RoboticArmController>();
        if (arm == null)
        {
            EditorUtility.DisplayDialog("Error", "No RoboticArmController found in scene. Run scene setup first.", "OK");
            return;
        }

        // Make sure arm is a child of the truck
        arm.transform.SetParent(truck.transform);

        // Calculate the top of the truck mesh in local space
        float truckTopY = GetLocalTopY(truck.gameObject);
        Debug.Log($"[FixArmPosition] Truck local top Y = {truckTopY:F3}");

        // Place arm centered on top of the truck, slightly forward
        arm.transform.localPosition = new Vector3(0f, truckTopY + 0.02f, 0.1f);
        arm.transform.localRotation = Quaternion.identity;
        arm.transform.localScale    = Vector3.one;

        // Mark scene dirty
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(
            UnityEditor.SceneManagement.EditorSceneManager.GetActiveScene());

        Debug.Log($"[FixArmPosition] Arm repositioned to local Y={truckTopY + 0.02f:F3}");
        EditorUtility.DisplayDialog("Done",
            $"Arm repositioned on top of truck (local Y = {truckTopY + 0.02f:F2}).\n\nSave the scene with Ctrl+S.",
            "OK");
    }

    /// <summary>
    /// Returns the highest Y point of all Renderer bounds on a GameObject
    /// expressed in the GameObject's own local space.
    /// </summary>
    private static float GetLocalTopY(GameObject go)
    {
        float maxY = 0f;
        foreach (Renderer r in go.GetComponentsInChildren<Renderer>())
        {
            // Convert world-space bounds top to local space
            Vector3 worldTop = new Vector3(
                r.bounds.center.x,
                r.bounds.max.y,
                r.bounds.center.z);
            float localY = go.transform.InverseTransformPoint(worldTop).y;
            if (localY > maxY) maxY = localY;
        }
        return maxY;
    }
}
