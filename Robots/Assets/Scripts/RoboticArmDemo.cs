using UnityEngine;

/// <summary>
/// RoboticArmDemo — Drop-in demo that cycles through poses.
/// Attach alongside RoboticArmController on the arm root.
/// </summary>
public class RoboticArmDemo : MonoBehaviour
{
    RoboticArmController arm;
    float timer = 0f;
    int poseIndex = 0;

    void Start()
    {
        arm = GetComponent<RoboticArmController>();
        if (arm == null)
        {
            Debug.LogError("RoboticArmDemo requires RoboticArmController on the same GameObject.");
            enabled = false;
        }
    }

    void Update()
    {
        if (arm == null) return;
        timer += Time.deltaTime;

        if (timer > 3f)
        {
            timer = 0f;
            poseIndex = (poseIndex + 1) % 5;
        }

        switch (poseIndex)
        {
            case 0: arm.GoHome(); break;
            case 1: arm.PosePickupReady(); break;
            case 2:
                arm.SetJointAngles(45, -20, 40, 90, -10, 0);
                arm.SetGripper(0f);
                break;
            case 3: arm.PoseParked(); break;
            case 4:
                arm.SetJointAngles(-60, 10, -30, -45, 30, 180);
                arm.SetGripper(1f);
                break;
        }
    }
}
