using UnityEngine;
using RobotSimulator.Communication;

/// <summary>
/// Network controller for RoboticArm - receives commands from Python via TCP.
/// Attach this to the same GameObject as RoboticArmController.
/// </summary>
[RequireComponent(typeof(RoboticArmController))]
public class RoboticArmNetworkController : MonoBehaviour
{
    [Header("Robot Identity")]
    [SerializeField] private string robotId = "arm_01";

    private RoboticArmController _armController;

    private void Start()
    {
        _armController = GetComponent<RoboticArmController>();

        if (_armController == null)
        {
            Debug.LogError($"[{robotId}] RoboticArmController not found!");
            enabled = false;
            return;
        }

        Debug.Log($"[{robotId}] RoboticArmNetworkController started");

        if (TcpServer.Instance != null)
        {
            TcpServer.Instance.OnMessageReceived += HandleMessage;
            Debug.Log($"[{robotId}] Connected to TcpServer");
        }
        else
        {
            Debug.LogWarning($"[{robotId}] TcpServer not found. Add TcpServer to scene.");
        }
    }

    private void OnDestroy()
    {
        if (TcpServer.Instance != null)
        {
            TcpServer.Instance.OnMessageReceived -= HandleMessage;
        }
    }

    private void HandleMessage(string message)
    {
        try
        {
            ArmCommandData cmd = JsonUtility.FromJson<ArmCommandData>(message);

            // Check if command is for this robot
            if (!string.IsNullOrEmpty(cmd.robot_id) && cmd.robot_id != robotId)
            {
                return;
            }

            Debug.Log($"[{robotId}] Processing command: {cmd.command}");

            switch (cmd.command)
            {
                case "arm_set_joint":
                    // Set individual joint angle
                    if (cmd.joint_index >= 0 && cmd.joint_index <= 5)
                    {
                        _armController.SetJoint(cmd.joint_index, cmd.angle);
                        Debug.Log($"[{robotId}] Set joint {cmd.joint_index} to {cmd.angle}°");
                    }
                    break;

                case "arm_set_joints":
                    // Set all joint angles at once
                    if (cmd.joints != null && cmd.joints.Length == 6)
                    {
                        _armController.SetJointAngles(
                            cmd.joints[0], cmd.joints[1], cmd.joints[2],
                            cmd.joints[3], cmd.joints[4], cmd.joints[5]
                        );
                        Debug.Log($"[{robotId}] Set all joints: [{string.Join(", ", cmd.joints)}]");
                    }
                    break;

                case "arm_set_gripper":
                    // Open/close gripper (0 = closed, 1 = open)
                    _armController.SetGripper(cmd.gripper_amount);
                    Debug.Log($"[{robotId}] Set gripper to {cmd.gripper_amount:F2}");
                    break;

                case "arm_home":
                    // Move to home position
                    _armController.GoHome();
                    Debug.Log($"[{robotId}] Moving to home position");
                    break;

                case "arm_pose":
                    // Move to preset pose
                    switch (cmd.pose_name?.ToLower())
                    {
                        case "pickup":
                        case "pickup_ready":
                            _armController.PosePickupReady();
                            Debug.Log($"[{robotId}] Moving to pickup pose");
                            break;

                        case "parked":
                        case "park":
                            _armController.PoseParked();
                            Debug.Log($"[{robotId}] Moving to parked pose");
                            break;

                        case "forward":
                        case "reach_forward":
                            _armController.PoseReachForward();
                            Debug.Log($"[{robotId}] Moving to reach forward pose");
                            break;

                        default:
                            Debug.LogWarning($"[{robotId}] Unknown pose: {cmd.pose_name}");
                            break;
                    }
                    break;

                case "arm_get_state":
                    // Get current arm state
                    float[] angles = _armController.GetJointAngles();
                    Vector3 endPos = _armController.GetEndEffectorPosition();
                    Debug.Log($"[{robotId}] Joint angles: [{string.Join(", ", angles)}]");
                    Debug.Log($"[{robotId}] End effector position: {endPos}");
                    break;

                // ── TTLServo shim commands ────────────────────────────────────
                case "servo_angle":
                    // {"command":"servo_angle","servo_id":N,"angle":deg,"direction":d}
                    ApplyServoAngle(cmd.servo_id, cmd.angle);
                    break;

                case "servo_sync":
                    // {"command":"servo_sync","sync_ids":[...],"sync_angles":[...]}
                    if (cmd.sync_ids != null && cmd.sync_angles != null)
                    {
                        int count = Mathf.Min(cmd.sync_ids.Length, cmd.sync_angles.Length);
                        for (int i = 0; i < count; i++)
                            ApplyServoAngle(cmd.sync_ids[i], cmd.sync_angles[i]);
                    }
                    break;

                case "servo_stop":
                    // Stop servo — in simulation just leave joint at current angle
                    Debug.Log($"[{robotId}] servo_stop servo_id={cmd.servo_id} (no-op in sim)");
                    break;

                default:
                    // Not an arm command, ignore silently
                    break;
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[{robotId}] Failed to parse arm command: {e.Message}");
        }
    }

    // ── TTLServo → arm joint mapping ─────────────────────────────────────────
    // Servo IDs from the real JETANK hardware:
    //   1 → joint 0 (base yaw)   2 → joint 1 (shoulder)  3 → joint 2 (elbow)
    //   4 → gripper              5 → joint 4 (wrist tilt)
    private void ApplyServoAngle(int servoId, float angleDeg)
    {
        switch (servoId)
        {
            case 1: _armController.SetJoint(0, angleDeg); break;
            case 2: _armController.SetJoint(1, angleDeg); break;
            case 3: _armController.SetJoint(2, angleDeg); break;
            case 4:
                // Gripper: hardware range [-100°, +100°] → sim [0=closed, 1=open]
                float grip = Mathf.Clamp01((angleDeg + 100f) / 200f);
                _armController.SetGripper(grip);
                break;
            case 5: _armController.SetJoint(4, angleDeg); break;
            default:
                Debug.LogWarning($"[{robotId}] Unknown servo_id: {servoId}");
                break;
        }
    }

    // Serializable class for JSON parsing
    [System.Serializable]
    private class ArmCommandData
    {
        public string command;
        public string robot_id;
        public int joint_index;
        public float angle;
        public float[] joints;
        public float gripper_amount;
        public string pose_name;
        // TTLServo shim fields
        public int servo_id;
        public int direction;
        public int[] sync_ids;
        public float[] sync_angles;
    }
}
