using UnityEngine;
using RobotSimulator.Communication;

namespace RobotSimulator
{
    /// <summary>
    /// Controls truck movement based on commands received from Python.
    /// Attach this to your truck GameObject.
    /// </summary>
    public class TruckController : MonoBehaviour
    {
        [Header("Movement Settings")]
        [SerializeField] private float moveSpeed = 5f;
        [SerializeField] private float turnSpeed = 90f;
        [SerializeField] private float acceleration = 10f;

        [Header("Robot Identity")]
        [SerializeField] private string robotId = "truck_01";

        private Vector3 _targetPosition;
        private float _targetRotationY;
        private float _currentSpeed;
        private float _currentTurnSpeed;
        private bool _moveToTarget;

        // Direct control values (from Python)
        private float _throttle;  // -1 to 1
        private float _steering;  // -1 to 1

        private void Start()
        {
            _targetPosition = transform.position;
            _targetRotationY = transform.eulerAngles.y;

            Debug.Log($"[{robotId}] TruckController started");

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
            Debug.Log($"[{robotId}] Received message: {message}");

            // Parse JSON command
            try
            {
                CommandData cmd = JsonUtility.FromJson<CommandData>(message);

                // Check if command is for this robot
                if (!string.IsNullOrEmpty(cmd.robot_id) && cmd.robot_id != robotId)
                {
                    return;
                }

                switch (cmd.command)
                {
                    case "move":
                        // Direct throttle/steering control
                        _throttle = Mathf.Clamp(cmd.throttle, -1f, 1f);
                        _steering = Mathf.Clamp(cmd.steering, -1f, 1f);
                        _moveToTarget = false;
                        break;

                    case "stop":
                        _throttle = 0;
                        _steering = 0;
                        _moveToTarget = false;
                        break;

                    case "goto":
                        // Move to absolute position
                        _targetPosition = new Vector3(cmd.x, transform.position.y, cmd.z);
                        _moveToTarget = true;
                        break;

                    case "set_position":
                        // Teleport to position
                        transform.position = new Vector3(cmd.x, cmd.y, cmd.z);
                        _targetPosition = transform.position;
                        break;

                    case "set_rotation":
                        // Set rotation (y-axis)
                        transform.rotation = Quaternion.Euler(0, cmd.rotation_y, 0);
                        _targetRotationY = cmd.rotation_y;
                        break;

                    case "get_state":
                        // State is logged for now; full implementation would send back via TCP
                        Debug.Log($"[{robotId}] Position: {transform.position}, Rotation: {transform.eulerAngles}");
                        break;

                    default:
                        Debug.LogWarning($"[{robotId}] Unknown command: {cmd.command}");
                        break;
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[{robotId}] Failed to parse command: {e.Message}");
            }
        }

        private void Update()
        {
            if (_moveToTarget)
            {
                MoveToTarget();
            }
            else
            {
                ApplyDirectControl();
            }
        }

        private void ApplyDirectControl()
        {
            // Apply throttle (forward/backward)
            float targetSpeed = _throttle * moveSpeed;
            _currentSpeed = Mathf.MoveTowards(_currentSpeed, targetSpeed, acceleration * Time.deltaTime);

            // Apply steering (only when moving)
            if (Mathf.Abs(_currentSpeed) > 0.1f)
            {
                float turn = _steering * turnSpeed * Time.deltaTime * Mathf.Sign(_currentSpeed);
                transform.Rotate(0, turn, 0);
            }

            // Move forward
            Vector3 movement = transform.forward * _currentSpeed * Time.deltaTime;
            transform.position += movement;
        }

        private void MoveToTarget()
        {
            Vector3 direction = _targetPosition - transform.position;
            direction.y = 0;

            if (direction.magnitude < 0.1f)
            {
                _moveToTarget = false;
                _currentSpeed = 0;
                Debug.Log($"[{robotId}] Reached target position: {_targetPosition}");
                return;
            }

            // Rotate towards target
            Quaternion targetRotation = Quaternion.LookRotation(direction);
            transform.rotation = Quaternion.RotateTowards(
                transform.rotation,
                targetRotation,
                turnSpeed * Time.deltaTime
            );

            // Move forward if roughly facing target
            float angleToTarget = Vector3.Angle(transform.forward, direction);
            if (angleToTarget < 30f)
            {
                _currentSpeed = Mathf.MoveTowards(_currentSpeed, moveSpeed, acceleration * Time.deltaTime);
                transform.Translate(Vector3.forward * _currentSpeed * Time.deltaTime);
            }
        }

        // Serializable class for JSON parsing
        [System.Serializable]
        private class CommandData
        {
            public string command;
            public string robot_id;
            public float throttle;
            public float steering;
            public float x;
            public float y;
            public float z;
            public float rotation_y;
        }
    }
}
