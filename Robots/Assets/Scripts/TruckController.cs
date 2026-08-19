using UnityEngine;
using RobotSimulator.Communication;

namespace RobotSimulator
{
    public class TruckController : MonoBehaviour
    {
        [Header("Movement Settings")]
        [SerializeField] private float moveSpeed    = 5f;
        [SerializeField] private float turnSpeed    = 90f;
        [SerializeField] private float acceleration = 10f;

        [Header("Robot Identity")]
        [SerializeField] private string robotId = "truck_01";

        private Vector3 _targetPosition;
        private float   _targetRotationY;
        private float   _currentSpeed;
        private bool    _moveToTarget;
        private float   _throttle;
        private float   _steering;

        private bool _subscribed = false;

        // ── Lifecycle ─────────────────────────────────────────────────────────

        private void Start()
        {
            _targetPosition  = transform.position;
            _targetRotationY = transform.eulerAngles.y;
            TrySubscribe();
        }

        private void Update()
        {
            // Re-try subscription every frame until it succeeds
            if (!_subscribed) TrySubscribe();

            if (_moveToTarget)
                MoveToTarget();
            else
                ApplyDirectControl();
        }

        private void OnDestroy()
        {
            if (TcpServer.Instance != null)
                TcpServer.Instance.OnMessageReceived -= HandleMessage;
        }

        // ── Subscription ──────────────────────────────────────────────────────

        private void TrySubscribe()
        {
            if (TcpServer.Instance == null) return;
            TcpServer.Instance.OnMessageReceived -= HandleMessage; // prevent duplicates
            TcpServer.Instance.OnMessageReceived += HandleMessage;
            _subscribed = true;
            Debug.Log($"[{robotId}] Subscribed to TcpServer");
        }

        // ── TCP message handler ───────────────────────────────────────────────

        private void HandleMessage(string message)
        {
            try
            {
                CommandData cmd = JsonUtility.FromJson<CommandData>(message);
                if (cmd == null) return;

                // Ignore commands for other robots
                if (!string.IsNullOrEmpty(cmd.robot_id) && cmd.robot_id != robotId)
                    return;

                switch (cmd.command)
                {
                    case "set_motors":
                        float l = Mathf.Clamp(cmd.left,  -1f, 1f);
                        float r = Mathf.Clamp(cmd.right, -1f, 1f);
                        _throttle     = (l + r) * 0.5f;
                        // (l - r), not (r - l). On a differential drive the
                        // LEFT track running faster turns the robot RIGHT,
                        // and _steering feeds transform.Rotate(0, +turn, 0),
                        // which is clockwise/right in Unity.
                        //
                        // This was backwards, so every steering command the
                        // Python side issued produced the opposite turn.
                        // Measured before the fix: set_motors(L=+0.40,
                        // R=+0.10) yawed -17.4 deg (left) where it should
                        // yaw right, and set_motors(L=-0.30, R=+0.30) spun
                        // right where it should spin left.
                        //
                        // Obstacle avoidance hid it for a long time, because
                        // going the WRONG way around an obstacle still
                        // avoids the obstacle — the robot just passes it on
                        // the other side. Course keeping is what exposed it:
                        // steering consistently away from a target turns the
                        // controller into a repulsive one, whose stable
                        // point is 180 deg off. Four consecutive live runs
                        // ended between 169 and 171 deg from their course,
                        // having driven backwards along it.
                        //
                        // jetbot_nav is the reference for this convention and
                        // is self-consistent: gap_follow._steer emits
                        // left = speed + turn with positive turn meaning
                        // right, and the offline simulator in
                        // test_gap_logic.py integrates yaw from (wl - wr).
                        // Both agree with each other and disagreed with this
                        // line, which is why the whole offline suite passed
                        // while the robot drove backwards.
                        _steering     = (l - r) * 0.5f;
                        _moveToTarget = false;
                        if (Mathf.Abs(_throttle) < 0.05f && Mathf.Abs(_steering) > 0.05f)
                            _throttle = 0.001f * Mathf.Sign(_steering);
                        Debug.Log($"[{robotId}] set_motors L={l:F2} R={r:F2} → throttle={_throttle:F2} steering={_steering:F2}");
                        break;

                    case "move":
                        _throttle     = Mathf.Clamp(cmd.throttle, -1f, 1f);
                        _steering     = Mathf.Clamp(cmd.steering, -1f, 1f);
                        _moveToTarget = false;
                        break;

                    case "stop":
                        _throttle     = 0;
                        _steering     = 0;
                        _moveToTarget = false;
                        break;

                    case "goto":
                        _targetPosition = new Vector3(cmd.x, transform.position.y, cmd.z);
                        _moveToTarget   = true;
                        break;

                    case "set_position":
                        transform.position = new Vector3(cmd.x, cmd.y, cmd.z);
                        _targetPosition    = transform.position;
                        break;

                    case "set_rotation":
                        transform.rotation = Quaternion.Euler(0, cmd.rotation_y, 0);
                        _targetRotationY   = cmd.rotation_y;
                        break;

                    default:
                        break;
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[{robotId}] Parse error: {e.Message}\nRaw: {message}");
            }
        }

        // ── Movement ──────────────────────────────────────────────────────────

        private void ApplyDirectControl()
        {
            float targetSpeed = _throttle * moveSpeed;
            _currentSpeed = Mathf.MoveTowards(_currentSpeed, targetSpeed, acceleration * Time.deltaTime);

            if (Mathf.Abs(_currentSpeed) > 0.01f || Mathf.Abs(_steering) > 0.01f)
            {
                float turn = _steering * turnSpeed * Time.deltaTime;
                transform.Rotate(0, turn, 0);
            }

            transform.position += transform.forward * _currentSpeed * Time.deltaTime;
        }

        private void MoveToTarget()
        {
            Vector3 dir = _targetPosition - transform.position;
            dir.y = 0;
            if (dir.magnitude < 0.1f)
            {
                _moveToTarget = false;
                _currentSpeed = 0;
                return;
            }
            Quaternion targetRot = Quaternion.LookRotation(dir);
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRot, turnSpeed * Time.deltaTime);
            float angle = Vector3.Angle(transform.forward, dir);
            if (angle < 30f)
            {
                _currentSpeed = Mathf.MoveTowards(_currentSpeed, moveSpeed, acceleration * Time.deltaTime);
                transform.Translate(Vector3.forward * _currentSpeed * Time.deltaTime);
            }
        }

        // ── Data class ────────────────────────────────────────────────────────

        [System.Serializable]
        private class CommandData
        {
            public string command;
            public string robot_id;
            public float  throttle;
            public float  steering;
            public float  left;
            public float  right;
            public float  x;
            public float  y;
            public float  z;
            public float  rotation_y;
        }
    }
}
