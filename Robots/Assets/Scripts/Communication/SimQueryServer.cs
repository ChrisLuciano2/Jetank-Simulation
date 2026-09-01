using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace RobotSimulator.Communication
{
    /// <summary>
    /// Second TCP server (port 5556) that handles data-returning queries
    /// from the Python simulation layer.
    ///
    /// Queries supported:
    ///   get_frame      → returns a JPEG frame from the virtual camera
    ///   detect_objects → returns bounding boxes of visible tagged objects
    ///
    /// Responses are single JSON lines (terminated with \n).
    ///
    /// SETUP: Attach this script to the same GameObject as TcpServer.
    ///        Also attach SimCamera to the scene's main Camera object.
    /// </summary>
    public class SimQueryServer : MonoBehaviour
    {
        [Header("Query Server Settings")]
        [SerializeField] private int queryPort = 5556;
        [SerializeField] private bool autoStart = true;

        private TcpListener  _listener;
        private Thread       _listenerThread;
        private volatile bool _isRunning;

        public static SimQueryServer Instance { get; private set; }

        private void Awake()
        {
            if (Instance != null && Instance != this) { Destroy(gameObject); return; }
            Instance = this;
        }

        private void Start()
        {
            if (autoStart) StartServer();
        }

        public void StartServer()
        {
            if (_isRunning) return;
            _isRunning = true;
            _listenerThread = new Thread(ListenLoop) { IsBackground = true };
            _listenerThread.Start();
            Debug.Log($"[SimQueryServer] Listening on port {queryPort}");
        }

        private void ListenLoop()
        {
            try
            {
                _listener = new TcpListener(IPAddress.Any, queryPort);
                _listener.Start();
                while (_isRunning)
                {
                    if (_listener.Pending())
                    {
                        TcpClient client = _listener.AcceptTcpClient();
                        Debug.Log("[SimQueryServer] Python query client connected");
                        new Thread(() => HandleClient(client)) { IsBackground = true }.Start();
                    }
                    Thread.Sleep(10);
                }
            }
            catch (Exception e)
            {
                if (_isRunning) Debug.LogError($"[SimQueryServer] {e.Message}");
            }
        }

        private void HandleClient(TcpClient client)
        {
            try
            {
                NetworkStream stream = client.GetStream();
                byte[] buf = new byte[4096];
                StringBuilder sb = new StringBuilder();

                while (_isRunning && client.Connected)
                {
                    if (stream.DataAvailable)
                    {
                        int n = stream.Read(buf, 0, buf.Length);
                        if (n == 0) break;
                        sb.Append(Encoding.UTF8.GetString(buf, 0, n));

                        // Process all complete lines (delimited by \n)
                        string accumulated = sb.ToString();
                        int nl;
                        while ((nl = accumulated.IndexOf('\n')) >= 0)
                        {
                            string line = accumulated.Substring(0, nl).Trim();
                            accumulated  = accumulated.Substring(nl + 1);
                            if (line.Length == 0) continue;

                            string response = ProcessQuery(line);
                            byte[] respBytes = Encoding.UTF8.GetBytes(response + "\n");
                            stream.Write(respBytes, 0, respBytes.Length);
                            stream.Flush();
                        }
                        sb.Clear();
                        sb.Append(accumulated);
                    }
                    Thread.Sleep(1);
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[SimQueryServer] Client disconnected: {e.Message}");
            }
            finally
            {
                client?.Close();
            }
        }

        /// <summary>
        /// Dispatch a JSON query string and return a JSON response string.
        /// Called on the background thread — reads cached data from SimCamera
        /// which is updated each Unity Update() on the main thread.
        /// </summary>
        private string ProcessQuery(string json)
        {
            try
            {
                QueryData q = JsonUtility.FromJson<QueryData>(json);
                if (q == null || q.command == null)
                    return "{\"status\":\"error\",\"message\":\"invalid json\"}";

                // Empty/missing robot_id defaults to "truck_01" — keeps every
                // existing single-robot script and test working unchanged.
                string robotId = string.IsNullOrEmpty(q.robot_id) ? "truck_01" : q.robot_id;

                switch (q.command)
                {
                    case "get_frame":
                        return HandleGetFrame(robotId);

                    case "detect_objects":
                        return HandleDetectObjects(robotId);

                    case "get_safety_warnings":
                        return HandleGetSafetyWarnings();

                    case "get_proximity_scan":
                        return HandleGetProximityScan(robotId);

                    case "get_pose":
                        return HandleGetPose(robotId);

                    case "get_block_pose":
                        // Dev-only ground truth for a GraspableBlock, keyed by its
                        // blockId (e.g. "block_A_large") -- see GraspableBlock.cs.
                        return HandleGetBlockPose(q.block_id);

                    case "get_arm_state":
                        // Arm queries use their own id convention ("arm_01"/"arm_02"),
                        // separate from the chassis's "truck_01"/"truck_02" — the
                        // caller must pass the arm's robot_id explicitly here rather
                        // than relying on the truck_01 default above.
                        return HandleGetArmState(q.robot_id);

                    default:
                        return $"{{\"status\":\"error\",\"message\":\"unknown query: {q.command}\"}}";
                }
            }
            catch (Exception e)
            {
                return $"{{\"status\":\"error\",\"message\":\"{e.Message}\"}}";
            }
        }

        // ── Frame query ───────────────────────────────────────────────────────

        private string HandleGetFrame(string robotId)
        {
            SimCamera cam = SimCamera.Get(robotId);
            if (cam == null)
                return $"{{\"status\":\"error\",\"message\":\"no SimCamera registered for robot_id '{robotId}'\"}}";

            string b64 = cam.GetCachedFrameBase64();
            if (b64 == null)
                return "{\"status\":\"error\",\"message\":\"no frame available yet\"}";

            // Manual JSON build (no third-party library needed; b64 is safe chars)
            return "{\"status\":\"ok\",\"jpeg\":\"" + b64 + "\"}";
        }

        // ── Safety warnings query ─────────────────────────────────────────────

        private string HandleGetSafetyWarnings()
        {
            string[] warnings = SafetyMonitor.DrainWarnings();
            var sb = new System.Text.StringBuilder();
            sb.Append("{\"status\":\"ok\",\"warnings\":[");
            for (int i = 0; i < warnings.Length; i++)
            {
                if (i > 0) sb.Append(",");
                sb.Append($"\"{EscapeJson(warnings[i])}\"");
            }
            sb.Append("]}");
            return sb.ToString();
        }

        /// <summary>
        /// Escape a string for embedding in a JSON string literal.
        ///
        /// Backslash and quote are the obvious ones; the CONTROL CHARACTERS
        /// are the ones that actually bit. A raw newline is illegal inside a
        /// JSON string, so a single multi-line warning produced
        /// "Unterminated string" on the Python side and took the whole
        /// response with it — every other warning in the same batch included.
        /// Warnings are written by humans for humans and will contain
        /// newlines sooner or later, so escape rather than forbid them.
        /// </summary>
        private static string EscapeJson(string s)
        {
            var sb = new System.Text.StringBuilder(s.Length + 16);
            foreach (char c in s)
            {
                switch (c)
                {
                    case '\\': sb.Append("\\\\"); break;
                    case '"':  sb.Append("\\\""); break;
                    case '\n': sb.Append("\\n");  break;
                    case '\r': sb.Append("\\r");  break;
                    case '\t': sb.Append("\\t");  break;
                    case '\b': sb.Append("\\b");  break;
                    case '\f': sb.Append("\\f");  break;
                    default:
                        if (c < 0x20)
                            sb.Append("\\u").Append(((int)c).ToString("x4"));
                        else
                            sb.Append(c);
                        break;
                }
            }
            return sb.ToString();
        }

        // ── Detection query ───────────────────────────────────────────────────

        private string HandleDetectObjects(string robotId)
        {
            SimCamera cam = SimCamera.Get(robotId);
            if (cam == null)
                return "{\"status\":\"ok\",\"objects\":[]}";

            return cam.GetCachedDetectionsJson();
        }

        // ── Proximity query ───────────────────────────────────────────────────

        private string HandleGetProximityScan(string robotId)
        {
            ProximitySensor sensor = ProximitySensor.Get(robotId);
            if (sensor == null)
                return $"{{\"status\":\"error\",\"message\":\"no ProximitySensor registered for robot_id '{robotId}'\"}}";

            float[] scan     = sensor.GetCachedScan();
            float   fov      = sensor.ScanFovDegrees;
            float   maxRange = sensor.MaxRange;

            // InvariantCulture matters: a machine set to a comma-decimal
            // locale would otherwise emit "3,50" and break Python's parse.
            var ic = System.Globalization.CultureInfo.InvariantCulture;

            // max_range lets Python auto-calibrate its slow-down band to the
            // sensor's actual reach (a mismatch here once made "completely
            // clear" indistinguishable from "obstacle at max range").
            var sb = new System.Text.StringBuilder();
            sb.Append("{\"status\":\"ok\",\"fov\":");
            sb.Append(fov.ToString("F1", ic));
            sb.Append(",\"count\":");
            sb.Append(scan.Length);
            sb.Append(",\"max_range\":");
            sb.Append(maxRange.ToString("F2", ic));
            sb.Append(",\"distances\":[");
            for (int i = 0; i < scan.Length; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append(scan[i].ToString("F2", ic));
            }
            sb.Append("]}");
            return sb.ToString();
        }

        // ── Pose query ────────────────────────────────────────────────────────

        /// <summary>
        /// Ground-truth robot pose. DEV ONLY: the real JETANK cannot answer
        /// this about itself, so anything in the deployment path that reads it
        /// is cheating and will not survive contact with hardware. It exists
        /// so a live navigation run can be scored against the truth instead of
        /// against the heading estimator's own opinion of itself.
        /// </summary>
        private string HandleGetBlockPose(string blockId)
        {
            GraspableBlock block = GraspableBlock.Get(blockId);
            if (block == null)
                return $"{{\"status\":\"error\",\"message\":\"no GraspableBlock registered for blockId '{blockId}'\"}}";

            Vector3 p = block.GetCachedPosition();
            var ic = System.Globalization.CultureInfo.InvariantCulture;
            return "{\"status\":\"ok\",\"x\":" + p.x.ToString("F4", ic) +
                   ",\"y\":" + p.y.ToString("F4", ic) +
                   ",\"z\":" + p.z.ToString("F4", ic) + "}";
        }

        private string HandleGetPose(string robotId)
        {
            ProximitySensor sensor = ProximitySensor.Get(robotId);
            if (sensor == null)
                return $"{{\"status\":\"error\",\"message\":\"no ProximitySensor registered for robot_id '{robotId}'\"}}";

            float[] p = sensor.GetCachedPose();
            if (p == null || p.Length < 4)
                return "{\"status\":\"error\",\"message\":\"no pose available yet\"}";

            // InvariantCulture for the same reason as the scan above: a
            // comma-decimal locale would emit "3,50" and break Python's parse.
            var ic = System.Globalization.CultureInfo.InvariantCulture;

            var sb = new System.Text.StringBuilder();
            sb.Append("{\"status\":\"ok\",\"x\":");
            sb.Append(p[0].ToString("F4", ic));
            sb.Append(",\"y\":");
            sb.Append(p[1].ToString("F4", ic));
            sb.Append(",\"z\":");
            sb.Append(p[2].ToString("F4", ic));
            sb.Append(",\"yaw_deg\":");
            sb.Append(p[3].ToString("F4", ic));
            sb.Append("}");
            return sb.ToString();
        }

        // ── Arm state query ──────────────────────────────────────────────────
        //
        // Joint angles, gripper amount, and IsHolding — all self-knowledge a
        // real TTLServo-driven arm has via its own commanded/servo-feedback
        // state, unlike chassis pose above. Lets Python confirm a grab
        // actually attached a block rather than assuming it did.

        private string HandleGetArmState(string armRobotId)
        {
            RoboticArmNetworkController net = RoboticArmNetworkController.Get(armRobotId);
            if (net == null || net.ArmController == null)
                return $"{{\"status\":\"error\",\"message\":\"no arm registered for robot_id '{armRobotId}'\"}}";

            // Read only the cached snapshot below -- never a live Transform.
            // GetEndEffectorPosition()/GripperOpenAmount etc. touch Unity
            // Component/Transform APIs, which throw when called from this
            // background thread (this was a real bug here, not theoretical:
            // "get_position can only be called from the main thread").
            RoboticArmController.ArmStateSnapshot state = net.ArmController.GetCachedState();
            if (state == null)
                return "{\"status\":\"error\",\"message\":\"arm state not ready yet\"}";

            var ic = System.Globalization.CultureInfo.InvariantCulture;

            var sb = new System.Text.StringBuilder();
            sb.Append("{\"status\":\"ok\",\"joint_angles\":[");
            for (int i = 0; i < state.JointAngles.Length; i++)
            {
                if (i > 0) sb.Append(',');
                sb.Append(state.JointAngles[i].ToString("F2", ic));
            }
            sb.Append("],\"gripper_amount\":");
            sb.Append(state.GripperAmount.ToString("F2", ic));
            sb.Append(",\"is_holding\":");
            sb.Append(state.IsHoldingSnapshot ? "true" : "false");
            sb.Append(",\"held_object\":");
            sb.Append(state.IsHoldingSnapshot ? $"\"{EscapeJson(state.HeldObjectNameSnapshot)}\"" : "null");
            sb.Append(",\"end_effector\":{\"x\":");
            sb.Append(state.EndEffectorPosition.x.ToString("F4", ic));
            sb.Append(",\"y\":");
            sb.Append(state.EndEffectorPosition.y.ToString("F4", ic));
            sb.Append(",\"z\":");
            sb.Append(state.EndEffectorPosition.z.ToString("F4", ic));
            sb.Append("},\"gripper_position\":{\"x\":");
            sb.Append(state.GripperPosition.x.ToString("F4", ic));
            sb.Append(",\"y\":");
            sb.Append(state.GripperPosition.y.ToString("F4", ic));
            sb.Append(",\"z\":");
            sb.Append(state.GripperPosition.z.ToString("F4", ic));
            sb.Append("}}");
            return sb.ToString();
        }

        // ── Lifecycle ─────────────────────────────────────────────────────────

        private void OnDestroy()       => StopServer();
        private void OnApplicationQuit() => StopServer();

        public void StopServer()
        {
            _isRunning = false;
            _listener?.Stop();
        }

        [System.Serializable]
        private class QueryData { public string command; public string robot_id; public string block_id; }
    }
}
