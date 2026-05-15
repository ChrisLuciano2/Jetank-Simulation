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

                switch (q.command)
                {
                    case "get_frame":
                        return HandleGetFrame();

                    case "detect_objects":
                        return HandleDetectObjects();

                    case "get_safety_warnings":
                        return HandleGetSafetyWarnings();

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

        private string HandleGetFrame()
        {
            if (SimCamera.Instance == null)
                return "{\"status\":\"error\",\"message\":\"SimCamera not found\"}";

            string b64 = SimCamera.Instance.GetCachedFrameBase64();
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
                string esc = warnings[i].Replace("\\", "\\\\").Replace("\"", "\\\"");
                sb.Append($"\"{esc}\"");
            }
            sb.Append("]}");
            return sb.ToString();
        }

        // ── Detection query ───────────────────────────────────────────────────

        private string HandleDetectObjects()
        {
            if (SimCamera.Instance == null)
                return "{\"status\":\"ok\",\"objects\":[]}";

            return SimCamera.Instance.GetCachedDetectionsJson();
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
        private class QueryData { public string command; }
    }
}
