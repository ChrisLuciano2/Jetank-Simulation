using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace RobotSimulator.Communication
{
    /// <summary>
    /// TCP Server that receives commands from Python and queues them for processing.
    /// Attach this to a GameObject in your scene.
    /// </summary>
    public class TcpServer : MonoBehaviour
    {
        [Header("Server Settings")]
        [SerializeField] private int port = 5555;
        [SerializeField] private bool autoStart = true;

        private TcpListener _listener;
        private Thread _listenerThread;
        private ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();
        private ConcurrentQueue<string> _outgoingQueue = new ConcurrentQueue<string>();
        private volatile bool _isRunning;
        private NetworkStream _clientStream;

        public event Action<string> OnMessageReceived;

        public static TcpServer Instance { get; private set; }

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }

        private void Start()
        {
            if (autoStart)
            {
                StartServer();
            }
        }

        public void StartServer()
        {
            if (_isRunning) return;

            _isRunning = true;
            _listenerThread = new Thread(ListenForClients)
            {
                IsBackground = true
            };
            _listenerThread.Start();
            Debug.Log($"[TcpServer] Started on port {port}");
        }

        private void ListenForClients()
        {
            try
            {
                _listener = new TcpListener(IPAddress.Any, port);
                _listener.Start();

                while (_isRunning)
                {
                    if (_listener.Pending())
                    {
                        TcpClient client = _listener.AcceptTcpClient();
                        Debug.Log("[TcpServer] Client connected");
                        Thread clientThread = new Thread(() => HandleClient(client))
                        {
                            IsBackground = true
                        };
                        clientThread.Start();
                    }
                    Thread.Sleep(10);
                }
            }
            catch (Exception e)
            {
                if (_isRunning)
                {
                    Debug.LogError($"[TcpServer] Error: {e.Message}");
                }
            }
        }

        private void HandleClient(TcpClient client)
        {
            try
            {
                NetworkStream stream = client.GetStream();
                _clientStream = stream; // Store for sending messages
                byte[] buffer = new byte[4096];

                while (_isRunning && client.Connected)
                {
                    // Process outgoing messages
                    while (_outgoingQueue.TryDequeue(out string outgoingMessage))
                    {
                        try
                        {
                            byte[] outgoingBytes = Encoding.UTF8.GetBytes(outgoingMessage + "\n");
                            stream.Write(outgoingBytes, 0, outgoingBytes.Length);
                            stream.Flush();
                        }
                        catch (Exception ex)
                        {
                            Debug.LogError($"[TcpServer] Error sending message: {ex.Message}");
                        }
                    }

                    // Process incoming messages
                    if (stream.DataAvailable)
                    {
                        int bytesRead = stream.Read(buffer, 0, buffer.Length);
                        if (bytesRead > 0)
                        {
                            string message = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                            _messageQueue.Enqueue(message);

                            // Send acknowledgment
                            byte[] response = Encoding.UTF8.GetBytes("OK\n");
                            stream.Write(response, 0, response.Length);
                        }
                    }
                    Thread.Sleep(1);
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[TcpServer] Client disconnected: {e.Message}");
            }
            finally
            {
                _clientStream = null;
                client?.Close();
            }
        }

        private void Update()
        {
            // Process messages on main thread
            while (_messageQueue.TryDequeue(out string message))
            {
                OnMessageReceived?.Invoke(message);
            }
        }

        /// <summary>
        /// Send a message to the connected Python client.
        /// </summary>
        public void SendMessage(string json)
        {
            if (_clientStream != null && _clientStream.CanWrite)
            {
                _outgoingQueue.Enqueue(json);
            }
        }

        private void OnDestroy()
        {
            StopServer();
        }

        private void OnApplicationQuit()
        {
            StopServer();
        }

        public void StopServer()
        {
            _isRunning = false;
            _listener?.Stop();
            Debug.Log("[TcpServer] Stopped");
        }
    }
}
