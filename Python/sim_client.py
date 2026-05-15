"""
sim_client.py — Singleton TCP client for the SenSym simulation layer.

Manages two TCP connections to Unity:
  - Port 5555: control commands (send JSON → receive "OK")
  - Port 5556: query commands  (send JSON → receive JSON response)

All shim modules (jetbot, SCSCtrl, jetson_utils, tensorrt) import this module
and call send_command() or send_query() without worrying about sockets.
"""

import socket
import json
import threading
import time

# ─── Configuration ──────────────────────────────────────────────────────────
UNITY_HOST = "127.0.0.1"
CONTROL_PORT = 5555   # existing Unity TcpServer (control commands → "OK")
QUERY_PORT   = 5556   # new SimQueryServer     (queries → JSON responses)

CONNECT_TIMEOUT = 5.0
RECV_TIMEOUT    = 3.0


class _SimClient:
    """
    Internal singleton.  Use the module-level functions below instead
    of instantiating this class directly.
    """

    def __init__(self):
        self._ctrl_sock = None  # type: socket.socket
        self._qry_sock  = None  # type: socket.socket
        self._lock = threading.Lock()
        self._connected = False

    # ── Connection management ────────────────────────────────────────────────

    def connect(self, host=UNITY_HOST, ctrl_port=CONTROL_PORT, qry_port=QUERY_PORT):
        """Connect to both Unity servers.  Called lazily on first use."""
        with self._lock:
            if self._connected:
                return True

            try:
                self._ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._ctrl_sock.settimeout(CONNECT_TIMEOUT)
                self._ctrl_sock.connect((host, ctrl_port))
                self._ctrl_sock.settimeout(RECV_TIMEOUT)
                print(f"[sim_client] Control channel connected → {host}:{ctrl_port}")
            except Exception as e:
                print(f"[sim_client] Could not connect control channel: {e}")
                self._ctrl_sock = None

            try:
                self._qry_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._qry_sock.settimeout(CONNECT_TIMEOUT)
                self._qry_sock.connect((host, qry_port))
                self._qry_sock.settimeout(RECV_TIMEOUT)
                print(f"[sim_client] Query channel connected    → {host}:{qry_port}")
            except Exception as e:
                print(f"[sim_client] Could not connect query channel: {e}")
                self._qry_sock = None

            self._connected = (self._ctrl_sock is not None
                               or self._qry_sock is not None)
            return self._connected

    def disconnect(self):
        with self._lock:
            for s in (self._ctrl_sock, self._qry_sock):
                try:
                    if s:
                        s.close()
                except Exception:
                    pass
            self._ctrl_sock = None
            self._qry_sock  = None
            self._connected = False
        print("[sim_client] Disconnected")

    # ── Control channel ─────────────────────────────────────────────────────

    def send_command(self, cmd: dict) -> bool:
        """
        Send a fire-and-forget control command.
        Unity responds with "OK\\n".
        Returns True on success.
        """
        self._ensure_connected()
        with self._lock:
            if self._ctrl_sock is None:
                return False
            try:
                payload = json.dumps(cmd).encode("utf-8")
                self._ctrl_sock.sendall(payload)
                resp = self._ctrl_sock.recv(64).decode("utf-8").strip()
                return resp == "OK"
            except Exception as e:
                print(f"[sim_client] send_command error: {e}")
                return False

    # ── Query channel ────────────────────────────────────────────────────────

    def send_query(self, cmd: dict) -> dict:
        """
        Send a query that expects a JSON response.
        Returns the parsed response dict, or {} on failure.
        """
        self._ensure_connected()
        with self._lock:
            if self._qry_sock is None:
                return {}
            try:
                payload = (json.dumps(cmd) + "\n").encode("utf-8")
                self._qry_sock.sendall(payload)

                # Read until newline (Unity sends complete JSON lines)
                buf = b""
                while b"\n" not in buf:
                    chunk = self._qry_sock.recv(65536)
                    if not chunk:
                        break
                    buf += chunk

                line = buf.split(b"\n")[0]
                return json.loads(line.decode("utf-8"))
            except Exception as e:
                print(f"[sim_client] send_query error: {e}")
                return {}

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _ensure_connected(self):
        if not self._connected:
            self.connect()


# ─── Module-level singleton + convenience functions ──────────────────────────

_client = _SimClient()


def connect(host: str = UNITY_HOST,
            ctrl_port: int = CONTROL_PORT,
            qry_port:  int = QUERY_PORT) -> bool:
    """Explicitly connect to Unity.  Optional — the shims connect lazily."""
    return _client.connect(host, ctrl_port, qry_port)


def disconnect():
    """Disconnect from Unity."""
    _client.disconnect()


def send_command(cmd: dict) -> bool:
    """Send a control command (fire-and-forget, expects OK)."""
    return _client.send_command(cmd)


def send_query(cmd: dict) -> dict:
    """Send a query command (expects JSON response)."""
    return _client.send_query(cmd)


# ─── Safety warning poller ────────────────────────────────────────────────────
# Background thread polls Unity every 2 seconds for safety warnings queued by
# SafetyMonitor.cs and prints them prominently to the PowerShell terminal.

_POLL_INTERVAL = 2.0
_WARN_BORDER   = "=" * 60

def _safety_poll_loop():
    while True:
        time.sleep(_POLL_INTERVAL)
        try:
            resp = _client.send_query({"command": "get_safety_warnings"})
            for w in resp.get("warnings", []):
                print(f"\n{_WARN_BORDER}")
                print(f"  [Unity SafetyMonitor] {w}")
                print(f"{_WARN_BORDER}\n")
        except Exception:
            pass   # Unity not running — silently skip

_poll_thread = threading.Thread(target=_safety_poll_loop, daemon=True)
_poll_thread.start()
