"""
robot_coord.world_model — The shared understanding both robots read from
and write to over MQTT (brainstorm doc section 4).

This is intentionally *not* a database — it's the latest message received
from each robot_id, kept in memory, plus a staleness check. Section 6 is
explicit that a robot which stops hearing from its peer should pause
rather than assume progress: "if the shared communication channel drops,
default behavior should be pause and wait, not keep building blind." A
missing or stale entry here is how the Coordinator (coordinator.py)
detects that condition.
"""

import json
import os
import threading
import time

try:
    import paho.mqtt.client as mqtt
except ImportError as e:  # pragma: no cover - import-time guidance only
    raise ImportError(
        "robot_coord needs paho-mqtt. Install it on both the dev PC and "
        "both physical robots: pip install paho-mqtt"
    ) from e

from . import config

STALE_AFTER_SECONDS = 5.0


class SharedWorldModel:
    """
    MQTT-backed shared state. One instance per robot process.

    connect() starts a background network thread (paho's loop_start()) —
    publish_state()/publish_flag() and the get_*() readers are safe to
    call from the caller's own thread while it runs.
    """

    def __init__(self, robot_id=None, broker_host=None, broker_port=None,
                 stale_after=STALE_AFTER_SECONDS):
        self.robot_id = robot_id or config.ROBOT_ID
        self._broker_host = broker_host or config.MQTT_BROKER_HOST
        self._broker_port = broker_port or config.MQTT_BROKER_PORT
        self._stale_after = stale_after

        self._lock = threading.Lock()
        self._latest_state = {}     # robot_id -> last state message
        self._received_at = {}      # robot_id -> time.time() of last state message
        self._flag_log = []         # list of flag messages, newest last

        client_id = f"sensym-{self.robot_id}-{os.getpid()}"
        self._client = mqtt.Client(client_id=client_id, clean_session=True)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._connected = threading.Event()

    # ── Connection ────────────────────────────────────────────────────────

    def connect(self, timeout=10.0):
        """Connect to the broker and start the background network thread."""
        self._client.connect(self._broker_host, self._broker_port, keepalive=15)
        self._client.loop_start()
        if not self._connected.wait(timeout):
            raise ConnectionError(
                f"[world_model] no MQTT CONNACK from {self._broker_host}:"
                f"{self._broker_port} within {timeout}s"
            )

    def disconnect(self):
        self._client.loop_stop()
        self._client.disconnect()

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc != 0:
            print(f"[world_model] MQTT connect failed, rc={rc}")
            return
        client.subscribe(config.TOPIC_STATE, qos=1)
        client.subscribe(config.TOPIC_FLAG, qos=1)
        self._connected.set()
        print(f"[world_model] {self.robot_id} connected to "
              f"{self._broker_host}:{self._broker_port}")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            print(f"[world_model] dropped unparseable message on {msg.topic}")
            return
        rid = payload.get("robot_id")
        if not rid:
            return
        with self._lock:
            if msg.topic == config.TOPIC_STATE:
                self._latest_state[rid] = payload
                self._received_at[rid] = time.time()
            elif msg.topic == config.TOPIC_FLAG:
                self._flag_log.append(payload)

    # ── Publishing ────────────────────────────────────────────────────────

    def publish_state(self, message):
        """Publish a state message (see messages.build_state_message)."""
        self._client.publish(config.TOPIC_STATE, json.dumps(message), qos=1)
        # Reflect our own publish immediately rather than waiting for the
        # broker echo, so get_state(self.robot_id) is never one cycle stale.
        with self._lock:
            self._latest_state[message["robot_id"]] = message
            self._received_at[message["robot_id"]] = time.time()

    def publish_flag(self, message):
        """Publish a flag message (see messages.build_flag_message)."""
        self._client.publish(config.TOPIC_FLAG, json.dumps(message), qos=1)
        with self._lock:
            self._flag_log.append(message)

    # ── Reading ───────────────────────────────────────────────────────────

    def get_state(self, robot_id):
        """Latest state message from robot_id, or None if never heard from."""
        with self._lock:
            return self._latest_state.get(robot_id)

    def get_other_robots(self):
        """{robot_id: latest_state} for every robot_id except our own."""
        with self._lock:
            return {rid: s for rid, s in self._latest_state.items()
                    if rid != self.robot_id}

    def is_stale(self, robot_id):
        """
        True if we've never heard from robot_id, or its last message is
        older than the staleness window. Callers should treat "stale" the
        same as "not there" — see coordinator.py's timeout-as-clearance
        behavior, which follows section 6's "pause and wait" rule in the
        other direction: don't wait forever on a peer that's gone dark.
        """
        with self._lock:
            ts = self._received_at.get(robot_id)
        return ts is None or (time.time() - ts) > self._stale_after

    def flags_since(self, timestamp=0.0, robot_id=None):
        """Flag messages at or after `timestamp`, optionally filtered by robot_id."""
        with self._lock:
            flags = list(self._flag_log)
        return [f for f in flags
                if f.get("timestamp", 0) >= timestamp
                and (robot_id is None or f.get("robot_id") == robot_id)]
