"""
robot_coord.config — Identity and transport configuration for the
coordination layer.

Everything here comes from environment variables, not code, so the exact
same script file deploys to either robot (real or simulated) without
edits — matching this repo's existing "write once" model for the
jetbot/SCSCtrl shims:

    # Robot A — real hardware
    ROBOT_ID=robot_a MQTT_BROKER_HOST=192.168.1.50 python build_script.py

    # Robot B — real hardware
    ROBOT_ID=robot_b MQTT_BROKER_HOST=192.168.1.50 python build_script.py

    # Both robots, simulated on one dev PC (two terminals, broker on
    # localhost — see the Mosquitto setup note in README.md)
    ROBOT_ID=robot_a python build_script.py
    ROBOT_ID=robot_b python build_script.py

Defaults assume a lone dev run against a broker on localhost, so a script
that hasn't set any of this still imports and connects rather than
raising on missing config.
"""

import os

# ── Identity ──────────────────────────────────────────────────────────────

ROBOT_ID = os.environ.get("ROBOT_ID", "robot_a")

# ── MQTT transport ───────────────────────────────────────────────────────

MQTT_BROKER_HOST = os.environ.get("MQTT_BROKER_HOST", "127.0.0.1")
MQTT_BROKER_PORT = int(os.environ.get("MQTT_BROKER_PORT", "1883"))

# One topic per message kind, shared by both robots. Keep these in sync
# with the schema in messages.py if you rename anything.
TOPIC_STATE = "sensym/build/state"   # per-cycle pose/state updates (schema: messages.build_state_message)
TOPIC_FLAG  = "sensym/build/flag"    # defect / assist-request flags (schema: messages.build_flag_message)
