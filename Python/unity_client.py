"""
Unity Robot Simulator - Python Client

Connects to Unity TCP server and sends robot control commands.
"""

import socket
import json
import time
import threading
import queue
import logging
from typing import Optional, Callable, Dict, List

logger = logging.getLogger(__name__)


class UnityClient:
    """Client for communicating with Unity robot simulator."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5555):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self._running = False
        self._receive_thread = None
        self._callbacks: Dict[str, List[Callable]] = {}
        self._event_queue = queue.Queue()

    def connect(self) -> bool:
        """Connect to Unity server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(5.0)
            print(f"Connected to Unity at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from Unity server."""
        self._running = False
        if self._receive_thread:
            self._receive_thread.join(timeout=2.0)
        if self.socket:
            self.socket.close()
            self.socket = None
            print("Disconnected from Unity")

    def send_command(self, command: dict) -> bool:
        """Send a command to Unity and wait for acknowledgment."""
        if not self.socket:
            print("Not connected to Unity")
            return False

        try:
            message = json.dumps(command)
            self.socket.sendall(message.encode('utf-8'))

            # Wait for acknowledgment
            response = self.socket.recv(1024).decode('utf-8')
            return response.strip() == "OK"
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def start_listening(self):
        """Start background thread to receive messages from Unity."""
        if self._running:
            logger.warning("Already listening")
            return

        self._running = True
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()
        logger.info("Started listening for Unity messages")

    def _receive_loop(self):
        """Background thread that receives messages from Unity."""
        buffer = ""
        while self._running:
            try:
                # Use non-blocking socket with timeout
                self.socket.settimeout(0.1)
                data = self.socket.recv(4096).decode('utf-8')
                if not data:
                    # Connection closed
                    logger.warning("Unity connection closed")
                    break

                buffer += data

                # Process complete messages (delimited by newlines)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    # Skip acknowledgment messages
                    if line and line != "OK":
                        try:
                            msg = json.loads(line)
                            self._dispatch_message(msg)
                        except json.JSONDecodeError as e:
                            logger.error(f"Failed to parse message: {line} - {e}")

            except socket.timeout:
                # Normal timeout, continue loop
                continue
            except Exception as e:
                if self._running:
                    logger.error(f"Receive error: {e}")
                    break

    def register_callback(self, message_type: str, callback: Callable):
        """
        Register a callback for a specific message type.

        Args:
            message_type: Type of message (e.g., "event", "state_update")
            callback: Function to call when message is received
        """
        if message_type not in self._callbacks:
            self._callbacks[message_type] = []
        self._callbacks[message_type].append(callback)
        logger.info(f"Registered callback for message_type: {message_type}")

    def _dispatch_message(self, msg: dict):
        """Dispatch received message to registered callbacks."""
        msg_type = msg.get('message_type')
        if msg_type in self._callbacks:
            for callback in self._callbacks[msg_type]:
                try:
                    callback(msg)
                except Exception as e:
                    logger.error(f"Callback error for {msg_type}: {e}")
        else:
            logger.debug(f"No callbacks registered for message_type: {msg_type}")

    # ---- Robot Control Methods ----

    def move(self, robot_id: str, throttle: float, steering: float) -> bool:
        """
        Send direct movement command.

        Args:
            robot_id: ID of the robot to control
            throttle: Forward/backward speed (-1 to 1)
            steering: Left/right turn (-1 to 1)
        """
        return self.send_command({
            "command": "move",
            "robot_id": robot_id,
            "throttle": throttle,
            "steering": steering
        })

    def stop(self, robot_id: str) -> bool:
        """Stop the robot."""
        return self.send_command({
            "command": "stop",
            "robot_id": robot_id
        })

    def goto(self, robot_id: str, x: float, z: float) -> bool:
        """
        Move robot to target position.

        Args:
            robot_id: ID of the robot
            x: Target X position
            z: Target Z position
        """
        return self.send_command({
            "command": "goto",
            "robot_id": robot_id,
            "x": x,
            "z": z
        })

    def set_position(self, robot_id: str, x: float, y: float, z: float) -> bool:
        """Teleport robot to position."""
        return self.send_command({
            "command": "set_position",
            "robot_id": robot_id,
            "x": x,
            "y": y,
            "z": z
        })

    def set_rotation(self, robot_id: str, rotation_y: float) -> bool:
        """Set robot rotation (Y-axis, in degrees)."""
        return self.send_command({
            "command": "set_rotation",
            "robot_id": robot_id,
            "rotation_y": rotation_y
        })

    # ---- Robotic Arm Control Methods ----

    def arm_set_joint(self, robot_id: str, joint_index: int, angle: float) -> bool:
        """
        Set a single joint angle on the robotic arm.

        Args:
            robot_id: ID of the robotic arm
            joint_index: Joint index (0-5)
                0 = Base yaw (turntable)
                1 = Shoulder pitch
                2 = Elbow pitch
                3 = Wrist roll
                4 = Wrist pitch
                5 = Tool roll
            angle: Target angle in degrees
        """
        return self.send_command({
            "command": "arm_set_joint",
            "robot_id": robot_id,
            "joint_index": joint_index,
            "angle": angle
        })

    def arm_set_joints(self, robot_id: str, angles: List[float]) -> bool:
        """
        Set all 6 joint angles at once.

        Args:
            robot_id: ID of the robotic arm
            angles: List of 6 angles in degrees [j1, j2, j3, j4, j5, j6]
        """
        if len(angles) != 6:
            print(f"Error: Expected 6 joint angles, got {len(angles)}")
            return False

        return self.send_command({
            "command": "arm_set_joints",
            "robot_id": robot_id,
            "joints": angles
        })

    def arm_set_gripper(self, robot_id: str, open_amount: float) -> bool:
        """
        Control the gripper.

        Args:
            robot_id: ID of the robotic arm
            open_amount: 0.0 = fully closed, 1.0 = fully open
        """
        return self.send_command({
            "command": "arm_set_gripper",
            "robot_id": robot_id,
            "gripper_amount": max(0.0, min(1.0, open_amount))
        })

    def arm_go_home(self, robot_id: str) -> bool:
        """Move robotic arm to home position (all joints at 0 degrees)."""
        return self.send_command({
            "command": "arm_home",
            "robot_id": robot_id
        })

    def arm_pose(self, robot_id: str, pose_name: str) -> bool:
        """
        Move arm to a preset pose.

        Args:
            robot_id: ID of the robotic arm
            pose_name: Name of preset pose
                "pickup" or "pickup_ready" - Ready to pick up objects
                "parked" or "park" - Folded/parked position
                "forward" or "reach_forward" - Reaching forward
        """
        return self.send_command({
            "command": "arm_pose",
            "robot_id": robot_id,
            "pose_name": pose_name
        })


def main():
    """Demo: Control a truck with simple commands."""
    client = UnityClient()

    if not client.connect():
        print("Could not connect to Unity. Make sure the game is running.")
        return

    try:
        robot_id = "truck_01"

        print("\n=== Robot Control Demo ===")
        print("Commands will be sent to Unity...\n")

        # Move forward for 2 seconds
        print("Moving forward...")
        client.move(robot_id, throttle=0.5, steering=0.0)
        time.sleep(2.0)

        # Turn right while moving
        print("Turning right...")
        client.move(robot_id, throttle=0.5, steering=0.5)
        time.sleep(1.5)

        # Move backward
        print("Moving backward...")
        client.move(robot_id, throttle=-0.3, steering=0.0)
        time.sleep(1.5)

        # Stop
        print("Stopping...")
        client.stop(robot_id)
        time.sleep(0.5)

        # Go to specific position
        print("Going to position (5, 5)...")
        client.goto(robot_id, x=5.0, z=5.0)
        time.sleep(3.0)

        print("\nDemo complete!")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        client.stop("truck_01")
        client.disconnect()


if __name__ == "__main__":
    main()
