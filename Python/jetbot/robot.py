"""
jetbot.robot — Robot class simulation shim.

Maps the JetBot Robot API to Unity "set_motors" TCP commands.
The Unity TruckController converts left/right wheel speeds into
differential-drive motion in the 3D scene.

Servo speeds are floats in [-1.0, 1.0], matching the real JetBot.
"""

import sim_client

ROBOT_ID = "truck_01"


class Robot:
    """
    Drop-in replacement for the real jetbot.Robot.

    Usage:
        robot = Robot()
        robot.forward(0.5)
        robot.stop()
    """

    def set_motors(self, left_speed: float, right_speed: float):
        """
        Set left and right wheel speeds independently.
        Speeds are floats in the range [-1.0, 1.0].
        """
        left_speed  = max(-1.0, min(1.0, float(left_speed)))
        right_speed = max(-1.0, min(1.0, float(right_speed)))
        sim_client.send_command({
            "command":  "set_motors",
            "robot_id": ROBOT_ID,
            "left":     left_speed,
            "right":    right_speed,
        })

    def forward(self, speed=1.0, duration=None):
        """Drive both wheels forward at the given speed."""
        self.set_motors(speed, speed)
        if duration is not None:
            import time
            time.sleep(duration)
            self.stop()

    def backward(self, speed: float = 1.0):
        """Drive both wheels backward at the given speed."""
        self.set_motors(-speed, -speed)

    def left(self, speed: float = 1.0):
        """Spin left in place (left wheel back, right wheel forward)."""
        self.set_motors(-speed, speed)

    def right(self, speed: float = 1.0):
        """Spin right in place (left wheel forward, right wheel back)."""
        self.set_motors(speed, -speed)

    def stop(self):
        """Stop both wheels immediately."""
        self.set_motors(0.0, 0.0)
