"""
SCSCtrl — simulation shim for the Feetech SCS/STS servo library.

Students write:
    from SCSCtrl import TTLServo
    TTLServo.servoAngleCtrl(1, 45, 1, 200)

This package provides a TTLServo module whose functions send servo commands
to the Unity simulation via TCP, without any real serial port.
"""

from . import TTLServo

__all__ = ["TTLServo"]
