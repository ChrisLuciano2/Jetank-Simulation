"""
jetbot — simulation shim for the NVIDIA JetBot library.

Students write:
    from jetbot import Robot
    robot = Robot()
    robot.forward(0.4)

This package intercepts those calls and sends them to the Unity simulation
via TCP.  The interface is identical to the real jetbot library so the same
code runs unchanged on physical hardware.
"""

from .robot import Robot

__all__ = ["Robot"]
