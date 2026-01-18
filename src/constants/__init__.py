"""

The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!

Physical constants must have their units specified
Default units:
    Length: meters
    Angle: radians

Axes Convention (right hand rule):
    Translation:
        +X: forward
        +Y: left
        +Z: up

    Rotation:
        +rotate around X: counterclockwise looking from the front, 0 aligned with +Y
        +rotate around Y: counterclockwise looking from the left, 0 aligned with +Z
        +rotate around Z: counterclockwise looking from the top, 0 aligned with +X

Swerve Module Layout:
    Drive (input) -> Drive Gearing -> Wheel (output)
    Steer (input) -> Steer Gearing -> Swerve (output)
"""

from enum import Enum
import os

from wpilib import RobotBase

kRobotUpdatePeriod = 1 / 50
kRobotUpdateFrequency = 1 / kRobotUpdatePeriod


class RobotModes(Enum):
    """Enum for robot modes."""

    REAL = 1
    SIMULATION = 2
    REPLAY = 3


kSimMode = (
    RobotModes.REPLAY
    if "LOG_PATH" in os.environ and os.environ["LOG_PATH"] != ""
    else RobotModes.SIMULATION
)
kRobotMode = RobotModes.REAL if RobotBase.isReal() else kSimMode
