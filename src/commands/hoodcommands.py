from commands2 import Command
from commands2 import cmd as Commands
from wpimath.geometry import Rotation2d

from robotstate import RobotState
from subsystems.hood.hoodsubsystem import HoodSubsystem
from constants.hood import (
    kHoodMinAngle,
    kHoodMaxAngle,
    kHoodTolerance,
    kHoodFudgeAmount,
)


def bumpHoodUp(hood: HoodSubsystem) -> Command:
    """
    Manual hood increment up
    """
    return Commands.runOnce(lambda: hood.bumpAngle(kHoodFudgeAmount)).withName(
        "bumpHoodUp"
    )


def bumpHoodDown(hood: HoodSubsystem) -> Command:
    """
    Manual hood increment down
    """
    return Commands.runOnce(lambda: hood.bumpAngle(-kHoodFudgeAmount)).withName(
        "bumpHoodDown"
    )
