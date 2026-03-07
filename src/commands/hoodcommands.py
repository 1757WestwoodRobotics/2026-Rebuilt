from typing import Callable

from commands2 import Command
from commands2 import cmd as Commands

from subsystems.hood.hoodsubsystem import HoodSubsystem
from constants.hood import (
    kHoodMinAngle,
    kHoodMaxAngle,
    kHoodFudgeAmount,
)

from constants.shooting import kHoodAngleMap


def bumpHoodUp(hood: HoodSubsystem) -> Command:
    """
    Manual hood increment up
    """
    return Commands.runOnce(lambda: hood.bumpAngle(kHoodFudgeAmount), hood).withName(
        "bumpHoodUp"
    )


def bumpHoodDown(hood: HoodSubsystem) -> Command:
    """
    Manual hood increment down
    """
    return Commands.runOnce(lambda: hood.bumpAngle(-kHoodFudgeAmount), hood).withName(
        "bumpHoodDown"
    )


def angleHoodWithDistance(
    hood: HoodSubsystem, distance: Callable[[], float]
) -> Command:
    """
    Hood angle adjustment based on distance to the hub
    """

    def aimHood():
        hood.setClosedLoop(True)
        hood.setHoodGoal(kHoodAngleMap(distance()))

    return Commands.run(aimHood, hood).withName("autoAngleFromHub")


def moveToMin(hood: HoodSubsystem) -> Command:
    """
    Move hood to minimum angle
    """
    return Commands.run(lambda: hood.setHoodGoal(kHoodMinAngle), hood).withName(
        "moveToMin"
    )


def moveToMax(hood: HoodSubsystem) -> Command:
    """
    Move hood to maximum angle
    """
    return Commands.run(lambda: hood.setHoodGoal(kHoodMaxAngle), hood).withName(
        "moveToMax"
    )
