import math

from commands2 import Command
from commands2 import cmd as Commands
from wpimath.geometry import Rotation2d

from robotstate import RobotState
from subsystems.hood.hoodsubsystem import HoodSubsystem
from constants.hood import (
    kHoodMinAngle,
    kHoodMaxAngle,
    kHoodFudgeAmount,
)
from constants.field import kCloseHubLocation, kHubHeight
from constants.math import kMetersPerFoot, kDegreesPerRevolution


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


def distanceToHoodAngle(distance: float) -> Rotation2d:
    """
    Using Desmos solver to convert distance into hood angle
    """

    h = kHubHeight / kMetersPerFoot
    xS = distance / kMetersPerFoot
    launcherOffset = 5

    # Desmos trajectory math
    vY = math.sqrt(h**2 + (launcherOffset - xS) * 32 * 2)
    tM = (vY + h) / 32
    vX = xS / tM

    thetaD = math.atan(vY / vX) * (kDegreesPerRevolution / math.pi)
    thetaD = max(kHoodMinAngle.degrees(), min(kHoodMaxAngle.degrees(), thetaD))
    return Rotation2d.fromDegrees(thetaD)


def autoAngleFromHub(hood: HoodSubsystem) -> Command:
    """
    Hood angle adjustment based on distance to the hub
    """

    def aimHood():
        hood.setClosedLoop(True)

        # get robot's current position on the field
        robotPose = RobotState.getHubPose()

        # calculate distance from robot to hub
        distanceToHub = robotPose.translation().distance(kCloseHubLocation)
        hoodAngle = distanceToHoodAngle(distanceToHub)
        hood.setHoodGoal(hoodAngle)

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
