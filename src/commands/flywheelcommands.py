from typing import Callable
from commands2 import Command, cmd

from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem

from constants.shooting import kShootingMap, kFeedFlywheelMap


def fireAtSpeed(flywheel: FlywheelSubsystem, speed: Callable[[], float]) -> Command:
    """
    Returns a command that fires the shooter at a given speed
    """
    return (
        cmd.run(lambda: flywheel.setGoal(speed()), flywheel)
        .finallyDo(lambda _interrupted: flywheel.flywheelIdle())
        .withName("FireAtSpeed")
    )


def shootWithDistance(
    flywheel: FlywheelSubsystem, distance: Callable[[], float]
) -> Command:
    """
    Returns a command that fires the shooter at a speed determined by the distance to the target
    """

    def calculateSpeed():
        return float(
            kShootingMap(distance())
        )  # pykit doesn't support numpy variables, so we need to convert the output of the interp function to a float

    return fireAtSpeed(flywheel, calculateSpeed).withName("ShootWithDistance")


def feedWithDistance(
    flywheel: FlywheelSubsystem, distance: Callable[[], float]
) -> Command:
    """
    Returns a command that fires the shooter at a speed determined by the distance to the target
    """

    def calculateSpeed():
        return float(
            kFeedFlywheelMap(distance())
        )  # pykit doesn't support numpy variables, so we need to convert the output of the interp function to a float

    return fireAtSpeed(flywheel, calculateSpeed).withName("FeedWithDistance")


def idle(flywheel: FlywheelSubsystem):
    """
    Idles the flywheel
    """
    return cmd.run(lambda: flywheel.flywheelIdle(), flywheel).withName("IdleFlywheel")
