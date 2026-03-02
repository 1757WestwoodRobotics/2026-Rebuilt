from typing import Callable
from commands2 import Command, cmd

from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem


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
        # Placeholder for actual distance to speed calculation
        return distance() * 0.1  # Example conversion factor

    return fireAtSpeed(flywheel, calculateSpeed).withName("ShootWithDistance")
