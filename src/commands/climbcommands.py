from commands2 import Command, cmd
from subsystems.climber.climbersubsystem import ClimberSubsystem

from constants.climber import kDeployedHeight, kRetractedHeight, kClimberBumpAmount


def deployClimber(climber: ClimberSubsystem) -> Command:
    """Deploy the climber to extended height"""
    return cmd.runOnce(lambda: climber.setClimberGoal(kDeployedHeight))


def retractClimber(climber: ClimberSubsystem) -> Command:
    """Retract the climber to retracted height"""
    return cmd.runOnce(lambda: climber.setClimberGoal(kRetractedHeight))


def bumpClimber(climber: ClimberSubsystem, bumpAmount: float) -> Command:
    """Bump the climber up or down by a specified amount"""
    return cmd.runOnce(lambda: climber.bumpClimberGoal(bumpAmount))


def bumpUp(climber: ClimberSubsystem) -> Command:
    """Bump the climber up by a small amount"""
    return bumpClimber(climber, kClimberBumpAmount)


def bumpDown(climber: ClimberSubsystem) -> Command:
    """Bump the climber down by a small amount"""
    return bumpClimber(climber, -kClimberBumpAmount)
