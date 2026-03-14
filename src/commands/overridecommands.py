from commands2 import Command
from pykit.networktables.loggednetworknumber import LoggedNetworkNumber
from wpimath.geometry import Rotation2d
from commands.flywheelcommands import fireAtSpeed
from commands.hoodcommands import angleHood
from commands.turretcommands import angleTurret
from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem
from subsystems.hood.hoodsubsystem import HoodSubsystem
from subsystems.turret.turretsubsystem import TurretSubsystem

_flywheelSetpoint = LoggedNetworkNumber("Flywheel/Setpoint", 0.0)
_hoodSetpoint = LoggedNetworkNumber("Hood/Setpoint", 0.0)
_turretSetpoint = LoggedNetworkNumber("Turret/Setpoint", 0.0)


def overrideFlywheel(flywheel: FlywheelSubsystem) -> Command:
    """
    Returns a command that allows the operator to override the flywheel speed using network input
    """

    return fireAtSpeed(flywheel, lambda: _flywheelSetpoint.value).withName(
        "OverrideFlywheel"
    )


def overrideHood(hood: HoodSubsystem) -> Command:
    """
    Returns a command that allows the operator to override the hood angle using network input
    """

    return angleHood(
        hood, lambda: Rotation2d.fromDegrees(_hoodSetpoint.value)
    ).withName("OverrideHood")


def overrideTurret(turret: TurretSubsystem) -> Command:
    """
    Returns a command that allows the operator to override the turret angle using network input
    """
    return angleTurret(
        turret, lambda: Rotation2d.fromDegrees(_turretSetpoint.value)
    ).withName("OverrideTurret")
