from commands2 import Command, cmd
from pykit.networktables.loggednetworknumber import LoggedNetworkNumber
from wpimath.geometry import Rotation2d
from commands.flywheelcommands import fireAtSpeed
from commands.hoodcommands import angleHood
from commands.turretcommands import angleTurret
from robotstate import RobotState
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

    return (
        fireAtSpeed(flywheel, lambda: _flywheelSetpoint.value)
        .alongWith(cmd.runOnce(lambda: setattr(RobotState, "flywheelOverriden", True)))
        .finallyDo(lambda _interrupted: setattr(RobotState, "flywheelOverriden", False))
        .withName("OverrideFlywheel")
        .ignoringDisable(True)
    )


def overrideHood(hood: HoodSubsystem) -> Command:
    """
    Returns a command that allows the operator to override the hood angle using network input
    """

    return (
        angleHood(hood, lambda: Rotation2d.fromDegrees(_hoodSetpoint.value))
        .alongWith(cmd.runOnce(lambda: setattr(RobotState, "hoodOverriden", True)))
        .finallyDo(lambda _interrupted: setattr(RobotState, "hoodOverriden", False))
        .withName("OverrideHood")
        .ignoringDisable(True)
    )


def overrideTurret(turret: TurretSubsystem) -> Command:
    """
    Returns a command that allows the operator to override the turret angle using network input
    """
    return (
        angleTurret(turret, lambda: Rotation2d.fromDegrees(_turretSetpoint.value))
        .alongWith(cmd.runOnce(lambda: setattr(RobotState, "turretOverriden", True)))
        .finallyDo(lambda _interrupted: setattr(RobotState, "turretOverriden", False))
        .withName("OverrideTurret")
        .ignoringDisable(True)
    )
