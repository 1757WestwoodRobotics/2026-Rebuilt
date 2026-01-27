from commands2 import Command, cmd
from wpimath.geometry import Rotation2d

from robotstate import RobotState
from subsystems.turret.turretsubsystem import TurretSubsystem
from constants.field import kCloseHubLocation
from constants.turret import kTurretLocation
from util.angleoptimize import optimizeAngle
from util.fliputil import FlipUtil


def trackedTurret(turret: TurretSubsystem) -> Command:
    """Identify a target specified by kTargetLocation, and enable subsystem to move toward it."""

    def trackFunc():
        turret.setClosedLoop(True)
        robotPose = RobotState.getPose()

        turret2DLocationOnField = (
            kTurretLocation.translation().toTranslation2d() + robotPose.translation()
        )  # add Turret location transform in 2D onto the robotPose
        targetRelativeToTurret = (
            FlipUtil.fieldTranslation(kCloseHubLocation) - turret2DLocationOnField
        )
        targetAngle = targetRelativeToTurret.angle()

        turretAngle = targetAngle - robotPose.rotation()  # account for robot rotation

        turret.setTurretGoal(
            optimizeAngle(Rotation2d(), turretAngle)
        )  # ensure within possible rotations of the turret

    return cmd.run(trackFunc, turret).withName("TurretTracking")


def runToGoal(turret: TurretSubsystem, goal) -> Command:
    """Move the turret toward the supplied goal angle until reached (using override)."""
    return runOverride(turret, goal).until(turret.atTarget).withName("TurretGoal")


def runManual(turret: TurretSubsystem, volts: float) -> Command:
    """Move the turret a certain amount (as dictated by volts supplied)."""

    def manualFunc():
        turret.setClosedLoop(False)
        turret.setTurretRawVolts(volts)

    return cmd.run(manualFunc, turret).withName("TurretManual")


def runOverride(turret: TurretSubsystem, goal) -> Command:
    """Move the turret toward the target goal angle."""

    def overrideFunc():
        turret.setClosedLoop(True)
        turret.setTurretGoal(goal)

    return cmd.run(overrideFunc, turret).withName("TurretOverride")
