from commands2 import Command, cmd
from wpimath.geometry import Rotation2d
from robotstate import RobotState
from subsystems.turret.turretsubsystem import TurretSubsystem

from constants.field import kTargetLocation
from util.angleoptimize import optimizeAngle


def trackedTurret(turret: TurretSubsystem) -> Command:
    """Returns a command that tracks a target specified by kTargetLocation"""

    def trackFunc():
        turret.setClosedLoop(True)
        robotPose = RobotState.getPose()
        targetRelativeToRobot = kTargetLocation - robotPose.translation()
        targetAngle = (
            targetRelativeToRobot.angle()
        )  # targetAngle ignores rotation of robot

        turretAngle = targetAngle - robotPose.rotation()  # account for robot rotation

        turret.setTurretGoal(
            optimizeAngle(Rotation2d(), turretAngle)
        )  # ensure within possible rotations of the turret

    return cmd.run(trackFunc, turret).withName("TurretTracking")


def runToGoal(turret: TurretSubsystem, goal) -> Command:
    """Returns a command that moves the turret toward the target goal (using override)"""
    return runOverride(turret, goal).until(turret.atTarget).withName("TurretGoal")


def runManual(turret: TurretSubsystem, volts: float) -> Command:
    """Returns a command that moves the turret toward the target goal a certain amount (as dictated by volts supplied)"""

    def manualFunc():
        turret.setClosedLoop(False)
        turret.setTurretRawVolts(volts)

    return cmd.run(manualFunc, turret).withName("TurretManual")


def runOverride(turret: TurretSubsystem, goal) -> Command:
    """Returns a command that moves the turret toward the target"""

    def overrideFunc():
        turret.setClosedLoop(True)
        turret.setTurretGoal(goal)

    return cmd.run(overrideFunc, turret).withName("TurretOverride")
