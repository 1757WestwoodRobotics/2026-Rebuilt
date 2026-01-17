from commands2 import Command, cmd
from wpimath.geometry import Rotation2d
from robotstate import RobotState
from subsystems.turret.turretsubsystem import TurretSubsystem

from constants.field import kTargetLocation
from util.angleoptimize import optimizeAngle


class TurretCommands:
    @staticmethod
    def trackedTurret(turret: TurretSubsystem) -> Command:
        def trackFunc():
            turret.setClosedLoop(True)
            robotPose = RobotState.getPose()
            targetRelativeToRobot = kTargetLocation - robotPose.translation()
            targetAngle = targetRelativeToRobot.angle()

            turretAngle = targetAngle - robotPose.rotation()

            turret.setTurretGoal(
                optimizeAngle(Rotation2d(), turretAngle)
            )  # ensure within possible rotations of the turret

        return cmd.run(trackFunc, turret).withName("TurretTracking")

    @staticmethod
    def runToGoal(turret: TurretSubsystem, goal) -> Command:
        return (
            TurretCommands.runOverride(turret, goal)
            .until(turret.atTarget)
            .withName("TurretGoal")
        )

    @staticmethod
    def runManual(turret: TurretSubsystem, volts: float) -> Command:
        def manualFunc():
            turret.setClosedLoop(False)
            turret.setTurretRawVolts(volts)

        return cmd.run(manualFunc, turret).withName("TurretManual")

    @staticmethod
    def runOverride(turret: TurretSubsystem, goal) -> Command:
        def overrideFunc():
            turret.setClosedLoop(True)
            turret.setTurretGoal(goal)

        return cmd.run(overrideFunc, turret).withName("TurretOverride")
