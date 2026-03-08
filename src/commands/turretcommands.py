from commands2 import Command, cmd
from wpimath.geometry import Rotation2d

from robotstate import RobotState
from subsystems.turret.turretsubsystem import TurretSubsystem
from constants.turret import kTurretLocation, kTurretMinAngle, kTurretMaxAngle
from util.angleoptimize import optimizeAngle
from util.convenientmath import pose3dFrom2d


def trackedTurret(turret: TurretSubsystem) -> Command:
    """Identify a target specified by kTargetLocation, and enable subsystem to move toward it."""

    def trackFunc():
        turret.setClosedLoop(True)
        robotPose = RobotState.getHubPose()

        turretLocation = (pose3dFrom2d(robotPose) + kTurretLocation).toPose2d()
        targetRelativeToTurret = (
            RobotState.objectiveLocation() - turretLocation.translation()
        )
        targetAngle = targetRelativeToTurret.angle()

        turretAngle = targetAngle - robotPose.rotation()  # account for robot rotation

        turret.setTurretGoal(
            optimizeAngle(
                Rotation2d((kTurretMinAngle.radians() + kTurretMaxAngle.radians()) / 2),
                turretAngle,
            )
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
