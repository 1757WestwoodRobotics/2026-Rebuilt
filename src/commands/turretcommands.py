from commands2 import Command, cmd
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from robotstate import RobotState
from subsystems.turret.turretsubsystem import TurretSubsystem
from constants.turret import (
    kTurretLocation,
    kTurretMinAngle,
    kTurretMaxAngle,
)
from util.angleoptimize import optimizeAngle
from util.convenientmath import pose3dFrom2d


def trackedTurretStatic(turret: TurretSubsystem) -> Command:
    """Identify a target specified by kTargetLocation, and enable subsystem to move toward it."""

    def trackFunc():
        turret.setClosedLoop(True)
        robotPose = RobotState.getHubPose()
        robotVelocity = RobotState.robotFieldVelocity

        turretLocation = (pose3dFrom2d(robotPose) + kTurretLocation).toPose2d()
        targetRelativeToTurret = (
            RobotState.objectiveLocation() - turretLocation.translation()
        )
        targetAngle = targetRelativeToTurret.angle()
        turretAngle = targetAngle - robotPose.rotation()  # account for robot rotation

        # velocity compensation
        targetRelativeDistance = targetRelativeToTurret.norm()
        turretRobotFrameVel = (
            Translation2d(
                -kTurretLocation.rotation().toRotation2d().sin(),
                kTurretLocation.rotation().toRotation2d().cos(),
            )
            * robotVelocity.omega
            * kTurretLocation.translation().toTranslation2d().norm()
        )
        turretFieldRefVel = turretRobotFrameVel.rotateBy(robotPose.rotation())
        turretVelocity = ChassisSpeeds(  # the velocity the turret moves in field space
            robotVelocity.vx + turretFieldRefVel.x,
            robotVelocity.vy + turretFieldRefVel.y,
            robotVelocity.omega,
        )
        distSquared = targetRelativeDistance * targetRelativeDistance

        if distSquared < 1e-6:
            goalTurretVel = (
                -turretVelocity.omega
            )  # if we're basically on top of the target, just cancel out robot rotation
        else:
            goalTurretVel = (
                -turretVelocity.omega
                + (
                    targetRelativeToTurret.x * turretVelocity.vy
                    - targetRelativeToTurret.y * turretVelocity.vx
                )
                / distSquared
            )

        turret.setTurretGoalWithVel(
            optimizeAngle(
                Rotation2d((kTurretMinAngle.radians() + kTurretMaxAngle.radians()) / 2),
                turretAngle,
            ),
            goalTurretVel,
        )  # ensure within possible rotations of the turret

    return cmd.run(trackFunc, turret).withName("TurretTracking")


def trackedTurretMoving(turret: TurretSubsystem) -> Command:
    """Identify a target specified by kTargetLocation, and enable subsystem to move toward it."""

    def trackFunc():
        turret.setClosedLoop(True)
        robotPose = RobotState.getHubPose()
        robotVelocity = RobotState.robotFieldVelocity

        turretLocation = (pose3dFrom2d(robotPose) + kTurretLocation).toPose2d()

        targetRelativeToTurret = (
            RobotState.effectiveObjectiveLocation - turretLocation.translation()
        )
        targetRelativeDistance = RobotState.effectiveObjectiveDistance
        RobotState.effectiveObjectiveDistance = targetRelativeDistance
        targetAngle = targetRelativeToTurret.angle()
        turretAngle = targetAngle - robotPose.rotation()  # account for robot rotation

        # velocity compensation
        turretRobotFrameVel = (
            Translation2d(
                -kTurretLocation.rotation().toRotation2d().sin(),
                kTurretLocation.rotation().toRotation2d().cos(),
            )
            * robotVelocity.omega
            * kTurretLocation.translation().toTranslation2d().norm()
        )
        turretFieldRefVel = turretRobotFrameVel.rotateBy(robotPose.rotation())
        turretVelocity = ChassisSpeeds(  # the velocity the turret moves in field space
            robotVelocity.vx + turretFieldRefVel.x,
            robotVelocity.vy + turretFieldRefVel.y,
            robotVelocity.omega,
        )
        distSquared = targetRelativeDistance * targetRelativeDistance

        if distSquared < 1e-6:
            goalTurretVel = (
                -turretVelocity.omega
            )  # if we're basically on top of the target, just cancel out robot rotation
        else:
            goalTurretVel = (
                -turretVelocity.omega
                + (
                    targetRelativeToTurret.x * turretVelocity.vy
                    - targetRelativeToTurret.y * turretVelocity.vx
                )
                / distSquared
            )

        turret.setTurretGoalWithVel(
            optimizeAngle(
                Rotation2d((kTurretMinAngle.radians() + kTurretMaxAngle.radians()) / 2),
                turretAngle,
            ),
            goalTurretVel,
        )  # ensure within possible rotations of the turret

    return cmd.run(trackFunc, turret).withName("TurretTrackingMoving")


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
