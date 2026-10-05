from typing import Callable
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


def trackTurretAtGoal(
    turret: TurretSubsystem, targetRelativeToTurret: Callable[[], Translation2d]
) -> Command:
    """
    Create a command that tracks the turret to a field-space goal.
    The `targetRelativeToTurret` supplier must return a `Translation2d` expressed in
    field coordinates, equal to (target position) minus (turret position). This is
    the vector from the turret to the target in field space.
    """

    _kTurretMinRad = kTurretMinAngle.radians()
    _kTurretMaxRad = kTurretMaxAngle.radians()
    _kTurretMidAngle = Rotation2d((_kTurretMinRad + _kTurretMaxRad) / 2)
    _kTurretRotSin = kTurretLocation.rotation().toRotation2d().sin()
    _kTurretRotCos = kTurretLocation.rotation().toRotation2d().cos()
    _kTurretTranslationNorm = kTurretLocation.translation().toTranslation2d().norm()

    def trackFunc():
        targetRelative = targetRelativeToTurret()
        turret.setClosedLoop(True)
        isShoot = RobotState.objective == RobotState.RobotMetaObjective.SHOOT
        robotPose = RobotState.getHubPose() if isShoot else RobotState.getFieldPose()
        robotVelocity = RobotState.robotFieldVelocity

        targetAngle = targetRelative.angle()
        turretAngle = targetAngle - robotPose.rotation()  # account for robot rotation

        # velocity compensation
        targetRelativeDistance = targetRelative.norm()
        turretRobotFrameVel = (
            Translation2d(
                -_kTurretRotSin,
                _kTurretRotCos,
            )
            * robotVelocity.omega
            * _kTurretTranslationNorm
        )
        turretFieldRefVel = turretRobotFrameVel.rotateBy(robotPose.rotation())
        turretVelocity = ChassisSpeeds(  # the velocity the turret moves in field space
            robotVelocity.vx + turretFieldRefVel.x,
            robotVelocity.vy + turretFieldRefVel.y,
            robotVelocity.omega,
        )
        distSquared = targetRelativeDistance * targetRelativeDistance

        if distSquared < 1e-6:
            goalTurretVel = -turretVelocity.omega
        else:
            goalTurretVel = (
                -turretVelocity.omega
                + (
                    targetRelative.x * turretVelocity.vy
                    - targetRelative.y * turretVelocity.vx
                )
                / distSquared
            )

        turret.setTurretGoalWithVel(
            optimizeAngle(
                _kTurretMidAngle,
                turretAngle,
            ),
            goalTurretVel,
        )  # ensure within possible rotations of the turret

    return cmd.run(trackFunc, turret).withName("TurretToGoal")


def trackedTurretStatic(turret: TurretSubsystem) -> Command:
    """Statically track the turret towards the robot objective"""

    def getTurretRelativeGoal() -> Translation2d:
        robotPose = (
            RobotState.getHubPose()
            if RobotState.objective == RobotState.RobotMetaObjective.SHOOT
            else RobotState.getFieldPose()
        )
        turretLocation = (pose3dFrom2d(robotPose) + kTurretLocation).toPose2d()
        return RobotState.objectiveLocation() - turretLocation.translation()

    return trackTurretAtGoal(turret, getTurretRelativeGoal).withName(
        "TurretTrackingStatic"
    )


def trackedTurretMoving(turret: TurretSubsystem) -> Command:
    """Track towards a target, compensating for the effects of relative velocity"""

    def getTurretRelativeGoal() -> Translation2d:
        robotPose = (
            RobotState.getHubPose()
            if RobotState.objective == RobotState.RobotMetaObjective.SHOOT
            else RobotState.getFieldPose()
        )
        turretLocation = (pose3dFrom2d(robotPose) + kTurretLocation).toPose2d()
        return RobotState.effectiveObjectiveLocation - turretLocation.translation()

    return trackTurretAtGoal(turret, getTurretRelativeGoal).withName(
        "TurretTrackingMoving"
    )


def trackedTurretBasedOnShooting(turret: TurretSubsystem) -> Command:
    """
    Track towards a target, compensating for relative velocity but only if desiring to shoot.
    Otherwise, just track statically (so the turret doesn't move around too much while driving
    if we're not shooting)
    """

    _turretTranslationOffset = kTurretLocation.translation().toTranslation2d()

    def getTurretRelativeGoal() -> Translation2d:
        isShoot = RobotState.objective == RobotState.RobotMetaObjective.SHOOT
        robotPose = RobotState.getHubPose() if isShoot else RobotState.getFieldPose()
        turretFieldTranslation = (
            robotPose.translation()
            + _turretTranslationOffset.rotateBy(robotPose.rotation())
        )
        isShooting = RobotState.isShooting
        objectiveLocation = (
            RobotState.effectiveObjectiveLocation
            if isShooting
            else RobotState.objectiveLocation()
        )
        return objectiveLocation - turretFieldTranslation

    return trackTurretAtGoal(turret, getTurretRelativeGoal).withName(
        "TurretTrackingShooting"
    )


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


def angleTurret(turret: TurretSubsystem, goal: Callable[[], Rotation2d]) -> Command:
    """Move the turret toward the target goal angle."""

    def overrideFunc():
        turret.setClosedLoop(True)
        turret.setTurretGoal(goal())

    return cmd.run(overrideFunc, turret).withName("AngleTurret")


def fudgeTurret(turret: TurretSubsystem, amount: Rotation2d) -> Command:
    def fudgeFunc():
        turret.turretFudge += amount

    return cmd.runOnce(fudgeFunc).withName("FudgeTurret")
