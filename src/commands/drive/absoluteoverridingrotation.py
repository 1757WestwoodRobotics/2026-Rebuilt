from math import atan2, pi
import typing
from commands2 import Command
from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from robotstate import RobotState
from subsystems.drive.drivesubsystem import DriveSubsystem
from util.angleoptimize import optimizeAngle
from util.convenientmath import clampRotation

from constants.trajectory import kRotationPGain, kRotationIGain, kRotationDGain
from constants.turret import kTurretMinAngle, kTurretMaxAngle, kTurretTolerance

from constants.drive import kSOTMSpeedMultiplier


class AbsoluteOverridingRotationDrive(Command):
    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        sideways: typing.Callable[[], float],
        rotationX: typing.Callable[[], float],
        rotationY: typing.Callable[[], float],
    ) -> None:
        Command.__init__(self)

        self.drive = drive
        self.forward = forward
        self.sideways = sideways
        self.rotationPid = PIDController(kRotationPGain, kRotationIGain, kRotationDGain)
        self.correctionPid = PIDController(
            kRotationPGain, kRotationIGain, kRotationDGain
        )
        self.rotationY = rotationY
        self.rotationX = rotationX

        self.addRequirements(self.drive)
        self.setName(type(self).__name__)

    def commandedRotation(self) -> float:
        targetRotation = atan2(
            self.rotationX(), self.rotationY()
        )  # rotate to be relative to driver
        if self.rotationX() == 0 and self.rotationY() == 0:
            return 0

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            targetRotation += pi

        optimizedDirection = optimizeAngle(
            RobotState.getRotation(), Rotation2d(targetRotation)
        ).radians()
        return self.rotationPid.calculate(
            RobotState.getRotation().radians(), optimizedDirection
        )

    def execute(self) -> None:
        commandedRotation = self.commandedRotation()
        robotPose = RobotState.getFieldPose()
        turretLocation = RobotState.getTurretPose().toPose2d()
        targetAngle = (
            RobotState.effectiveObjectiveLocation - turretLocation.translation()
        ).angle()
        turretRequestedAngle = targetAngle - robotPose.rotation()
        turretRequestedAngle = optimizeAngle(
            Rotation2d((kTurretMinAngle.radians() + kTurretMinAngle.radians()) / 2),
            turretRequestedAngle,
        )

        outsideOfBounds = (
            turretRequestedAngle.radians()
            < kTurretMinAngle.radians() + kTurretTolerance.radians()
            or turretRequestedAngle.radians()
            > kTurretMaxAngle.radians() - kTurretTolerance.radians()
        )
        if outsideOfBounds:
            clampedTurretAngle = clampRotation(
                turretRequestedAngle,
                Rotation2d(kTurretMinAngle.radians() + kTurretTolerance.radians()),
                Rotation2d(kTurretMaxAngle.radians() - kTurretTolerance.radians()),
            )
            driveAngle = targetAngle - clampedTurretAngle
            correction = optimizeAngle(RobotState.getRotation(), driveAngle).radians()
            rotation = self.correctionPid.calculate(
                robotPose.rotation().radians(), correction
            )
        else:
            rotation = commandedRotation

        if (
            abs(self.forward()) < 0.01
            and abs(self.sideways()) < 0.01
            and abs(rotation) < 0.01
        ):  # deadband should put to zero, put a delta errorbound for floats
            self.drive.defenseState()
        else:
            forward = self.forward() * kSOTMSpeedMultiplier
            sideways = self.sideways() * kSOTMSpeedMultiplier
            rotation = rotation * kSOTMSpeedMultiplier
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                # if we're on the other side, switch the controls around
                self.drive.arcadeDriveWithFactors(
                    -forward,
                    -sideways,
                    rotation,
                    DriveSubsystem.CoordinateMode.FieldRelative,
                )
            else:
                self.drive.arcadeDriveWithFactors(
                    forward,
                    sideways,
                    rotation,
                    DriveSubsystem.CoordinateMode.FieldRelative,
                )
