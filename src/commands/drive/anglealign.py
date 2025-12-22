import typing
from commands2 import Command
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpilib import DriverStation

from robotstate import RobotState
from subsystems.drive.drivesubsystem import DriveSubsystem
from util.angleoptimize import optimizeAngle

from constants.trajectory import kRotationPGain, kRotationIGain, kRotationDGain


class AngleAlignDrive(Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        sideways: typing.Callable[[], float],
    ) -> None:
        Command.__init__(self)

        self.drive = drive
        self.forward = forward
        self.sideways = sideways
        self.rotationPid = PIDController(kRotationPGain, kRotationIGain, kRotationDGain)
        self.targetRotation = Rotation2d()
        self.addRequirements(self.drive)
        self.setName(type(self).__name__)

    def initialize(self) -> None:
        currentRotation = RobotState.getRotation()
        self.targetRotation = Rotation2d.fromDegrees(
            round(currentRotation.degrees() / 60) * 60
        )

    def rotation(self) -> float:
        optimizedDirection = optimizeAngle(
            RobotState.getRotation(), self.targetRotation
        ).radians()
        return self.rotationPid.calculate(
            RobotState.getRotation().radians(), optimizedDirection
        )

    def execute(self) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.drive.arcadeDriveWithFactors(
                -self.forward(),
                -self.sideways(),
                self.rotation(),
                DriveSubsystem.CoordinateMode.FieldRelative,
            )
        else:
            self.drive.arcadeDriveWithFactors(
                self.forward(),
                self.sideways(),
                self.rotation(),
                DriveSubsystem.CoordinateMode.FieldRelative,
            )
