from math import atan2, pi
import typing
from commands2 import Command
from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from robotstate import RobotState
from subsystems.drive.drivesubsystem import DriveSubsystem
from util.angleoptimize import optimizeAngle

from constants.trajectory import kRotationPGain, kRotationIGain, kRotationDGain


class AbsoluteRelativeDrive(Command):
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
        self.rotationY = rotationY
        self.rotationX = rotationX

        self.addRequirements(self.drive)
        self.setName(type(self).__name__)

    def rotation(self) -> float:
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
        rotation = self.rotation()
        if (
            abs(self.forward()) < 0.01
            and abs(self.sideways()) < 0.01
            and abs(rotation) < 0.01
        ):  # deadband should put to zero, put a delta errorbound for floats
            self.drive.defenseState()
        else:
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                # if we're on the other side, switch the controls around
                self.drive.arcadeDriveWithFactors(
                    -self.forward(),
                    -self.sideways(),
                    rotation,
                    DriveSubsystem.CoordinateMode.FieldRelative,
                )
            else:
                self.drive.arcadeDriveWithFactors(
                    self.forward(),
                    self.sideways(),
                    rotation,
                    DriveSubsystem.CoordinateMode.FieldRelative,
                )
