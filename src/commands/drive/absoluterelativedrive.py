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
        rx = self.rotationX()
        ry = self.rotationY()
        if rx == 0.0 and ry == 0.0:
            return 0.0

        targetRotation = atan2(rx, ry)  # rotate to be relative to driver

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            targetRotation += pi

        currentRot = RobotState.getRotation()
        optimizedDirection = optimizeAngle(
            currentRot, Rotation2d(targetRotation)
        ).radians()
        return self.rotationPid.calculate(currentRot.radians(), optimizedDirection)

    def execute(self) -> None:
        fwd = self.forward()
        side = self.sideways()
        rot = self.rotation()
        if (
            abs(fwd) < 0.01 and abs(side) < 0.01 and abs(rot) < 0.01
        ):  # deadband should put to zero, put a delta errorbound for floats
            self.drive.defenseState()
        else:
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                # if we're on the other side, switch the controls around
                self.drive.arcadeDriveWithFactors(
                    -fwd,
                    -side,
                    rot,
                    DriveSubsystem.CoordinateMode.FieldRelative,
                )
            else:
                self.drive.arcadeDriveWithFactors(
                    fwd,
                    side,
                    rot,
                    DriveSubsystem.CoordinateMode.FieldRelative,
                )
