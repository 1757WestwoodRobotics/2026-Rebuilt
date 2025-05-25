import typing

from ntcore import NetworkTableInstance
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d

from commands2 import Command
from subsystems.drivesubsystem import DriveSubsystem

import constants


class TargetRelativeDrive(Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        sideways: typing.Callable[[], float],
        rotation: typing.Callable[[], float],
    ) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.forward = forward
        self.sideways = sideways
        self.rotation = rotation

        self.angleController = PIDController(
            constants.kTargetRelativeDriveAnglePGain,
            constants.kTargetRelativeDriveAngleIGain,
            constants.kTargetRelativeDriveAngleDGain,
        )

        self.angleValid = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kTargetAngleRelativeToRobotKeys.validKey)
            .subscribe(False)
        )
        self.angle = (
            NetworkTableInstance.getDefault()
            .getStructTopic(
                constants.kTargetAngleRelativeToRobotKeys.valueKey, Rotation2d
            )
            .subscribe(Rotation2d)
        )

        self.addRequirements(self.drive)
        self.setName(__class__.__name__)

    def execute(self) -> None:
        angleControllerOutput = 0
        if self.angleValid.get():
            targetAngle = self.angle.get()

            angleControllerOutput = self.angleController.calculate(
                -1 * targetAngle.radians(), 0
            )
        else:
            self.angleController.reset()

        self.drive.arcadeDriveWithFactors(
            self.forward(),
            self.sideways(),
            angleControllerOutput,
            DriveSubsystem.CoordinateMode.TargetRelative,
        )
