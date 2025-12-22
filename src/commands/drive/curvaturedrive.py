"""curvature drive, aka chezy drive
maintains a constant curvature for a given rotation amount"""

import typing
from commands2 import Command

from subsystems.drive.drivesubsystem import DriveSubsystem


class CurvatureDrive(Command):  # Arcade drive is just robot relative, but no sideways
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        rotation: typing.Callable[[], float],
    ) -> None:
        Command.__init__(self)

        self.drive = drive
        self.forward = forward
        self.rotation = rotation

        self.addRequirements(self.drive)
        self.setName(type(self).__name__)

    def execute(self) -> None:
        self.drive.arcadeDriveWithFactors(
            self.forward(),
            0,
            self.rotation() * abs(self.forward()),
            DriveSubsystem.CoordinateMode.RobotRelative,
        )
