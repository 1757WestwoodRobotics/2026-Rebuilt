from wpilib import DataLogManager
from commands2.command import Command
from wpimath.geometry import Pose2d

from robotstate import RobotState
from subsystems.drive.drivesubsystem import DriveSubsystem


class ResetGyro(Command):
    def __init__(self, drive: DriveSubsystem, position: Pose2d = Pose2d()) -> None:
        Command.__init__(self)
        self.setName(type(self).__name__)
        self.drive = drive
        self.position = position
        self.addRequirements(self.drive)

    def initialize(self) -> None:
        DataLogManager.log(f"Command: {self.getName()}")

    def execute(self) -> None:
        RobotState.resetPose()

    def end(self, _interrupted: bool) -> None:
        DataLogManager.log("... DONE")

    def isFinished(self) -> bool:
        return True
