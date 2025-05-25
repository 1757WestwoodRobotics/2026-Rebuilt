from commands2.command import Command
from wpilib import DataLogManager
from subsystems.drivesubsystem import DriveSubsystem


class DefenseState(Command):
    def __init__(self, drive: DriveSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive

        self.addRequirements(self.drive)

    def initialize(self) -> None:
        DataLogManager.log(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.drive.defenseState()

    def isFinished(self) -> bool:
        return True

    def end(self, _interrupted: bool) -> None:
        DataLogManager.log("... DONE")
