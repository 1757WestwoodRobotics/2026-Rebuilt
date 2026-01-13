from commands2.command import Command

from subsystems.drive.drivesubsystem import DriveSubsystem


class DefenseState(Command):
    def __init__(self, drive: DriveSubsystem) -> None:
        Command.__init__(self)
        self.setName(type(self).__name__)
        self.drive = drive

        self.addRequirements(self.drive)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.drive.defenseState()

    def isFinished(self) -> bool:
        return True

    def end(self, _interrupted: bool) -> None:
        print("... DONE")
