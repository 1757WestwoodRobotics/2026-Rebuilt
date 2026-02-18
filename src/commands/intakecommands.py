from functools import partial
from commands2 import Command
from commands2 import cmd as Commands
from subsystems.intake.intakesubsystem import IntakeSubsystem, IntakeSubsystemGoal


class IntakeCommands:
    @staticmethod
    def reverseIntake(intake: IntakeSubsystem) -> Command:
        return Commands.run(
            partial(intake.setSubsystemGoal, IntakeSubsystemGoal.REVERSED), intake
        ).withName("ReverseIntake")

    @staticmethod
    def deployIntake(intake: IntakeSubsystem) -> Command:
        return Commands.run(
            partial(intake.setSubsystemGoal, IntakeSubsystemGoal.DEPLOYED), intake
        ).withName("DeployIntake")

    @staticmethod
    def retractIntake(intake: IntakeSubsystem) -> Command:
        return Commands.run(
            partial(intake.setSubsystemGoal, IntakeSubsystemGoal.RETRACTED), intake
        ).withName("RetractIntake")
