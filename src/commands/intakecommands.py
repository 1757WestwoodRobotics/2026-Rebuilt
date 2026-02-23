from functools import partial
from commands2 import Command
from commands2 import cmd as Commands
from subsystems.intake.intakesubsystem import (
    IntakeSubsystem,
    IntakeSubsystemGoal,
    PivotGoal,
    RollerGoal,
)


def reverseIntake(intake: IntakeSubsystem) -> Command:
    return Commands.run(
        partial(intake.setSubsystemGoal, IntakeSubsystemGoal.REVERSED), intake
    ).withName("ReverseIntake")


def deployIntake(intake: IntakeSubsystem) -> Command:
    return Commands.run(
        partial(intake.setSubsystemGoal, IntakeSubsystemGoal.DEPLOYED), intake
    ).withName("DeployIntake")


def extendIntake(intake: IntakeSubsystem) -> Command:
    return Commands.run(
        partial(intake.setSubsystemGoal, IntakeSubsystemGoal.EXTENDED), intake
    ).withName("ExtendIntake")


def retractIntake(intake: IntakeSubsystem) -> Command:
    return Commands.run(
        partial(intake.setSubsystemGoal, IntakeSubsystemGoal.RETRACTED), intake
    ).withName("RetractIntake")


def toggleIntakeDeployment(intake: IntakeSubsystem) -> Command:
    def toggle():
        if intake.pivotGoal == PivotGoal.RETRACTED:
            intake.setPivotGoal(PivotGoal.DEPLOYED)
        else:
            intake.setPivotGoal(PivotGoal.RETRACTED)

    return Commands.run(toggle, intake).withName("ToggleIntakeDeployment")


def runIntakeRollers(intake: IntakeSubsystem) -> Command:
    return (
        Commands.run(lambda: intake.setRollerGoal(RollerGoal.FORWARD), intake)
        .finallyDo(lambda _interrupted: intake.setRollerGoal(RollerGoal.NEUTRAL))
        .withName("RunIntakeRollers")
    )
