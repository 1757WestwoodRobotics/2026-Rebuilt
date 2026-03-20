from functools import partial
from commands2 import Command
from commands2 import cmd as Commands
from subsystems.intake.intakesubsystem import (
    IntakeSubsystem,
    IntakeSubsystemGoal,
    PivotGoal,
    RollerGoal,
)
from constants.intake import kPivotBumpAmount


def reverseIntake(intake: IntakeSubsystem) -> Command:
    return (
        Commands.run(
            partial(intake.setSubsystemGoal, IntakeSubsystemGoal.REVERSED), intake
        )
        .finallyDo(
            lambda _interrupted: intake.setSubsystemGoal(IntakeSubsystemGoal.EXTENDED)
        )
        .withName("ReverseIntake")
    )


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


def oscillateIntake(intake: IntakeSubsystem) -> Command:
    return (
        Commands.run(partial(intake.setPivotGoal, PivotGoal.OSCILLATE))
        .finallyDo(lambda _interrupted: intake.setPivotGoal(PivotGoal.DEPLOYED))
        .withName("OscillateIntake")
    )


def toggleIntakeDeployment(intake: IntakeSubsystem) -> Command:
    def toggle():
        if intake.pivotGoal == PivotGoal.DEPLOYED:
            intake.setPivotGoal(PivotGoal.RETRACTED)
        else:
            intake.setPivotGoal(PivotGoal.DEPLOYED)

    return Commands.runOnce(toggle, intake).withName("ToggleIntakeDeployment")


def runIntakeRollers(intake: IntakeSubsystem) -> Command:
    return (
        Commands.run(lambda: intake.setRollerGoal(RollerGoal.FORWARD), intake)
        .finallyDo(lambda _interrupted: intake.setRollerGoal(RollerGoal.NEUTRAL))
        .withName("RunIntakeRollers")
    )

def stopIntakeRollers(intake: IntakeSubsystem) -> Command:
    return (
        Commands.runOnce(lambda: intake.setRollerGoal(RollerGoal.NEUTRAL))
        .withName("StopIntakeRollers")
    )


def bumpIntakeDown(intake: IntakeSubsystem) -> Command:
    return Commands.runOnce(
        lambda: intake.bumpPivotFudge(kPivotBumpAmount), intake
    ).withName("BumpIntakeDown")


def bumpIntakeUp(intake: IntakeSubsystem) -> Command:
    return Commands.runOnce(
        lambda: intake.bumpPivotFudge(-kPivotBumpAmount), intake
    ).withName("BumpIntakeUp")


class _DepotIntake(Command):
    def __init__(self, intake: IntakeSubsystem) -> None:
        super().__init__()
        self.intake = intake
        self.prevGoal = intake.pivotGoal

        self.addRequirements(intake)

    def initialize(self) -> None:
        self.prevGoal = self.intake.pivotGoal

    def execute(self) -> None:
        self.intake.setPivotGoal(PivotGoal.DEPOT)

    def end(self, _interrupted: bool) -> None:
        self.intake.setPivotGoal(self.prevGoal)


def depotIntake(intake: IntakeSubsystem) -> Command:
    return _DepotIntake(intake).withName("DepotIntake")
