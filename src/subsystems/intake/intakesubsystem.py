from enum import Enum
from functools import partial
from commands2 import Subsystem
from commands2.command import Command
import commands2.cmd as Commands
from commands2.sysid import SysIdRoutine
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib.sysid import State
from wpimath.geometry import Rotation2d

from subsystems.intake.intakesubsystemio import IntakeSubsystemIO

from constants.intake import (
    kPivotRetractedPosition,
    kPivotExtendedPosition,
    kRollerForwardVoltage,
    kRollerReverseVoltage,
    kPivotTolerance,
)
from util.logtracer import LogTracer


class PivotGoal(Enum):
    DEPLOYED = kPivotExtendedPosition
    RETRACTED = kPivotRetractedPosition


class RollerGoal(Enum):
    FORWARD = kRollerForwardVoltage
    NEUTRAL = 0
    REVERSE = kRollerReverseVoltage


class IntakeSubsystemGoal(Enum):
    DEPLOYED = (
        PivotGoal.DEPLOYED,
        RollerGoal.FORWARD,
    )
    RETRACTED = (
        PivotGoal.RETRACTED,
        RollerGoal.NEUTRAL,
    )
    REVERSED = (
        PivotGoal.DEPLOYED,
        RollerGoal.REVERSE,
    )


@autologgable_output
class IntakeSubsystem(Subsystem):

    def __init__(self, io: IntakeSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.io = io
        self.inputs = IntakeSubsystemIO.IntakeSubsystemInputs()

        self.isClosedLoop = True
        self.pivotGoal = PivotGoal.RETRACTED
        self.rollerGoal = RollerGoal.NEUTRAL

    def periodic(self) -> None:
        LogTracer.resetOuter("IntakeSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Intake", self.inputs)
        LogTracer.record("UpdateInputs")

        if self.isClosedLoop:
            self.io.setIntakeTarget(self.rollerGoal.value, self.pivotGoal.value)
        LogTracer.record("SetIntakeTarget")

        Logger.recordOutput("Intake/Roller Goal", self.rollerGoal.name)
        Logger.recordOutput("Intake/Pivot Goal", self.pivotGoal.name)
        LogTracer.recordTotal()

    def setSubsystemGoal(self, goal: IntakeSubsystemGoal):
        self.pivotGoal, self.rollerGoal = goal.value

    def setRollerGoal(self, goal: RollerGoal):
        self.rollerGoal = goal

    def setPivotGoal(self, goal: PivotGoal):
        self.pivotGoal = goal

    @autolog_output(key="Intake/at target")
    def isAtTarget(self) -> bool:
        return (
            abs(self.inputs.pivotPosition - self.pivotGoal.value.radians())
            < kPivotTolerance.radians()
        )

    def isAtGoal(self, goal: Rotation2d) -> bool:
        return (
            abs(self.inputs.pivotPosition - goal.radians()) < kPivotTolerance.radians()
        )

    def setOpenLoop(self) -> None:
        self.isClosedLoop = False

    def sysIdRoutine(self, subsystem: Subsystem) -> Command:
        def logState(state: State) -> None:
            loggedStateStr = ""
            match state:
                case State.kQuasistaticForward:
                    loggedStateStr = "quasistatic-forward"
                case State.kQuasistaticReverse:
                    loggedStateStr = "quasistatic-reverse"
                case State.kDynamicForward:
                    loggedStateStr = "dynamic-forward"
                case State.kDynamicReverse:
                    loggedStateStr = "dynamic-reverse"
                case State.kNone:
                    loggedStateStr = "none"
            Logger.recordOutput("Intake/SysID State", loggedStateStr)

        charactarizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(0.1, 6, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.setPivotVolts,
                (lambda _: None),
                subsystem,
            ),
        )

        return Commands.sequence(
            Commands.runOnce(self.setOpenLoop, self),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(
                partial(self.isAtGoal, PivotGoal.DEPLOYED.value)
            ),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(
                partial(self.isAtGoal, PivotGoal.RETRACTED.value)
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(
                partial(self.isAtGoal, PivotGoal.DEPLOYED.value)
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(
                partial(self.isAtGoal, PivotGoal.RETRACTED.value)
            ),
        )
