from enum import Enum, auto

from commands2 import Subsystem, Command, cmd
from commands2.sysid import SysIdRoutine
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib.sysid import State

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.logtracer import LogTracer

from constants.indexer import (
    kSpindexerForwardVoltage,
    kSpindexerReverseVoltage,
    kKickForwardVoltage,
    kKickReverseVoltage,
)


class IndexerMotorGoal(Enum):
    FORWARD = kSpindexerForwardVoltage, kKickForwardVoltage
    NEUTRAL = 0.0
    REVERSE = kSpindexerReverseVoltage, kKickReverseVoltage


class IndexerSubsystemGoal(Enum):
    KICK = IndexerMotorGoal.FORWARD
    REVERSE = IndexerMotorGoal.REVERSE
    HOLD = IndexerMotorGoal.NEUTRAL


class IndexerSubsystemTarget(Enum):
    HOLDING = auto()
    SHOOT = auto()
    EJECT = auto()


@autologgable_output
class IndexerSubsystem(Subsystem):
    def __init__(self, io: IndexerSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.io = io
        self.inputs = IndexerSubsystemIO.IndexerSubsystemInputs()

        self.indexerMotorGoal = IndexerMotorGoal.NEUTRAL

        self.subsystemTarget = IndexerSubsystemTarget.HOLDING
        self.subsystemGoal = IndexerSubsystemGoal.HOLD

    def periodic(self) -> None:
        LogTracer.resetOuter("IndexerSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Indexer", self.inputs)
        LogTracer.record("UpdateInputs")

        match self.subsystemTarget:
            case IndexerSubsystemTarget.HOLDING:
                self.subsystemGoal = IndexerSubsystemGoal.HOLD
            case IndexerSubsystemTarget.SHOOT:
                self.subsystemGoal = IndexerSubsystemGoal.KICK
            case IndexerSubsystemTarget.EJECT:
                self.subsystemGoal = IndexerSubsystemGoal.REVERSE

        self.updateGoals()
        self.io.setIndexerTarget(
            self.spindexerMotorGoal.value, self.kickMotorGoal.value
        )
        LogTracer.record("SetIndexerTarget")

        Logger.recordOutput(
            "Indexer/Goal/Indexer Motor Goal", self.spindexerMotorGoal.value
        )
        Logger.recordOutput("Indexer/Goal/Kick Motor Goal", self.kickMotorGoal.value)

        LogTracer.recordTotal()

    def setTarget(self, target: IndexerSubsystemTarget) -> None:
        self.subsystemTarget = target

    def updateGoals(self):
        self.spindexerMotorGoal, self.kickMotorGoal = self.subsystemGoal.value

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
            Logger.recordOutput("Indexer/SysID State", loggedStateStr)

        charactarizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(0.5, 6, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.setIndexerTarget,
                (lambda _: None),
                subsystem,
            ),
        )

        return cmd.sequence(
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        )
