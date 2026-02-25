from enum import Enum, auto

from commands2 import Subsystem, Command, cmd
from commands2.sysid import SysIdRoutine
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib.sysid import State

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.logtracer import LogTracer

from constants.indexer import (
    kIndexerForwardVoltage,
    kIndexerReverseVoltage,
)


class IndexerMotorGoal(Enum):
    FORWARD = kIndexerForwardVoltage
    NEUTRAL = 0.0
    REVERSE = kIndexerReverseVoltage


class IndexerSubsystemGoal(Enum):
    KICK = IndexerMotorGoal.FORWARD
    HOLD = IndexerMotorGoal.NEUTRAL
    EJECT = IndexerMotorGoal.REVERSE


class IndexerSubsystemTarget(Enum):
    KICK = auto()
    HOLD = auto()
    EJECT = auto()


@autologgable_output
class IndexerSubsystem(Subsystem):
    def __init__(self, io: IndexerSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.io = io
        self.inputs = IndexerSubsystemIO.IndexerSubsystemInputs()

        self.indexerMotorGoal = IndexerMotorGoal.NEUTRAL

        self.subsystemTarget = IndexerSubsystemTarget.HOLD
        self.subsystemGoal = IndexerSubsystemGoal.HOLD

    def periodic(self) -> None:
        LogTracer.resetOuter("IndexerSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Indexer", self.inputs)
        LogTracer.record("UpdateInputs")

        match self.subsystemTarget:
            case IndexerSubsystemTarget.HOLD:
                self.subsystemGoal = IndexerSubsystemGoal.HOLD
            case IndexerSubsystemTarget.KICK:
                self.subsystemGoal = IndexerSubsystemGoal.KICK
            case IndexerSubsystemTarget.EJECT:
                self.subsystemGoal = IndexerSubsystemGoal.EJECT

        self.updateGoals()
        self.io.setIndexerTarget(self.indexerMotorGoal.value)
        LogTracer.record("SetIndexerTarget")

        Logger.recordOutput(
            "Indexer/Goal/Indexer Motor Goal", self.indexerMotorGoal.value
        )

        LogTracer.recordTotal()

    def setTarget(self, target: IndexerSubsystemTarget) -> None:
        self.subsystemTarget = target

    def updateGoals(self):
        self.indexerMotorGoal = self.subsystemGoal.value

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
