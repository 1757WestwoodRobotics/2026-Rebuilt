from enum import Enum, auto

from commands2 import Subsystem
from pykit.autolog import autologgable_output
from pykit.logger import Logger

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.logtracer import LogTracer

from constants.indexer import (
    kSpindexerForwardVoltage,
    kSpindexerReverseVoltage,
    kKickForwardVoltage,
    kKickReverseVoltage,
)


class IndexerMotorGoal(Enum):
    FORWARD = [kSpindexerForwardVoltage, kKickForwardVoltage]
    NEUTRAL = [0.0, 0.0]
    REVERSE = [kSpindexerReverseVoltage, kKickReverseVoltage]


class IndexerSubsystemGoal(Enum):
    KICK = IndexerMotorGoal.FORWARD
    HOLD = IndexerMotorGoal.NEUTRAL
    EJECT = IndexerMotorGoal.REVERSE


@autologgable_output
class IndexerSubsystem(Subsystem):
    def __init__(self, io: IndexerSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.io = io
        self.inputs = IndexerSubsystemIO.IndexerSubsystemInputs()

        self.spindexerMotorGoal = IndexerMotorGoal.NEUTRAL
        self.kickMotorGoal = IndexerMotorGoal.NEUTRAL

        self.subsystemGoal = IndexerSubsystemGoal.HOLD

    def periodic(self) -> None:
        LogTracer.resetOuter("IndexerSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Indexer", self.inputs)
        LogTracer.record("UpdateInputs")

        self.updateGoals()
        self.io.setIndexerTarget(self.indexerMotorGoal.value)
        LogTracer.record("SetIndexerTarget")

        Logger.recordOutput(
            "Indexer/Goal/Spindexer Motor Goal", self.spindexerMotorGoal.value
        )
        Logger.recordOutput("Indexer/Goal/Kick Motor Goal", self.kickMotorGoal.value)

        LogTracer.recordTotal()

    def setTarget(self, goal: IndexerSubsystemGoal) -> None:
        self.subsystemGoal = goal

    def updateGoals(self):
        self.indexerMotorGoal = self.subsystemGoal.value

    # def sysIdRoutine(self, subsystem: Subsystem) -> Command:

    #     def logState(state: State) -> None:
    #         loggedStateStr = ""
    #         match state:
    #             case State.kQuasistaticForward:
    #                 loggedStateStr = "quasistatic-forward"
    #             case State.kQuasistaticReverse:
    #                 loggedStateStr = "quasistatic-reverse"
    #             case State.kDynamicForward:
    #                 loggedStateStr = "dynamic-forward"
    #             case State.kDynamicReverse:
    #                 loggedStateStr = "dynamic-reverse"
    #             case State.kNone:
    #                 loggedStateStr = "none"
    #         Logger.recordOutput("Indexer/SysID State", loggedStateStr)

    #     charactarizationRoutine = SysIdRoutine(
    #         SysIdRoutine.Config(0.5, 6, 10, logState),
    #         SysIdRoutine.Mechanism(
    #             self.io.setIndexerTarget,
    #             (lambda _: None),
    #             subsystem,
    #         ),
    #     )

    #     return cmd.sequence(
    #         charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
    #         charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
    #         charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward),
    #         charactarizationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
    #     )
