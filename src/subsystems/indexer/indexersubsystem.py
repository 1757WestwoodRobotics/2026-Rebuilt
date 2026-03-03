from enum import Enum, auto

from commands2 import Subsystem
from pykit.autolog import autologgable_output
from pykit.logger import Logger

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.logtracer import LogTracer

from constants.indexer import (
    kSpindexer1ForwardVoltage,
    kSpindexer1ReverseVoltage,
    kSpindexer2ForwardVoltage,
    kSpindexer2ReverseVoltage,
    kKickLowerForwardVoltage,
    kKickLowerReverseVoltage,
    kKickUpperForwardVoltage,
    kKickUpperReverseVoltage,
)


class Spindexer1MotorGoal(Enum):
    FORWARD = kSpindexer1ForwardVoltage
    NEUTRAL = 0
    REVERSE = kSpindexer1ReverseVoltage


class Spindexer2MotorGoal(Enum):
    # wanted to make two even though they are the exact same so as to be able to change them later
    FORWARD = kSpindexer2ForwardVoltage
    NEUTRAL = 0
    REVERSE = kSpindexer2ReverseVoltage


class KickUpperMotorGoal(Enum):
    FORWARD = kKickUpperForwardVoltage
    NEUTRAL = 0
    REVERSE = kKickUpperReverseVoltage


class KickLowerMotorGoal(Enum):
    FORWARD = kKickLowerForwardVoltage
    NEUTRAL = 0
    REVERSE = kKickLowerReverseVoltage


class IndexerSubsystemGoal(Enum):
    KICK = (
        Spindexer1MotorGoal.FORWARD,
        Spindexer2MotorGoal.FORWARD,
        KickUpperMotorGoal.FORWARD,
        KickUpperMotorGoal.FORWARD,
    )
    HOLD = (
        Spindexer1MotorGoal.NEUTRAL,
        Spindexer2MotorGoal.NEUTRAL,
        KickUpperMotorGoal.NEUTRAL,
        KickUpperMotorGoal.NEUTRAL,
    )
    EJECT = (
        Spindexer1MotorGoal.REVERSE,
        Spindexer2MotorGoal.REVERSE,
        KickUpperMotorGoal.REVERSE,
        KickUpperMotorGoal.REVERSE,
    )


@autologgable_output
class IndexerSubsystem(Subsystem):
    def __init__(self, io: IndexerSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.io = io
        self.inputs = IndexerSubsystemIO.IndexerSubsystemInputs()

        self.spindexer1MotorGoal = Spindexer1MotorGoal.NEUTRAL
        self.spindexer2MotorGoal = Spindexer2MotorGoal.NEUTRAL
        self.kickLowerMotorGoal = KickLowerMotorGoal.NEUTRAL
        self.kicUpperMotorGoal = KickUpperMotorGoal.NEUTRAL

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
