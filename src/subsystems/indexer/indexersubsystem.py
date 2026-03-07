from enum import Enum

from commands2 import Subsystem
from pykit.autolog import autologgable_output
from pykit.logger import Logger

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.logtracer import LogTracer

from constants.indexer import (
    kSpindexer1ForwardVoltage,
    kSpindexer1ReverseVoltage,
    kKickUpperForwardVoltage,
    kKickUpperReverseVoltage,
)


class SpindexerMotorGoal(Enum):
    FORWARD = kSpindexer1ForwardVoltage
    NEUTRAL = 0
    REVERSE = kSpindexer1ReverseVoltage


class KickerMotorGoal(Enum):
    FORWARD = kKickUpperForwardVoltage
    NEUTRAL = 0
    REVERSE = kKickUpperReverseVoltage


class IndexerSubsystemGoal(Enum):
    KICK = (
        SpindexerMotorGoal.FORWARD,
        KickerMotorGoal.FORWARD,
    )
    HOLD = (
        SpindexerMotorGoal.NEUTRAL,
        KickerMotorGoal.NEUTRAL,
    )
    EJECT = (
        SpindexerMotorGoal.REVERSE,
        KickerMotorGoal.REVERSE,
    )


@autologgable_output
class IndexerSubsystem(Subsystem):
    def __init__(self, io: IndexerSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.io = io
        self.inputs = IndexerSubsystemIO.IndexerSubsystemInputs()

        self.spindexerMotorGoal = SpindexerMotorGoal.NEUTRAL
        self.kickerMotorGoal = KickerMotorGoal.NEUTRAL

        self.subsystemGoal = IndexerSubsystemGoal.HOLD

    def periodic(self) -> None:
        LogTracer.resetOuter("IndexerSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Indexer", self.inputs)
        LogTracer.record("UpdateInputs")

        self.io.setIndexerTarget(
            self.spindexerMotorGoal.value,
            self.kickerMotorGoal.value,
        )

        LogTracer.record("SetIndexerTarget")

        Logger.recordOutput(
            "Indexer/Goal/Spindexer Motor Goal", self.spindexerMotorGoal.value
        )
        Logger.recordOutput(
            "Indexer/Goal/Kicker Motor Goal", self.kickerMotorGoal.value
        )

        LogTracer.recordTotal()

    def setTarget(self, goal: IndexerSubsystemGoal) -> None:
        (
            self.spindexerMotorGoal,
            self.kickerMotorGoal,
        ) = goal.value
