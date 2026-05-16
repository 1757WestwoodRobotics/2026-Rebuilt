from enum import Enum

from commands2 import Subsystem
from pykit.autolog import autologgable_output
from pykit.logger import Logger

from robotstate import RobotState
from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.logtracer import LogTracer

from constants.intake import kPivotDangerZoneStart
from constants.indexer import (
    kSpindexer1ForwardVoltage,
    kSpindexer1ReverseVoltage,
    kSpindexerReverseSlowVoltage,
    kKickUpperForwardVoltage,
    kKickUpperReverseVoltage,
)


class SpindexerMotorGoal(Enum):
    FORWARD = kSpindexer1ForwardVoltage
    NEUTRAL = 0.0
    REVERSE = kSpindexer1ReverseVoltage
    SLOWREVERSE = kSpindexerReverseSlowVoltage


class KickerMotorGoal(Enum):
    FORWARD = kKickUpperForwardVoltage
    NEUTRAL = 0.0
    REVERSE = kKickUpperReverseVoltage


class IndexerSubsystemGoal(Enum):
    KICK = (
        SpindexerMotorGoal.FORWARD,
        KickerMotorGoal.FORWARD,
    )
    HOLD = (
        SpindexerMotorGoal.SLOWREVERSE,
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
        spindexerMotorGoal = self.spindexerMotorGoal.value
        kickerMotorGoal = self.kickerMotorGoal.value

        if RobotState.intakeRotation.radians() > kPivotDangerZoneStart.radians():
            spindexerMotorGoal = SpindexerMotorGoal.NEUTRAL.value
            kickerMotorGoal = KickerMotorGoal.NEUTRAL.value

        self.io.setIndexerTarget(
            spindexerMotorGoal,
            kickerMotorGoal,
        )

        LogTracer.record("SetIndexerTarget")

        Logger.recordOutput("Indexer/Goal/Subsystem Goal", self.subsystemGoal.name)
        Logger.recordOutput(
            "Indexer/Goal/Spindexer Motor Goal", self.spindexerMotorGoal.value
        )
        Logger.recordOutput(
            "Indexer/Goal/Kicker Motor Goal", self.kickerMotorGoal.value
        )

        LogTracer.recordTotal()

    def setTarget(self, goal: IndexerSubsystemGoal) -> None:
        self.subsystemGoal = goal
        (
            self.spindexerMotorGoal,
            self.kickerMotorGoal,
        ) = goal.value
