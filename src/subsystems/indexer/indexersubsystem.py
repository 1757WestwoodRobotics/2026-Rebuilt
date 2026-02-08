from enum import Enum, auto

from commands2 import Subsystem
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.logtracer import LogTracer

from constants.indexer import (
    kSpindexerForwardVoltage,
    kSpindexerReverseVoltage,
    kKickForwardVoltage,
    kKickReverseVoltage,
)


class SpindexerMotorGoal(Enum):
    FORWARD = kSpindexerForwardVoltage
    NEUTRAL = 0.0
    REVERSE = kSpindexerReverseVoltage


class KickMotorGoal(Enum):
    FORWARD = kKickForwardVoltage
    NEUTRAL = 0.0
    REVERSE = kKickReverseVoltage


class IndexerSubsystemGoal(Enum):
    KICK = (
        SpindexerMotorGoal.FORWARD,
        KickMotorGoal.FORWARD,
    )
    REVERSE = (
        SpindexerMotorGoal.REVERSE,
        KickMotorGoal.REVERSE,
    )
    HOLD = (
        SpindexerMotorGoal.NEUTRAL,
        KickMotorGoal.NEUTRAL,
    )


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

        self.spindexerMotorGoal = SpindexerMotorGoal.NEUTRAL
        self.kickMotorGoal = KickMotorGoal.NEUTRAL

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
