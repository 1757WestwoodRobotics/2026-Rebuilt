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
    SINGLE_HOLD = (
        SpindexerMotorGoal.FORWARD,
        KickMotorGoal.REVERSE,
    )
    DUAL_HOLD = (
        SpindexerMotorGoal.NEUTRAL,
        SpindexerMotorGoal.NEUTRAL,
    )
    REVERSE = (
        SpindexerMotorGoal.REVERSE,
        KickMotorGoal.REVERSE,
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

        self.indexerMotorGoal = SpindexerMotorGoal.NEUTRAL
        self.kickMotorGoal = KickMotorGoal.NEUTRAL

        self.subsystemTarget = SpindexerSubsystemTarget.HOLDING
        self.subsystemGoal = IndexerSubsystemGoal.SINGLE_HOLD

    def periodic(self) -> None:
        LogTracer.resetOuter("IntakeSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Intake", self.inputs)
        LogTracer.record("UpdateInputs")

        match self.subsystemTarget:
            case IndexerSubsystemTarget.HOLDING:
                if self.hasTwoBall():
                    self.subsystemGoal = IndexerSubsystemGoal.DUAL_HOLD
                else:
                    self.subsystemGoal = IndexerSubsystemGoal.SINGLE_HOLD
            case IndexerSubsystemTarget.SHOOT:
                self.subsystemGoal = IndexerSubsystemGoal.KICK
            case IndexerSubsystemTarget.EJECT:
                self.subsystemGoal = IndexerSubsystemGoal.REVERSE

        self.updateGoals()
        self.io.setIndexerTarget(self.indexerMotorGoal.value, self.kickMotorGoal.value)
        LogTracer.record("SetIndexerTarget")

        Logger.recordOutput(
            "Indexer/Goal/Indexer Motor Goal", self.indexerMotorGoal.value
        )
        Logger.recordOutput("Indexer/Goal/Kick Motor Goal", self.kickMotorGoal.value)
        Logger.recordOutput("Indexer/Sensor/Indexer", self.inputs.indexerSensor)
        Logger.recordOutput("Indexer/Sensor/Kick", self.inputs.kickSensor)

        LogTracer.recordTotal()

    def setTarget(self, target: IndexerSubsystemTarget) -> None:
        self.subsystemTarget = target

    def updateGoals(self):
        self.indexerMotorGoal, self.kickMotorGoal = self.subsystemGoal.value

    @autolog_output(key="Indexer/has ball")
    def hasBall(self) -> bool:
        return self.inputs.indexerSensor or self.inputs.kickSensor

    @autolog_output(key="Indexer/has two ball")
    def hasTwoBall(self) -> bool:
        return self.inputs.indexerSensor and self.inputs.kickSensor
