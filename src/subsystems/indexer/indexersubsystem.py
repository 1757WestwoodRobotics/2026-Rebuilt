from enum import Enum, auto

from commands2 import Subsystem
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.logtracer import LogTracer

from constants.indexer import (
    kIndexerForwardVoltage,
    kIndexerReverseVoltage,
    kFeedForwardVoltage,
    kFeedReverseVoltage,
)


class IndexerMotorGoal(Enum):
    FORWARD = kIndexerForwardVoltage
    NEUTRAL = 0.0
    REVERSE = kIndexerReverseVoltage


class FeedMotorGoal(Enum):
    FORWARD = kFeedForwardVoltage
    NEUTRAL = 0.0
    REVERSE = kFeedReverseVoltage


class IndexerSubsystemGoal(Enum):
    FEED = (
        IndexerMotorGoal.FORWARD,
        FeedMotorGoal.FORWARD,
    )
    SINGLE_HOLD = (
        IndexerMotorGoal.FORWARD,
        FeedMotorGoal.REVERSE,
    )
    DUAL_HOLD = (
        IndexerMotorGoal.NEUTRAL,
        FeedMotorGoal.NEUTRAL,
    )
    REVERSE = (
        IndexerMotorGoal.REVERSE,
        FeedMotorGoal.REVERSE,
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

        self.indexerMotorGoal = IndexerMotorGoal.NEUTRAL
        self.feedMotorGoal = FeedMotorGoal.NEUTRAL

        self.subsystemTarget = IndexerSubsystemTarget.HOLDING
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
                self.subsystemGoal = IndexerSubsystemGoal.FEED
            case IndexerSubsystemTarget.EJECT:
                self.subsystemGoal = IndexerSubsystemGoal.REVERSE

        self.updateGoals()
        self.io.setIndexerTarget(self.indexerMotorGoal.value, self.feedMotorGoal.value)
        LogTracer.record("SetIndexerTarget")

        Logger.recordOutput(
            "Indexer/Goal/Indexer Motor Goal", self.indexerMotorGoal.value
        )
        Logger.recordOutput("Indexer/Goal/Feed Motor Goal", self.feedMotorGoal.value)
        Logger.recordOutput("Indexer/Sensor/Indexer", self.inputs.indexerSensor)
        Logger.recordOutput("Indexer/Sensor/Feed", self.inputs.feedSensor)

        LogTracer.recordTotal()

    def setTarget(self, target: IndexerSubsystemTarget) -> None:
        self.subsystemTarget = target

    def updateGoals(self):
        self.indexerMotorGoal, self.feedMotorGoal = self.subsystemGoal.value

    @autolog_output(key="Indexer/has ball")
    def hasBall(self) -> bool:
        return self.inputs.indexerSensor or self.inputs.feedSensor

    @autolog_output(key="Indexer/has two ball")
    def hasTwoBall(self) -> bool:
        return self.inputs.indexerSensor and self.inputs.feedSensor
