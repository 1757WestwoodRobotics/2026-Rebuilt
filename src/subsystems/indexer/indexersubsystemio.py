from dataclasses import dataclass

from pykit.autolog import autolog


class IndexerSubsystemIO:

    @autolog
    @dataclass
    class IndexerSubsystemInputs:
        indexerConnected: bool = False
        feedConnected: bool = False

        indexPosition: float = 0  # radians
        indexVelocity: float = 0  # rad / s

        feedPosition: float = 0  # rad
        feedVelocity: float = 0  # rad / s

        indexAppliedVolts: float = 0
        indexSupplyAmps: float = 0

        feedAppliedVolts: float = 0
        feedSupplyAmps: float = 0

        indexerSensor: bool = False
        feedSensor: bool = False

    def updateInputs(self, inputs: IndexerSubsystemInputs):
        pass

    def setIndexerTarget(self, indexer: float, feeder: float):
        """
        Sets the target for the indexer
        indexer and feeder are in volts
        """
