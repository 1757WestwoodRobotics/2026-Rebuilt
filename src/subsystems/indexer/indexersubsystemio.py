from dataclasses import dataclass

from pykit.autolog import autolog


class IndexerSubsystemIO:

    @autolog
    @dataclass
    class IndexerSubsystemInputs:
        indexerConnected: bool = False
        kickConnected: bool = False

        indexPosition: float = 0  # radians
        indexVelocity: float = 0  # rad / s

        kickPosition: float = 0  # rad
        kickVelocity: float = 0  # rad / s

        indexAppliedVolts: float = 0
        indexSupplyAmps: float = 0

        kickAppliedVolts: float = 0
        kickSupplyAmps: float = 0

        indexerSensor: bool = False
        kickSensor: bool = False

    def updateInputs(self, inputs: IndexerSubsystemInputs):
        pass

    def setIndexerTarget(self, indexer: float, kicker: float):
        """
        Sets the target for the indexer
        indexer and kicker are in volts
        """
