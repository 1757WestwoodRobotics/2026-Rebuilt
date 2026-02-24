from dataclasses import dataclass

from pykit.autolog import autolog


class IndexerSubsystemIO:

    @autolog
    @dataclass
    class IndexerSubsystemInputs:
        spindexerConnected: bool = False
        kickConnected: bool = False

        spindexPosition: float = 0  # radians
        indexVelocity: float = 0  # rad / s

        kickPosition: float = 0  # rad
        kickVelocity: float = 0  # rad / s

        spindexAppliedVolts: float = 0
        spindexSupplyAmps: float = 0

        kickAppliedVolts: float = 0
        kickSupplyAmps: float = 0

    def updateInputs(self, inputs: IndexerSubsystemInputs):
        pass

    def setIndexerTarget(self, spindexer: float, kicker: float):
        """
        Sets the target for the indexer
        indexer and kicker are in volts
        """
