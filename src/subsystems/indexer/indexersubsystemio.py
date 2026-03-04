from dataclasses import dataclass

from pykit.autolog import autolog


class IndexerSubsystemIO:

    @autolog
    @dataclass
    class IndexerSubsystemInputs:
        spindexer1Connected: bool = False
        spindexer2Connected: bool = False
        kickLowerConnected: bool = False
        kickUpperConnected: bool = False

        spindexer1Position: float = 0  # radians
        spindexer2Position: float = 0
        spindex1Velocity: float = 0  # rad / s
        spindex2Velocity: float = 0

        kickLowerPosition: float = 0  # rad
        kickUpperPosition: float = 0
        kickLowerVelocity: float = 0  # rad / s
        kickUpperVelocity: float = 0

        spindex1AppliedVolts: float = 0
        spindex2AppliedVolts: float = 0
        spindex1SupplyAmps: float = 0
        spindex2SupplyAmps: float = 0

        kickLowerAppliedVolts: float = 0
        kickUpperAppliedVolts: float = 0
        kickLowerSupplyAmps: float = 0
        kickUpperSupplyAmps: float = 0

    def updateInputs(self, inputs: IndexerSubsystemInputs):
        pass

    def setIndexerTarget(self, spindexer: float, kicker: float):
        """
        Sets the target for the indexer
        indexer and kicker are in volts
        """
