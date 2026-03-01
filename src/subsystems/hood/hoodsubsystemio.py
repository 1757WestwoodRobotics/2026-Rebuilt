from dataclasses import dataclass, field
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d


class HoodSubsystemIO:
    @autolog
    @dataclass
    class HoodSubsystemIOInputs:
        hoodConnected: bool = False

        hoodPosition: Rotation2d = field(default_factory=Rotation2d)
        hoodSpeed: float = 0.0

        hoodAppliedVolts: float = 0.0
        hoodSupplyAmps: float = 0.0

    def updateInputs(self, inputs: HoodSubsystemIOInputs):
        pass

    def set_hood_position(self, position: float):
        # Code to set the hood position in radians
        pass

    def set_hood_volts(self, volts: float):
        # Code to set the hood voltage in volts
        pass