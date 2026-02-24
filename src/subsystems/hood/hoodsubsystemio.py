from dataclasses import dataclass
from pykit.autolog import autolog


class HoodSubsystemIO:
    @autolog
    @dataclass
    class HoodSubsystemIOInputs:
        hoodConnected: bool = False

        hoodPosition: float = 0.0
        hoodSpeed: float = 0.0

        hoodAppliedVolts: float = 0.0
        hoodSupplyAmps: float = 0.0

#TODO
    def updateInputs(self, inputs: HoodSubsystemIOInputs):
        pass

    def set_hood_position(self, position: float):
        # Code to set the hood position in radians
        pass

    def set_hood_volts(self, volts: float):
        # Code to set the hood voltage in volts
        pass
