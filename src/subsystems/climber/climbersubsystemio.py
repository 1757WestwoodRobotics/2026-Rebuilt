from dataclasses import dataclass
from pykit.autolog import autolog


class ClimberSubsystemIO:
    @autolog
    @dataclass
    class ClimberSubsystemIOInputs:
        """Initialize required climber motor controls."""

        climberConnected: bool = False

        climberPosition: float = 0.0
        climberSpeed: float = 0.0

        climberAppliedVolts: float = 0.0
        climberSupplyAmps: float = 0.0
        climberTorqueAmps: float = 0.0

    def updateInputs(self, inputs: ClimberSubsystemIOInputs):
        pass

    def set_climber_position(self, position: float):
        # sets the climber position in meters
        pass

    def set_climber_volts(self, volts: float):
        # sets the climber voltage in volts
        pass
