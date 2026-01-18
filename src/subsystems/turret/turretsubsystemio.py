from dataclasses import dataclass
from pykit.autolog import autolog


class TurretSubsystemIO:
    """Serve as a template for specific IO classes (e.g., Talon, Sim, etc.)."""

    @autolog
    @dataclass
    class TurretSubsystemIOInputs:
        """Initialize required turret motor controls."""

        turretConnected: bool = False

        turretPosition: float = 0.0
        turretSpeed: float = 0.0

        turretAppliedVolts: float = 0.0
        turretSupplyAmps: float = 0.0

    def updateInputs(self, inputs: TurretSubsystemIOInputs):
        pass

    def set_turret_angle(self, position: float):
        # sets the turret angle in radians
        pass

    def set_turret_volts(self, volts: float):
        # sets the turret voltage in volts
        pass
