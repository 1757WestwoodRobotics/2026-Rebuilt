from dataclasses import dataclass, field
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d


class TurretSubsystemIO:
    """Serve as a template for specific IO classes (e.g., Talon, Sim, etc.)."""

    @autolog
    @dataclass
    class TurretSubsystemIOInputs:
        """Initialize required turret motor controls."""

        turretConnected: bool = False

        turretPosition: Rotation2d = field(default_factory=Rotation2d)
        turretSpeed: float = 0.0

        turretAppliedVolts: float = 0.0
        turretSupplyAmps: float = 0.0

    def updateInputs(self, inputs: TurretSubsystemIOInputs):
        pass

    def set_turret_angle(self, position: Rotation2d):
        """
        sets the turret angle in radians
        """

    def set_turret_volts(self, volts: float):
        """
        sets the turret voltage in volts
        """

    def set_turret_position(self, position: Rotation2d):
        """
        sets the current position of the turret in radians
        used for startup, or "re-zeroing"
        """
