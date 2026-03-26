from dataclasses import dataclass, field
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d


class IntakeSubsystemIO:

    @autolog
    @dataclass
    class IntakeSubsystemIOInputs:
        pivotConnected: bool = False
        rollerConnected: bool = False

        pivotPosition: Rotation2d = field(default_factory=Rotation2d)
        pivotVelocity: float = 0  # rad / s

        pivotAppliedVolts: float = 0
        pivotSupplyAmps: float = 0

        rollerPosition: float = 0  # rad
        rollerVelocity: float = 0  # rad / s

        rollerAppliedVolts: float = 0
        rollerSupplyAmps: float = 0

    def updateInputs(self, inputs: IntakeSubsystemIOInputs) -> None:
        pass

    def setIntakeAngle(self, position: Rotation2d) -> None:
        """
        Sets the current angle for the intake pivot
        Useful for initialization
        """

    def setIntakeTarget(self, roller: float, position: Rotation2d) -> None:
        """
        Sets the target for the intake
        Roller is in volts, position is a rotation
        """

    def setPivotVolts(self, volt: float) -> None:
        """
        Sets the pivot motor to a voltage, for open loop control
        """

    def setMaxVel(self, vel: float) -> None:
        """
        Sets the maximum velocity of the pivot in radians per second
        """

    def setMaxAccel(self, accel: float) -> None:
        """
        Sets the maximum acceleration of the pivot in radians per second squared
        """
