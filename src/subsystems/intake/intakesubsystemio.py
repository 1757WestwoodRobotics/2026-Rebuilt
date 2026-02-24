from dataclasses import dataclass
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d


class IntakeSubsystemIO:

    @autolog
    @dataclass
    class IntakeSubsystemInputs:
        pivotConnected: bool = False
        rollerConnected: bool = False

        pivotPosition: float = 0  # rad
        pivotVelocity: float = 0  # rad / s

        pivotAppliedVolts: float = 0
        pivotSupplyAmps: float = 0

        rollerPosition: float = 0  # rad
        rollerVelocity: float = 0  # rad / s

        rollerAppliedVolts: float = 0
        rollerSupplyAmps: float = 0

    def updateInputs(self, inputs: IntakeSubsystemInputs):
        pass

    def setIntakeTarget(self, roller: float, position: Rotation2d) -> None:
        """
        Sets the target for the intake
        Roller is in volts, position is a rotation
        """
