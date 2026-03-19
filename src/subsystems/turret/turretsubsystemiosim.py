from phoenix6.signals import InvertedValue
from pykit.logger import Logger
from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.turret.turretsubsystemio import TurretSubsystemIO
from subsystems.turret.turretsubsystemiotalon import TurretSubsystemIOTalon
from util.convenientmath import clamp

from constants.turret import (
    kTurretSimMotor,
    kTurretGearRatio,
    kTurretSimInertia,
    kTurretMinAngle,
    kTurretMaxAngle,
    kTurretStartingAngle,
    kTurretInvertedValue,
)
from constants.math import kRadiansPerRevolution
from constants import kRobotUpdatePeriod


class TurretSubsystemIOSim(TurretSubsystemIOTalon):
    """Simulate the Talon motor."""

    def __init__(self) -> None:
        super().__init__()  # Initialize the Sim motor the same way as the actual Talon motor
        self.turretSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                kTurretSimMotor, kTurretSimInertia, kTurretGearRatio
            ),
            kTurretSimMotor,
        )  # Create a DC motor simulation model with specified parameters

    def updateInputs(self, inputs: TurretSubsystemIO.TurretSubsystemIOInputs) -> None:
        """Simulate the motor behavior, then update TalonIO inputs."""
        turretMotorSim = self.motor.sim_state
        simVoltage = RobotController.getInputVoltage()

        turretAppliedVoltage = clamp(turretMotorSim.motor_voltage, -12.0, 12.0)

        self.turretSimModel.setInputVoltage(turretAppliedVoltage)
        self.turretSimModel.update(kRobotUpdatePeriod)

        positionFlip = (
            1 if kTurretInvertedValue == InvertedValue.CLOCKWISE_POSITIVE else -1
        )
        effectiveTurretPosition = self.turretSimModel.getAngularPosition() * (
            positionFlip
        )
        if (
            effectiveTurretPosition
            > kTurretMaxAngle.radians() - kTurretStartingAngle.radians()
        ):
            self.turretSimModel.setAngle(
                (kTurretMaxAngle.radians() - kTurretStartingAngle.radians())
                * positionFlip
            )
            self.turretSimModel.setAngularVelocity(0)
        elif (
            effectiveTurretPosition
            < kTurretMinAngle.radians() - kTurretStartingAngle.radians()
        ):
            self.turretSimModel.setAngle(
                (kTurretMinAngle.radians() - kTurretStartingAngle.radians())
                * positionFlip
            )
            self.turretSimModel.setAngularVelocity(0)

        turretMotorSim.set_raw_rotor_position(
            self.turretSimModel.getAngularPositionRotations() * kTurretGearRatio
        )
        turretMotorSim.set_rotor_velocity(
            self.turretSimModel.getAngularVelocity()
            / kRadiansPerRevolution
            * kTurretGearRatio
        )
        turretMotorSim.set_supply_voltage(
            clamp(
                simVoltage - turretMotorSim.supply_current * kTurretSimMotor.R,
                0,
                simVoltage,
            )  # Apply some simulated voltage within appropriate limits
        )
        super().updateInputs(inputs)  # Call the TalonIO updateInputs method
