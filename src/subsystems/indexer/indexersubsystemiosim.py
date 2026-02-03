from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from subsystems.indexer.indexersubsystemiotalon import IndexerSubsystemIOTalon

from constants.math import kRadiansPerRevolution
from constants.indexer import (
    kKickerGearRatio,
    kKickerMotor,
    kIndexerGearRatio,
    kIndexerMotor,
)
from constants.sim import kSimMotorResistance
from constants import kRobotUpdatePeriod
from util.convenientmath import clamp


class IndexerSubsystemIOSIM(IndexerSubsystemIOTalon):
    def __init__(self) -> None:
        super().__init__()
        self.indexerSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kIndexerMotor, 0.04, kIndexerGearRatio),
            kIndexerMotor,
        )

        self.kickerSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kKickerMotor, 0.04, kKickerGearRatio),
            kKickerMotor,
        )

    def updateInputs(self, inputs: IndexerSubsystemIO.IndexerSubsystemInputs) -> None:
        indexerMotorSim = self.indexerMotor.sim_state
        kickerMotorSim = self.kickerMotor.sim_state

        simVoltage = RobotController.getInputVoltage()

        indexerAppliedVoltage = clamp(indexerMotorSim.motor_voltage, -12.0, 12.0)
        kickerAppliedVoltage = clamp(kickerMotorSim.motor_voltage, -12.0, 12.0)

        self.indexerSimModel.setInputVoltage(indexerAppliedVoltage)
        self.kickerSimModel.setInputVoltage(kickerAppliedVoltage)

        self.indexerSimModel.update(kRobotUpdatePeriod)
        self.kickerSimModel.update(kRobotUpdatePeriod)

        indexerMotorSim.set_raw_rotor_position(
            self.indexerSimModel.getAngularPositionRotations() * kIndexerGearRatio
        )
        indexerMotorSim.set_rotor_velocity(
            self.indexerSimModel.getAngularVelocity()
            * kIndexerGearRatio
            / kRadiansPerRevolution
        )
        indexerMotorSim.set_supply_voltage(
            clamp(
                simVoltage - indexerMotorSim.supply_current * kIndexerMotor.R,
                0,
                simVoltage,
            )
        )

        kickerMotorSim.set_raw_rotor_position(
            self.kickerSimModel.getAngularPositionRotations() * kKickerGearRatio
        )
        kickerMotorSim.set_rotor_velocity(
            self.kickerSimModel.getAngularVelocity()
            * kKickerGearRatio
            / kRadiansPerRevolution
        )
        kickerMotorSim.set_supply_voltage(
            clamp(
                simVoltage - kickerMotorSim.supply_current * kKickerMotor.R,
                0,
                simVoltage,
            )
        )

        super().updateInputs(inputs)
