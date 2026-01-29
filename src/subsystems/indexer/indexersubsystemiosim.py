from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from subsystems.indexer.indexersubsystemiotalon import IndexerSubsystemIOTalon

from constants.math import kRadiansPerRevolution
from constants.indexer import (
    kFeederGearRatio,
    kFeederMotor,
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

        self.feederSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kFeederMotor, 0.04, kFeederGearRatio),
            kFeederMotor,
        )

    def updateInputs(self, inputs: IndexerSubsystemIO.IndexerSubsystemInputs) -> None:
        indexerMotorSim = self.indexerMotor.sim_state
        feederMotorSim = self.feederMotor.sim_state

        simVoltage = RobotController.getInputVoltage()

        indexerAppliedVoltage = clamp(indexerMotorSim.motor_voltage, -12.0, 12.0)
        feederAppliedVoltage = clamp(feederMotorSim.motor_voltage, -12.0, 12.0)

        self.indexerSimModel.setInputVoltage(indexerAppliedVoltage)
        self.feederSimModel.setInputVoltage(feederAppliedVoltage)

        self.indexerSimModel.update(kRobotUpdatePeriod)
        self.feederSimModel.update(kRobotUpdatePeriod)

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

        feederMotorSim.set_raw_rotor_position(
            self.feederSimModel.getAngularPositionRotations() * kFeederGearRatio
        )
        feederMotorSim.set_rotor_velocity(
            self.feederSimModel.getAngularVelocity()
            * kFeederGearRatio
            / kRadiansPerRevolution
        )
        feederMotorSim.set_supply_voltage(
            clamp(
                simVoltage - feederMotorSim.supply_current * kFeederMotor.R,
                0,
                simVoltage,
            )
        )

        super().updateInputs(inputs)
