from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from subsystems.indexer.indexersubsystemiotalon import IndexerSubsystemIOTalon

from constants.math import kRadiansPerRevolution
from constants.indexer import (
    kKickerGearRatio,
    kKickerMotor,
    kKickerMotor2,
    kSpindexerGearRatio,
    kSpindexerMotor,
    kSpindexerMotor2,
)
from constants import kRobotUpdatePeriod
from util.convenientmath import clamp


class IndexerSubsystemIOSIM(IndexerSubsystemIOTalon):
    def __init__(self) -> None:
        super().__init__()
        self.spindexerSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kSpindexerMotor, 0.04, kSpindexerGearRatio),
            kSpindexerMotor,
            kSpindexerMotor2,
        )

        self.kickerSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kKickerMotor, 0.04, kKickerGearRatio),
            kKickerMotor,
            kKickerMotor2,
        )

    def updateInputs(self, inputs: IndexerSubsystemIO.IndexerSubsystemInputs) -> None:
        spindexerMotorSim = self.spindexerMotor.sim_state
        kickerMotorSim = self.kickerMotor.sim_state

        simVoltage = RobotController.getInputVoltage()

        spindexerAppliedVoltage = clamp(spindexerMotorSim.motor_voltage, -12.0, 12.0)
        kickerAppliedVoltage = clamp(kickerMotorSim.motor_voltage, -12.0, 12.0)

        self.spindexerSimModel.setInputVoltage(spindexerAppliedVoltage)
        self.kickerSimModel.setInputVoltage(kickerAppliedVoltage)

        self.spindexerSimModel.update(kRobotUpdatePeriod)
        self.kickerSimModel.update(kRobotUpdatePeriod)

        spindexerMotorSim.set_raw_rotor_position(
            self.spindexerSimModel.getAngularPositionRotations() * kSpindexerGearRatio
        )
        spindexerMotorSim.set_rotor_velocity(
            self.spindexerSimModel.getAngularVelocity()
            * kSpindexerGearRatio
            / kRadiansPerRevolution
        )
        spindexerMotorSim.set_supply_voltage(
            clamp(
                simVoltage - spindexerMotorSim.supply_current * kSpindexerMotor.R,
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
