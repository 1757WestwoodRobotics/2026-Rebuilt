from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from subsystems.indexer.indexersubsystemiotalon import IndexerSubsystemIOTalon

from constants.math import kRadiansPerRevolution
from constants.indexer import (
    kKickerGearRatio,
    kKickerMotorLower,
    kKickerMotorUpper,
    kSpindexerGearRatio,
    kSpindexerMotor1,
    kSpindexerMotor2,
)
from constants import kRobotUpdatePeriod
from util.convenientmath import clamp


class IndexerSubsystemIOSIM(IndexerSubsystemIOTalon):
    def __init__(self) -> None:
        super().__init__()
        self.spindexerSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kSpindexerMotor1, 0.04, kSpindexerGearRatio),
            kSpindexerMotor1,
            kSpindexerMotor2,
        )

        self.kickerLowerSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kKickerMotorLower, 0.04, kKickerGearRatio),
            kKickerMotorLower,
        )

        self.kickerUpperSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kKickerMotorUpper, 0.04, kKickerGearRatio),
            kKickerMotorUpper,
        )

    def updateInputs(self, inputs: IndexerSubsystemIO.IndexerSubsystemInputs) -> None:
        spindexerMotorSim = self.spindexerMotor.sim_state
        kickerMotorSim = self.kickerMotor.sim_state

        simVoltage = RobotController.getInputVoltage()

        spindexerAppliedVoltage = clamp(spindexerMotorSim.motor_voltage, -12.0, 12.0)
        kickerAppliedVoltage = clamp(kickerMotorSim.motor_voltage, -12.0, 12.0)

        self.spindexerSimModel.setInputVoltage(spindexerAppliedVoltage)
        self.kickerLowerSimModel.setInputVoltage(kickerAppliedVoltage)
        self.kickerUpperSimModel.setInputVoltage(kickerAppliedVoltage)

        self.spindexerSimModel.update(kRobotUpdatePeriod)
        self.kickerLowerSimModel.update(kRobotUpdatePeriod)
        self.kickerUpperSimModel.update(kRobotUpdatePeriod)

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
                simVoltage - spindexerMotorSim.supply_current * kSpindexerMotor1.R,
                0,
                simVoltage,
            )
        )

        kickerMotorSim.set_raw_rotor_position(
            self.kickerLowerSimModel.getAngularPositionRotations() * kKickerGearRatio
        )
        kickerMotorSim.set_rotor_velocity(
            self.kickerLowerSimModel.getAngularVelocity()
            * kKickerGearRatio
            / kRadiansPerRevolution
        )
        kickerMotorSim.set_supply_voltage(
            clamp(
                simVoltage - kickerMotorSim.supply_current * kKickerMotorLower.R,
                0,
                simVoltage,
            )
        )

        super().updateInputs(inputs)
