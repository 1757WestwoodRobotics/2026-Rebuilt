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
        self.spindexer1SimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kSpindexerMotor1, 0.04, kSpindexerGearRatio),
            kSpindexerMotor1,
        )

        self.spindexer2SimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kKickerMotorUpper, 0.04, kSpindexerGearRatio),
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
        spindexer1MotorSim = self.spindexer1Motor.sim_state
        spindexer2MotorSim = self.spindexer2Motor.sim_state
        kickerLowerMotorSim = self.kickerLowerMotor.sim_state
        kickerUpperMotorSim = self.kickerUpperMotor.sim_state

        simVoltage = RobotController.getInputVoltage()

        spindexer1AppliedVoltage = clamp(spindexer1MotorSim.motor_voltage, -12.0, 12.0)
        spindexer2AppliedVoltage = clamp(spindexer2MotorSim.motor_voltage, -12.0, 12.0)
        kickerLowerAppliedVoltage = clamp(
            kickerLowerMotorSim.motor_voltage, -12.0, 12.0
        )
        kickerUpperAppliedVoltage = clamp(
            kickerUpperMotorSim.motor_voltage, -12.0, 12.0
        )

        self.spindexer1SimModel.setInputVoltage(spindexer1AppliedVoltage)
        self.spindexer2SimModel.setInputVoltage(spindexer2AppliedVoltage)
        self.kickerLowerSimModel.setInputVoltage(kickerLowerAppliedVoltage)
        self.kickerUpperSimModel.setInputVoltage(kickerUpperAppliedVoltage)

        self.spindexer1SimModel.update(kRobotUpdatePeriod)
        self.spindexer2SimModel.update(kRobotUpdatePeriod)
        self.kickerLowerSimModel.update(kRobotUpdatePeriod)
        self.kickerUpperSimModel.update(kRobotUpdatePeriod)

        spindexer1MotorSim.set_raw_rotor_position(
            self.spindexer1SimModel.getAngularPositionRotations() * kSpindexerGearRatio
        )
        spindexer1MotorSim.set_rotor_velocity(
            self.spindexer1SimModel.getAngularVelocity()
            * kSpindexerGearRatio
            / kRadiansPerRevolution
        )
        spindexer1MotorSim.set_supply_voltage(
            clamp(
                simVoltage - spindexer1MotorSim.supply_current * kSpindexerMotor1.R,
                0,
                simVoltage,
            )
        )

        spindexer2MotorSim.set_raw_rotor_position(
            self.spindexer2SimModel.getAngularPositionRotations() * kSpindexerGearRatio
        )
        spindexer2MotorSim.set_rotor_velocity(
            self.spindexer2SimModel.getAngularVelocity()
            * kSpindexerGearRatio
            / kRadiansPerRevolution
        )
        spindexer2MotorSim.set_supply_voltage(
            clamp(
                simVoltage - spindexer2MotorSim.supply_current * kSpindexerMotor2.R,
                0,
                simVoltage,
            )
        )

        kickerLowerMotorSim.set_raw_rotor_position(
            self.kickerLowerSimModel.getAngularPositionRotations() * kKickerGearRatio
        )
        kickerLowerMotorSim.set_rotor_velocity(
            self.kickerLowerSimModel.getAngularVelocity()
            * kKickerGearRatio
            / kRadiansPerRevolution
        )
        kickerLowerMotorSim.set_supply_voltage(
            clamp(
                simVoltage - kickerLowerMotorSim.supply_current * kKickerMotorLower.R,
                0,
                simVoltage,
            )
        )

        kickerUpperMotorSim.set_raw_rotor_position(
            self.kickerUpperSimModel.getAngularPositionRotations() * kKickerGearRatio
        )
        kickerUpperMotorSim.set_rotor_velocity(
            self.kickerUpperSimModel.getAngularVelocity()
            * kKickerGearRatio
            / kRadiansPerRevolution
        )
        kickerUpperMotorSim.set_supply_voltage(
            clamp(
                simVoltage - kickerUpperMotorSim.supply_current * kKickerMotorUpper.R,
                0,
                simVoltage,
            )
        )

        super().updateInputs(inputs)
