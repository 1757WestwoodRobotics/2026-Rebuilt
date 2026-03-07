from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from subsystems.indexer.indexersubsystemiotalon import IndexerSubsystemIOTalon

from constants.math import kRadiansPerRevolution
from constants.indexer import (
    kKickerGearRatio,
    kKickerSystem,
    kSpindexerGearRatio,
    kSpindexerSystem,
)
from constants import kRobotUpdatePeriod
from util.convenientmath import clamp


class IndexerSubsystemIOSIM(IndexerSubsystemIOTalon):
    def __init__(self) -> None:
        super().__init__()
        self.spindexer1SimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kSpindexerSystem, 0.04, kSpindexerGearRatio),
            kSpindexerSystem,
        )

        self.kickerUpperSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kKickerSystem, 0.04, kKickerGearRatio),
            kKickerSystem,
        )

    def updateInputs(self, inputs: IndexerSubsystemIO.IndexerSubsystemInputs) -> None:
        spindexer1MotorSim = self.spindexer1Motor.sim_state
        spindexer2MotorSim = self.spindexer1Motor.sim_state
        kickerLowerMotorSim = self.kickerUpperMotor.sim_state
        kickerUpperMotorSim = self.kickerUpperMotor.sim_state

        simVoltage = RobotController.getInputVoltage()

        spindexer1AppliedVoltage = clamp(spindexer1MotorSim.motor_voltage, -12.0, 12.0)
        kickerUpperAppliedVoltage = clamp(
            kickerUpperMotorSim.motor_voltage, -12.0, 12.0
        )

        self.spindexer1SimModel.setInputVoltage(spindexer1AppliedVoltage)
        self.kickerUpperSimModel.setInputVoltage(kickerUpperAppliedVoltage)

        self.spindexer1SimModel.update(kRobotUpdatePeriod)
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
                simVoltage - spindexer1MotorSim.supply_current * kSpindexerSystem.R,
                0,
                simVoltage,
            )
        )

        spindexer2MotorSim.set_raw_rotor_position(
            self.spindexer1SimModel.getAngularPosition()
        )
        spindexer2MotorSim.set_rotor_velocity(
            self.spindexer1SimModel.getAngularVelocity()
        )
        spindexer2MotorSim.set_supply_voltage(self.spindexer1SimModel.getInputVoltage())

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
                simVoltage - kickerUpperMotorSim.supply_current * kKickerSystem.R,
                0,
                simVoltage,
            )
        )

        # Setting negative values as Upper because they are opposed.  Not sure this is correct.
        kickerLowerMotorSim.set_raw_rotor_position(
            -self.kickerUpperSimModel.getAngularPosition()
        )
        kickerLowerMotorSim.set_rotor_velocity(
            -self.kickerUpperSimModel.getAngularVelocity()
        )
        kickerLowerMotorSim.set_supply_voltage(
            -self.kickerUpperSimModel.getInputVoltage()
        )

        super().updateInputs(inputs)
