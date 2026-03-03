from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import NeutralModeValue
from phoenix6.controls import VoltageOut
from phoenix6.hardware.talon_fx import TalonFX

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.phoenixutil import tryUntilOk, PhoenixUtil

from constants import kRobotUpdateFrequency
from constants.indexer import (
    kKickerLowerCANId,
    kKickerUpperCANId,
    kKickerCurrentLimit,
    kKickerGearRatio,
    kSpindexer1CANId,
    kSpindexer2CANId,
    kSpindexerCurrentLimit,
    kSpindexerGearRatio,
)


class IndexerSubsystemIOTalon(IndexerSubsystemIO):
    spindexer1Config: TalonFXConfiguration = TalonFXConfiguration()
    spindexer2Config: TalonFXConfiguration = TalonFXConfiguration()
    kickerLowerConfig: TalonFXConfiguration = TalonFXConfiguration()
    kickerUpperConfig: TalonFXConfiguration = TalonFXConfiguration()

    spindexer1Demand: VoltageOut = VoltageOut(0, False)
    spindexer2Demand: VoltageOut = VoltageOut(0, False)
    kickerLowerDemand: VoltageOut = VoltageOut(0, False)
    kickerUpperDemand: VoltageOut = VoltageOut(0, False)

    def __init__(self) -> None:
        self.spindexer1Motor = TalonFX(kSpindexer1CANId)
        self.spindexer2Motor = TalonFX(kSpindexer2CANId)
        self.kickerLowerMotor = TalonFX(kKickerLowerCANId)
        self.kickerUpperMotor = TalonFX(kKickerUpperCANId)

        self.spindexer1Config.motor_output.neutral_mode = NeutralModeValue.COAST
        self.spindexer2Config.motor_output.neutral_mode = NeutralModeValue.COAST
        self.kickerLowerConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.kickerUpperConfig.motor_output.neutral_mode = NeutralModeValue.COAST

        self.spindexer1Config.current_limits = kSpindexerCurrentLimit
        self.spindexer2Config.current_limits = kSpindexerCurrentLimit
        self.kickerLowerConfig.current_limits = kKickerCurrentLimit
        self.kickerUpperConfig.current_limits = kKickerCurrentLimit

        self.spindexer1Config.feedback.sensor_to_mechanism_ratio = kSpindexerGearRatio
        self.spindexer2Config.feedback.sensor_to_mechanism_ratio = kSpindexerGearRatio
        self.kickerLowerConfig.feedback.sensor_to_mechanism_ratio = kKickerGearRatio
        self.kickerUpperConfig.feedback.sensor_to_mechanism_ratio = kKickerGearRatio

        tryUntilOk(
            5,
            lambda: self.spindexer1Motor.configurator.apply(
                self.spindexer1Config, 0.25
            ),
        )
        tryUntilOk(
            5,
            lambda: self.spindexer2Motor.configurator.apply(
                self.spindexer2Config, 0.25
            ),
        )
        tryUntilOk(
            5,
            lambda: self.kickerLowerMotor.configurator.apply(
                self.kickerUpperConfig, 0.25
            ),
        )
        tryUntilOk(
            5,
            lambda: self.kickerUpperMotor.configurator.apply(
                self.kickerUpperConfig, 0.25
            ),
        )

        self.spindexerPosition = self.spindexer1Motor.get_position()
        self.spindexerVelocity = self.spindexer1Motor.get_velocity()
        self.spindexerSupply = self.spindexer1Motor.get_supply_current()
        self.spindexerApplied = self.spindexer1Motor.get_motor_voltage()

        self.spindexerPosition = self.spindexer2Motor.get_position()
        self.spindexerVelocity = self.spindexer2Motor.get_velocity()
        self.spindexerSupply = self.spindexer2Motor.get_supply_current()
        self.spindexerApplied = self.spindexer2Motor.get_motor_voltage()

        self.kickerPosition = self.kickerLowerMotor.get_position()
        self.kickerVelocity = self.kickerLowerMotor.get_velocity()
        self.kickerSupply = self.kickerLowerMotor.get_supply_current()
        self.kickerApplied = self.kickerLowerMotor.get_motor_voltage()

        self.kickerPosition = self.kickerUpperMotor.get_position()
        self.kickerVelocity = self.kickerUpperMotor.get_velocity()
        self.kickerSupply = self.kickerUpperMotor.get_supply_current()
        self.kickerApplied = self.kickerUpperMotor.get_motor_voltage()

        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.spindexerPosition,
            self.spindexerVelocity,
            self.spindexerSupply,
            self.spindexerApplied,
            self.kickerPosition,
            self.kickerVelocity,
            self.kickerSupply,
            self.kickerApplied,
        )

        PhoenixUtil.registerSignals(
            "",
            self.spindexerPosition,
            self.spindexerVelocity,
            self.spindexerSupply,
            self.spindexerApplied,
            self.kickerPosition,
            self.kickerVelocity,
            self.kickerSupply,
            self.kickerApplied,
        )

    def updateInputs(self, inputs: IndexerSubsystemIO.IndexerSubsystemInputs) -> None:
        inputs.spindexerConnected = BaseStatusSignal.is_all_good(
            self.spindexerPosition,
            self.spindexerVelocity,
            self.spindexerApplied,
            self.spindexerSupply,
        )

        inputs.spindexPosition = self.spindexerPosition.value
        inputs.indexVelocity = self.spindexerVelocity.value
        inputs.spindexAppliedVolts = self.spindexerApplied.value
        inputs.spindexSupplyAmps = self.spindexerSupply.value

        inputs.kickConnected = BaseStatusSignal.is_all_good(
            self.kickerPosition,
            self.kickerVelocity,
            self.kickerApplied,
            self.kickerSupply,
        )
        inputs.kickPosition = self.kickerPosition.value
        inputs.kickVelocity = self.kickerVelocity.value
        inputs.kickAppliedVolts = self.kickerApplied.value
        inputs.kickSupplyAmps = self.kickerSupply.value

    def setIndexerTarget(self, spindexer: float, kicker: float) -> None:
        self.spindexerMotor.set_control(self.spindexerDemand.with_output(spindexer))
        self.kickerMotor.set_control(self.kickerDemand.with_output(kicker))
