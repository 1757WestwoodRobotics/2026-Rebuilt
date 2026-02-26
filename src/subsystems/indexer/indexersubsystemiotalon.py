from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import NeutralModeValue
from phoenix6.controls import VoltageOut
from phoenix6.hardware.talon_fx import TalonFX

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.phoenixutil import tryUntilOk, PhoenixUtil

from constants import kRobotUpdateFrequency
from constants.indexer import (
    kKickerCANId,
    kKickerCurrentLimit,
    kKickerGearRatio,
    kSpindexerCANId,
    kSpindexerCurrentLimit,
    kSpindexerGearRatio,
)


class IndexerSubsystemIOTalon(IndexerSubsystemIO):
    spindexerConfig: TalonFXConfiguration = TalonFXConfiguration()
    kickerConfig: TalonFXConfiguration = TalonFXConfiguration()

    spindexerDemand: VoltageOut = VoltageOut(0, False)
    kickerDemand: VoltageOut = VoltageOut(0, False)

    def __init__(self) -> None:
        self.spindexerMotor = TalonFX(kSpindexerCANId)
        self.kickerMotor = TalonFX(kKickerCANId)

        self.spindexerConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.kickerConfig.motor_output.neutral_mode = NeutralModeValue.COAST

        self.spindexerConfig.current_limits = kSpindexerCurrentLimit
        self.kickerConfig.current_limits = kKickerCurrentLimit

        self.spindexerConfig.feedback.sensor_to_mechanism_ratio = kSpindexerGearRatio
        self.kickerConfig.feedback.sensor_to_mechanism_ratio = kKickerGearRatio

        tryUntilOk(
            5,
            lambda: self.spindexerMotor.configurator.apply(self.spindexerConfig, 0.25),
        )
        tryUntilOk(
            5, lambda: self.kickerMotor.configurator.apply(self.kickerConfig, 0.25)
        )

        self.spindexerPosition = self.spindexerMotor.get_position()
        self.spindexerVelocity = self.spindexerMotor.get_velocity()
        self.spindexerSupply = self.spindexerMotor.get_supply_current()
        self.spindexerApplied = self.spindexerMotor.get_motor_voltage()

        self.kickerPosition = self.kickerMotor.get_position()
        self.kickerVelocity = self.kickerMotor.get_velocity()
        self.kickerSupply = self.kickerMotor.get_supply_current()
        self.kickerApplied = self.kickerMotor.get_motor_voltage()

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
