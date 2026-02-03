from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import ForwardLimitValue, NeutralModeValue
from phoenix6.controls import VoltageOut
from phoenix6.hardware.talon_fx import TalonFX

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.phoenixutil import tryUntilOk, PhoenixUtil

from constants import kRobotUpdateFrequency
from constants.indexer import (
    kKickerCANId,
    kKickerCurrentLimit,
    kKickerGearRatio,
    kIndexerCANId,
    kIndexerCurrentLimit,
    kIndexerGearRatio,
)


class IndexerSubsystemIOTalon(IndexerSubsystemIO):
    indexerConfig: TalonFXConfiguration = TalonFXConfiguration()
    kickerConfig: TalonFXConfiguration = TalonFXConfiguration()

    indexerDemand: VoltageOut = VoltageOut(0, False)
    kickerDemand: VoltageOut = VoltageOut(0, False)

    def __init__(self) -> None:
        self.indexerMotor = TalonFX(kIndexerCANId)
        self.kickerMotor = TalonFX(kKickerCANId)

        self.indexerConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.kickerConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE

        self.indexerConfig.current_limits = kIndexerCurrentLimit
        self.kickerConfig.current_limits = kKickerCurrentLimit

        self.indexerConfig.kickback.sensor_to_mechanism_ratio = kIndexerGearRatio
        self.kickerConfig.kickback.sensor_to_mechanism_ratio = kKickerGearRatio

        tryUntilOk(
            5, lambda: self.indexerMotor.configurator.apply(self.indexerConfig, 0.25)
        )
        tryUntilOk(
            5, lambda: self.kickerMotor.configurator.apply(self.kickerConfig, 0.25)
        )

        self.indexerPosition = self.indexerMotor.get_position()
        self.indexerVelocity = self.indexerMotor.get_velocity()
        self.indexerSupply = self.indexerMotor.get_supply_current()
        self.indexerApplied = self.indexerMotor.get_motor_voltage()

        self.indexerSensor = self.indexerMotor.get_forward_limit()

        self.kickerPosition = self.kickerMotor.get_position()
        self.kickerVelocity = self.kickerMotor.get_velocity()
        self.kickerSupply = self.kickerMotor.get_supply_current()
        self.kickerApplied = self.kickerMotor.get_motor_voltage()

        self.kickerSensor = self.kickerMotor.get_forward_limit()

        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.indexerPosition,
            self.indexerVelocity,
            self.indexerSupply,
            self.indexerApplied,
            self.indexerSensor,
            self.kickerPosition,
            self.kickerVelocity,
            self.kickerSupply,
            self.kickerApplied,
            self.kickerSensor,
        )

        PhoenixUtil.registerSignals(
            "",
            self.indexerPosition,
            self.indexerVelocity,
            self.indexerSupply,
            self.indexerApplied,
            self.indexerSensor,
            self.kickerPosition,
            self.kickerVelocity,
            self.kickerSupply,
            self.kickerApplied,
            self.kickerSensor,
        )

    def updateInputs(self, inputs: IndexerSubsystemIO.IndexerSubsystemInputs) -> None:
        inputs.indexerConnected = BaseStatusSignal.is_all_good(
            self.indexerPosition,
            self.indexerVelocity,
            self.indexerApplied,
            self.indexerSupply,
            self.indexerSensor,
        )

        inputs.indexPosition = self.indexerPosition.value
        inputs.indexVelocity = self.indexerVelocity.value
        inputs.indexAppliedVolts = self.indexerApplied.value
        inputs.indexSupplyAmps = self.indexerSupply.value

        inputs.kickConnected = BaseStatusSignal.is_all_good(
            self.kickerPosition,
            self.kickerVelocity,
            self.kickerApplied,
            self.kickerSupply,
            self.kickerSensor,
        )
        inputs.kickPosition = self.kickerPosition.value
        inputs.kickVelocity = self.kickerVelocity.value
        inputs.kickAppliedVolts = self.kickerApplied.value
        inputs.kickSupplyAmps = self.kickerSupply.value

        inputs.indexerSensor = (
            self.indexerSensor.value == ForwardLimitValue.CLOSED_TO_GROUND
        )
        inputs.kickSensor = (
            self.kickerSensor.value == ForwardLimitValue.CLOSED_TO_GROUND
        )

    def setIndexerTarget(self, indexer: float, kicker: float) -> None:
        self.indexerMotor.set_control(self.indexerDemand.with_output(indexer))
        self.kickerMotor.set_control(self.kickerDemand.with_output(kicker))
