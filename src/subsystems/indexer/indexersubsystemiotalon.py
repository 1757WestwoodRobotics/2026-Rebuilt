from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import ForwardLimitValue, NeutralModeValue
from phoenix6.controls import VoltageOut
from phoenix6.hardware.talon_fx import TalonFX

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.phoenixutil import tryUntilOk, PhoenixUtil

from constants import kRobotUpdateFrequency
from constants.indexer import (
    kFeederCANId,
    kFeederCurrentLimit,
    kFeederGearRatio,
    kIndexerCANId,
    kIndexerCurrentLimit,
    kIndexerGearRatio,
)


class IndexerSubsystemIOTalon(IndexerSubsystemIO):
    indexerConfig: TalonFXConfiguration = TalonFXConfiguration()
    feederConfig: TalonFXConfiguration = TalonFXConfiguration()

    indexerDemand: VoltageOut = VoltageOut(0, False)
    feederDemand: VoltageOut = VoltageOut(0, False)

    def __init__(self) -> None:
        self.indexerMotor = TalonFX(kIndexerCANId)
        self.feederMotor = TalonFX(kFeederCANId)

        self.indexerConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.feederConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE

        self.indexerConfig.current_limits = kIndexerCurrentLimit
        self.feederConfig.current_limits = kFeederCurrentLimit

        self.indexerConfig.feedback.sensor_to_mechanism_ratio = kIndexerGearRatio
        self.feederConfig.feedback.sensor_to_mechanism_ratio = kFeederGearRatio

        tryUntilOk(
            5, lambda: self.indexerMotor.configurator.apply(self.indexerConfig, 0.25)
        )
        tryUntilOk(
            5, lambda: self.feederMotor.configurator.apply(self.feederConfig, 0.25)
        )

        self.indexerPosition = self.indexerMotor.get_position()
        self.indexerVelocity = self.indexerMotor.get_velocity()
        self.indexerSupply = self.indexerMotor.get_supply_current()
        self.indexerApplied = self.indexerMotor.get_motor_voltage()

        self.indexerSensor = self.indexerMotor.get_forward_limit()

        self.feederPosition = self.feederMotor.get_position()
        self.feederVelocity = self.feederMotor.get_velocity()
        self.feederSupply = self.feederMotor.get_supply_current()
        self.feederApplied = self.feederMotor.get_motor_voltage()

        self.feederSensor = self.feederMotor.get_forward_limit()

        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.indexerPosition,
            self.indexerVelocity,
            self.indexerSupply,
            self.indexerApplied,
            self.indexerSensor,
            self.feederPosition,
            self.feederVelocity,
            self.feederSupply,
            self.feederApplied,
            self.feederSensor,
        )

        PhoenixUtil.registerSignals(
            "",
            self.indexerPosition,
            self.indexerVelocity,
            self.indexerSupply,
            self.indexerApplied,
            self.indexerSensor,
            self.feederPosition,
            self.feederVelocity,
            self.feederSupply,
            self.feederApplied,
            self.feederSensor,
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

        inputs.feedConnected = BaseStatusSignal.is_all_good(
            self.feederPosition,
            self.feederVelocity,
            self.feederApplied,
            self.feederSupply,
            self.feederSensor,
        )
        inputs.feedPosition = self.feederPosition.value
        inputs.feedVelocity = self.feederVelocity.value
        inputs.feedAppliedVolts = self.feederApplied.value
        inputs.feedSupplyAmps = self.feederSupply.value

        inputs.indexerSensor = (
            self.indexerSensor.value == ForwardLimitValue.CLOSED_TO_GROUND
        )
        inputs.feedSensor = (
            self.feederSensor.value == ForwardLimitValue.CLOSED_TO_GROUND
        )

    def setIndexerTarget(self, indexer: float, feeder: float) -> None:
        self.indexerMotor.set_control(self.indexerDemand.with_output(indexer))
        self.feederMotor.set_control(self.feederDemand.with_output(feeder))
