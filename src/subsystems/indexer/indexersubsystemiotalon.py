from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import NeutralModeValue
from phoenix6.controls import VoltageOut, Follower
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.signals import MotorAlignmentValue

from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from util.phoenixutil import tryUntilOk, PhoenixUtil

from constants import kRobotUpdateFrequency, kRioCANBus, kRobotDiagnosticUpdateFrequency
from constants.indexer import (
    kKickerLowerCANId,
    kKickerUpperCANId,
    kKickerCurrentLimit,
    kKickerGearRatio,
    kSpindexer1CANId,
    kSpindexer2CANId,
    kSpindexerCurrentLimit,
    kSpindexerGearRatio,
    kSpindexerInvertedValue,
)


class IndexerSubsystemIOTalon(IndexerSubsystemIO):
    spindexer1Config: TalonFXConfiguration = TalonFXConfiguration()
    spindexer2Config: TalonFXConfiguration = TalonFXConfiguration()
    kickerLowerConfig: TalonFXConfiguration = TalonFXConfiguration()
    kickerUpperConfig: TalonFXConfiguration = TalonFXConfiguration()

    spindexerDemand: VoltageOut = VoltageOut(0)
    kickerDemand: VoltageOut = VoltageOut(0)

    def __init__(self) -> None:
        self.spindexer1Motor = TalonFX(kSpindexer1CANId)
        self.spindexer2Motor = TalonFX(kSpindexer2CANId)
        self.kickerLowerMotor = TalonFX(kKickerLowerCANId)
        self.kickerUpperMotor = TalonFX(kKickerUpperCANId)

        # we want the spindexer to stop spitting balls when comanded neutrally to
        self.spindexer1Config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.spindexer2Config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.kickerLowerConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.kickerUpperConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE

        self.spindexer1Config.motor_output.inverted = kSpindexerInvertedValue

        self.spindexer1Config.current_limits = kSpindexerCurrentLimit
        self.spindexer2Config.current_limits = kSpindexerCurrentLimit
        self.kickerLowerConfig.current_limits = kKickerCurrentLimit
        self.kickerUpperConfig.current_limits = kKickerCurrentLimit

        self.spindexer1Config.feedback.sensor_to_mechanism_ratio = kSpindexerGearRatio
        self.spindexer2Config.feedback.sensor_to_mechanism_ratio = kSpindexerGearRatio
        self.kickerLowerConfig.feedback.sensor_to_mechanism_ratio = kKickerGearRatio
        self.kickerUpperConfig.feedback.sensor_to_mechanism_ratio = kKickerGearRatio

        # Create a leader/follower arrangement for the spindexer and kicker motors.
        self.spindexerFollower = Follower(
            self.spindexer1Motor.device_id, MotorAlignmentValue.ALIGNED
        )
        self.spindexer2Motor.set_control(self.spindexerFollower)

        self.kickerFollower = Follower(
            self.kickerUpperMotor.device_id, MotorAlignmentValue.OPPOSED
        )
        self.kickerLowerMotor.set_control(self.kickerFollower)

        tryUntilOk(
            5,
            lambda: self.spindexer1Motor.configurator.apply(
                self.spindexer1Config, 0.25
            ),
            "Indexer spindexer1 config",
        )
        tryUntilOk(
            5,
            lambda: self.spindexer2Motor.configurator.apply(
                self.spindexer2Config, 0.25
            ),
            "Indexer spindexer2 config",
        )
        tryUntilOk(
            5,
            lambda: self.kickerUpperMotor.configurator.apply(
                self.kickerUpperConfig, 0.25
            ),
            "Indexer kickerUpper config",
        )
        tryUntilOk(
            5,
            lambda: self.kickerLowerMotor.configurator.apply(
                self.kickerLowerConfig, 0.25
            ),
            "Indexer kickerLower config",
        )

        self.spindexer1Position = self.spindexer1Motor.get_position()
        self.spindexer1Velocity = self.spindexer1Motor.get_velocity()
        self.spindexer1Supply = self.spindexer1Motor.get_supply_current()
        self.spindexer1Applied = self.spindexer1Motor.get_motor_voltage()

        self.spindexer2Position = self.spindexer2Motor.get_position()
        self.spindexer2Velocity = self.spindexer2Motor.get_velocity()
        self.spindexer2Supply = self.spindexer2Motor.get_supply_current()
        self.spindexer2Applied = self.spindexer2Motor.get_motor_voltage()

        self.kickerLowerPosition = self.kickerLowerMotor.get_position()
        self.kickerLowerVelocity = self.kickerLowerMotor.get_velocity()
        self.kickerLowerSupply = self.kickerLowerMotor.get_supply_current()
        self.kickerLowerApplied = self.kickerLowerMotor.get_motor_voltage()

        self.kickerUpperPosition = self.kickerUpperMotor.get_position()
        self.kickerUpperVelocity = self.kickerUpperMotor.get_velocity()
        self.kickerUpperSupply = self.kickerUpperMotor.get_supply_current()
        self.kickerUpperApplied = self.kickerUpperMotor.get_motor_voltage()

        # Leader control-critical signals at full rate
        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.spindexer1Position,
            self.spindexer1Velocity,
            self.kickerUpperPosition,
            self.kickerUpperVelocity,
        )
        # Leader diagnostic signals at reduced rate (logging only)
        BaseStatusSignal.set_update_frequency_for_all(
            kRobotDiagnosticUpdateFrequency,
            self.spindexer1Supply,
            self.spindexer1Applied,
            self.kickerUpperSupply,
            self.kickerUpperApplied,
        )
        # Follower signals at reduced rate (redundant with leader, diagnostic only)
        BaseStatusSignal.set_update_frequency_for_all(
            kRobotDiagnosticUpdateFrequency,
            self.spindexer2Position,
            self.spindexer2Velocity,
            self.spindexer2Supply,
            self.spindexer2Applied,
            self.kickerLowerPosition,
            self.kickerLowerVelocity,
            self.kickerLowerSupply,
            self.kickerLowerApplied,
        )

        PhoenixUtil.registerSignals(
            kRioCANBus,
            self.spindexer1Position,
            self.spindexer1Velocity,
            self.spindexer1Supply,
            self.spindexer1Applied,
            self.spindexer2Position,
            self.spindexer2Velocity,
            self.spindexer2Supply,
            self.spindexer2Applied,
            self.kickerLowerPosition,
            self.kickerLowerVelocity,
            self.kickerLowerSupply,
            self.kickerLowerApplied,
            self.kickerUpperPosition,
            self.kickerUpperVelocity,
            self.kickerUpperSupply,
            self.kickerUpperApplied,
        )
        self.spindexer1Motor.optimize_bus_utilization()
        self.spindexer2Motor.optimize_bus_utilization()
        self.kickerUpperMotor.optimize_bus_utilization()
        self.kickerLowerMotor.optimize_bus_utilization()

    def updateInputs(self, inputs: IndexerSubsystemIO.IndexerSubsystemInputs) -> None:
        inputs.spindexer1Connected = BaseStatusSignal.is_all_good(
            self.spindexer1Position,
            self.spindexer1Velocity,
            self.spindexer1Applied,
            self.spindexer1Supply,
        )

        inputs.spindexer1Position = self.spindexer1Position.value
        inputs.spindex1Velocity = self.spindexer1Velocity.value
        inputs.spindex1AppliedVolts = self.spindexer1Applied.value
        inputs.spindex1SupplyAmps = self.spindexer1Supply.value

        inputs.spindexer2Connected = BaseStatusSignal.is_all_good(
            self.spindexer2Position,
            self.spindexer2Velocity,
            self.spindexer2Applied,
            self.spindexer2Supply,
        )

        inputs.spindexer2Position = self.spindexer2Position.value
        inputs.spindex2Velocity = self.spindexer2Velocity.value
        inputs.spindex2AppliedVolts = self.spindexer2Applied.value
        inputs.spindex2SupplyAmps = self.spindexer2Supply.value

        inputs.kickLowerConnected = BaseStatusSignal.is_all_good(
            self.kickerLowerPosition,
            self.kickerLowerVelocity,
            self.kickerLowerApplied,
            self.kickerLowerSupply,
        )
        inputs.kickLowerPosition = self.kickerLowerPosition.value
        inputs.kickLowerVelocity = self.kickerLowerVelocity.value
        inputs.kickLowerAppliedVolts = self.kickerLowerApplied.value
        inputs.kickLowerSupplyAmps = self.kickerLowerSupply.value

        inputs.kickUpperConnected = BaseStatusSignal.is_all_good(
            self.kickerUpperPosition,
            self.kickerUpperVelocity,
            self.kickerUpperApplied,
            self.kickerUpperSupply,
        )
        inputs.kickUpperPosition = self.kickerUpperPosition.value
        inputs.kickUpperVelocity = self.kickerUpperVelocity.value
        inputs.kickUpperAppliedVolts = self.kickerUpperApplied.value
        inputs.kickUpperSupplyAmps = self.kickerUpperSupply.value

    def setIndexerTarget(
        self,
        spindexer1: float,
        kickerUpper: float,
    ) -> None:
        self.spindexer1Motor.set_control(self.spindexerDemand.with_output(spindexer1))
        self.kickerUpperMotor.set_control(self.kickerDemand.with_output(kickerUpper))
