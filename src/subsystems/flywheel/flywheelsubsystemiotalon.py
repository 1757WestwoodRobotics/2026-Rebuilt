from phoenix6 import BaseStatusSignal
from phoenix6.configs.talon_fx_configs import (
    MotionMagicConfigs,
    NeutralModeValue,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import MotionMagicVelocityVoltage, NeutralOut, VoltageOut
from phoenix6.hardware.talon_fx import TalonFX
from subsystems.flywheel.flywheelsubsystemio import FlywheelSubsystemIO

from constants import kRobotUpdateFrequency, kRioCANBus, kRobotDiagnosticUpdateFrequency
from constants.math import kRadiansPerRevolution
from constants.flywheel import (
    kFlywheelCANId,
    kFlywheelGearing,
    kFlywheelPGain,
    kFlywheelIGain,
    kFlywheelDGain,
    kFlywheelSGain,
    kFlywheelVGain,
    kFlywheelAGain,
    kFlywheelCurrentLimit,
    kFlywheelMaxAcceleration,
    kFlywheelMaxVelocity,
)
from util.phoenixutil import PhoenixUtil, tryUntilOk


class FlywheelSubsystemIOTalon(FlywheelSubsystemIO):
    config: TalonFXConfiguration = TalonFXConfiguration()

    closedLoopDemand: MotionMagicVelocityVoltage = MotionMagicVelocityVoltage(0, 0)
    openLoopDemand: VoltageOut = VoltageOut(0)
    neutralOutput: NeutralOut = NeutralOut()

    def __init__(self) -> None:
        self.motor = TalonFX(kFlywheelCANId)

        self.config.motor_output.neutral_mode = NeutralModeValue.COAST

        self.config.current_limits = kFlywheelCurrentLimit
        self.config.feedback.sensor_to_mechanism_ratio = kFlywheelGearing
        self.config.slot0 = (
            Slot0Configs()
            .with_k_p(kFlywheelPGain)
            .with_k_i(kFlywheelIGain)
            .with_k_d(kFlywheelDGain)
            .with_k_s(kFlywheelSGain)
            .with_k_v(kFlywheelVGain)
            .with_k_a(kFlywheelAGain)
        )
        self.config.motion_magic = (
            MotionMagicConfigs()
            .with_motion_magic_expo_k_a(kFlywheelAGain * kRadiansPerRevolution)
            .with_motion_magic_expo_k_v(kFlywheelVGain * kRadiansPerRevolution)
            .with_motion_magic_acceleration(
                kFlywheelMaxAcceleration / kRadiansPerRevolution
            )
            .with_motion_magic_cruise_velocity(
                kFlywheelMaxVelocity / kRadiansPerRevolution
            )
        )

        tryUntilOk(5, lambda: self.motor.configurator.apply(self.config))

        self.flywheelPosition = self.motor.get_position()
        self.flywheelVelocity = self.motor.get_velocity()

        self.flywheelApplied = self.motor.get_motor_voltage()
        self.flywheelSupply = self.motor.get_supply_current()

        # Control-critical signals at full rate
        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.flywheelPosition,
            self.flywheelVelocity,
        )
        # Diagnostic signals at reduced rate (logging only)
        BaseStatusSignal.set_update_frequency_for_all(
            kRobotDiagnosticUpdateFrequency,
            self.flywheelApplied,
            self.flywheelSupply,
        )
        PhoenixUtil.registerSignals(
            kRioCANBus,
            self.flywheelPosition,
            self.flywheelVelocity,
            self.flywheelApplied,
            self.flywheelSupply,
        )
        self.motor.optimize_bus_utilization()

    def updateInputs(
        self, inputs: FlywheelSubsystemIO.FlywheelSubsystemIOInputs
    ) -> None:
        inputs.flywheelConnected = BaseStatusSignal.is_all_good(
            self.flywheelPosition,
            self.flywheelVelocity,
            self.flywheelApplied,
            self.flywheelSupply,
        )

        inputs.flywheelPosition = self.flywheelPosition.value * kRadiansPerRevolution
        inputs.flywheelSpeed = self.flywheelVelocity.value * kRadiansPerRevolution

        inputs.flywheelAppliedVolts = self.flywheelApplied.value
        inputs.flywheelSupplyAmps = self.flywheelSupply.value

    def set_speed(self, speed: float):
        self.motor.set_control(
            self.closedLoopDemand.with_velocity(speed / kRadiansPerRevolution)
        )

    def set_volts(self, volts: float):
        self.motor.set_control(self.openLoopDemand.with_output(volts))

    def neutral_output(self):
        self.motor.set_control(self.neutralOutput)
