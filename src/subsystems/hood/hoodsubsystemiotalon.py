from phoenix6.configs.talon_fx_configs import (
    MotionMagicConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import MotionMagicVoltage, VoltageOut
from phoenix6.hardware.core.core_talon_fx import BaseStatusSignal
from phoenix6.hardware.talon_fx import TalonFX
from subsystems.hood.hoodsubsystemio import HoodSubsystemIO

from constants.hood import (
    kHoodCANId,
    kHoodCurrentLimit,
    kHoodGearRatio,
    kHoodMaxAcceleration,
    kHoodMaxVelocity,
    kHoodPGain,
    kHoodIGain,
    kHoodDGain,
    kHoodSGain,
    kHoodVGain,
    kHoodAGain,
)
from constants.math import kRadiansPerRevolution
from constants import kRobotUpdateFrequency
from util.phoenixutil import PhoenixUtil, tryUntilOk


class HoodSubsystemIOTalon(HoodSubsystemIO):
    hoodConfig: TalonFXConfiguration = TalonFXConfiguration()

    closedDemand: MotionMagicVoltage = MotionMagicVoltage(0, False)
    openDemand: VoltageOut = VoltageOut(0, False)

    def __init__(self) -> None:
        self.motor = TalonFX(kHoodCANId)

        self.hoodConfig.current_limits = kHoodCurrentLimit
        self.hoodConfig.feedback.sensor_to_mechanism_ratio = kHoodGearRatio
        self.hoodConfig.slot0 = (
            Slot0Configs()
            .with_k_p(kHoodPGain)
            .with_k_i(kHoodIGain)
            .with_k_d(kHoodDGain)
            .with_k_s(kHoodSGain)
            .with_k_v(kHoodVGain)
            .with_k_a(kHoodAGain)
        )
        self.hoodConfig.motion_magic = (
            MotionMagicConfigs()
            .with_motion_magic_expo_k_a(kHoodAGain)
            .with_motion_magic_expo_k_v(kHoodVGain)
            .with_motion_magic_acceleration(
                kHoodMaxAcceleration.radians() / kRadiansPerRevolution
            )
            .with_motion_magic_cruise_velocity(
                kHoodMaxVelocity.radians() / kRadiansPerRevolution
            )
        )
        tryUntilOk(5, lambda: self.motor.configurator.apply(self.hoodConfig))

        self.position = self.motor.get_position()
        self.velocity = self.motor.get_velocity()

        self.appled = self.motor.get_motor_voltage()
        self.supply = self.motor.get_supply_current()

        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.position,
            self.velocity,
            self.appled,
            self.supply,
        )
        PhoenixUtil.registerSignals(
            "", self.position, self.velocity, self.appled, self.supply
        )

    def updateInputs(self, inputs: HoodSubsystemIO.HoodSubsystemIOInputs) -> None:
        inputs.hoodConnected = BaseStatusSignal.is_all_good(
            self.position, self.velocity, self.appled, self.supply
        )
        inputs.hoodPosition = self.position.value * kRadiansPerRevolution
        inputs.hoodSpeed = self.velocity.value * kRadiansPerRevolution
        inputs.hoodAppliedVolts = self.appled.value
        inputs.hoodSupplyAmps = self.supply.value

    def set_hood_position(self, position: float):
        self.motor.set_control(
            self.closedDemand.with_position(position / kRadiansPerRevolution)
        )

    def set_hood_volts(self, volts: float):
        self.motor.set_control(self.openDemand.with_output(volts))
