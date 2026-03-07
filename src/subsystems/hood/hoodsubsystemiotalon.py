from phoenix6.configs.talon_fx_configs import (
    MotionMagicConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import MotionMagicVoltage, PositionVoltage, VoltageOut
from phoenix6 import BaseStatusSignal
from phoenix6.hardware.talon_fx import TalonFX
from wpimath.geometry import Rotation2d
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
    kHoodInvertedValue,
)
from constants.math import kRadiansPerRevolution
from constants import kRobotUpdateFrequency, kRioCANBus
from util.phoenixutil import PhoenixUtil, tryUntilOk


class HoodSubsystemIOTalon(HoodSubsystemIO):
    hoodConfig: TalonFXConfiguration = TalonFXConfiguration()

    closedDemand: PositionVoltage = PositionVoltage(0)
    openDemand: VoltageOut = VoltageOut(0)

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
        self.hoodConfig.motor_output.inverted = kHoodInvertedValue
        tryUntilOk(5, lambda: self.motor.configurator.apply(self.hoodConfig))

        self.position = self.motor.get_position()
        self.velocity = self.motor.get_velocity()

        self.applied = self.motor.get_motor_voltage()
        self.supply = self.motor.get_supply_current()

        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.position,
            self.velocity,
            self.applied,
            self.supply,
        )
        PhoenixUtil.registerSignals(
            kRioCANBus, self.position, self.velocity, self.applied, self.supply
        )

    def updateInputs(self, inputs: HoodSubsystemIO.HoodSubsystemIOInputs) -> None:
        inputs.hoodConnected = BaseStatusSignal.is_all_good(
            self.position, self.velocity, self.applied, self.supply
        )
        inputs.hoodPosition = Rotation2d.fromRotations(self.position.value)
        inputs.hoodSpeed = self.velocity.value * kRadiansPerRevolution
        inputs.hoodAppliedVolts = self.applied.value
        inputs.hoodSupplyAmps = self.supply.value

    def set_hood_target(self, position: Rotation2d):
        self.motor.set_control(
            self.closedDemand.with_position(position.radians() / kRadiansPerRevolution)
        )

    def set_hood_position(self, position: Rotation2d):
        self.motor.set_position(position.radians() / kRadiansPerRevolution)

    def set_hood_volts(self, volts: float):
        self.motor.set_control(self.openDemand.with_output(volts))
