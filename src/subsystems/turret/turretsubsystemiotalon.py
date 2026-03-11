from phoenix6 import BaseStatusSignal
from phoenix6.configs.talon_fx_configs import (
    MotionMagicConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware.talon_fx import TalonFX
from wpimath.geometry import Rotation2d

from subsystems.turret.turretsubsystemio import TurretSubsystemIO

from constants.turret import (
    kTurretCanId,
    kTurretCurrentLimit,
    kTurretGearRatio,
    kTurretMaxAcceleration,
    kTurretMaxVelocity,
    kTurretPGain,
    kTurretIGain,
    kTurretDGain,
    kTurretSGain,
    kTurretVGain,
    kTurretAGain,
    kTurretInvertedValue,
)
from constants.math import kRadiansPerRevolution
from constants import kRobotUpdateFrequency, kRioCANBus
from util.phoenixutil import PhoenixUtil, tryUntilOk


class TurretSubsystemIOTalon(TurretSubsystemIO):
    turretConfig: TalonFXConfiguration = TalonFXConfiguration()

    closedDemand: PositionVoltage = PositionVoltage(0)
    openDemand: VoltageOut = VoltageOut(0)

    def __init__(self) -> None:
        """Initialize the Talon motor with appropriate robot-specific parameters and record starting values."""
        self.motor = TalonFX(kTurretCanId)

        self.turretConfig.current_limits = kTurretCurrentLimit
        self.turretConfig.feedback.sensor_to_mechanism_ratio = kTurretGearRatio
        self.turretConfig.slot0 = (
            Slot0Configs()
            .with_k_p(kTurretPGain)
            .with_k_i(kTurretIGain)
            .with_k_d(kTurretDGain)
            .with_k_s(kTurretSGain)
            .with_k_v(kTurretVGain)
            .with_k_a(kTurretAGain)
        )
        self.turretConfig.motion_magic = (
            MotionMagicConfigs()
            .with_motion_magic_expo_k_a(kTurretAGain)
            .with_motion_magic_expo_k_v(kTurretVGain)
            .with_motion_magic_acceleration(
                kTurretMaxAcceleration.radians() / kRadiansPerRevolution
            )
            .with_motion_magic_cruise_velocity(
                kTurretMaxVelocity.radians() / kRadiansPerRevolution
            )
        )
        self.turretConfig.motor_output.inverted = kTurretInvertedValue
        tryUntilOk(5, lambda: self.motor.configurator.apply(self.turretConfig))

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

    def updateInputs(self, inputs: TurretSubsystemIO.TurretSubsystemIOInputs):
        """Update state of motor per the appropriate specifc API."""
        inputs.turretConnected = BaseStatusSignal.is_all_good(
            self.position,
            self.velocity,
            self.applied,
            self.supply,
        )

        inputs.turretPosition = Rotation2d(self.position.value * kRadiansPerRevolution)
        inputs.turretSpeed = self.velocity.value * kRadiansPerRevolution
        inputs.turretAppliedVolts = self.applied.value
        inputs.turretSupplyAmps = self.supply.value

    def set_turret_angle(self, position: Rotation2d, velocity: float):
        """Move the motor a specified amount of radians."""
        self.motor.set_control(
            self.closedDemand.with_position(
                position.radians() / kRadiansPerRevolution
            ).with_velocity(velocity / kRadiansPerRevolution)
        )

    def set_turret_volts(self, volts: float):
        """Move the motor by applying a specific amount of volts."""
        self.motor.set_control(self.openDemand.with_output(volts))

    def set_turret_position(self, position: Rotation2d):
        self.motor.set_position(position.radians() / kRadiansPerRevolution)
