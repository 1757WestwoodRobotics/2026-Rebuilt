from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import (
    GravityTypeValue,
    MotionMagicConfigs,
    NeutralModeValue,
    Slot0Configs,
)
from phoenix6.controls import MotionMagicVoltage, VoltageOut
from phoenix6.hardware.talon_fx import TalonFX
from wpimath.geometry import Rotation2d
from subsystems.intake.intakesubsystemio import IntakeSubsystemIO

from constants.intake import (
    kPivotCANId,
    kRollerCANId,
    kPivotKp,
    kPivotKi,
    kPivotKd,
    kPivotKs,
    kPivotKv,
    kPivotKa,
    kPivotKg,
    kPivotCurrentLimit,
    kPivotGearRatio,
    kPivotMaxVelocity,
    kPivotMaxAcceleration,
    kPivotMinAngle,
    kPivotMaxAngle,
    kRollerGearRatio,
    kRollerCurrentLimit,
)
from constants.math import kRadiansPerRevolution

from constants import kRobotUpdateFrequency
from util.convenientmath import clampRotation
from util.phoenixutil import PhoenixUtil, tryUntilOk


class IntakeSubsystemIOTalon(IntakeSubsystemIO):
    pivotConfig: TalonFXConfiguration = TalonFXConfiguration()
    rollerConfig: TalonFXConfiguration = TalonFXConfiguration()

    pivotDemand: MotionMagicVoltage = MotionMagicVoltage(0)
    pivotVoltageRequest: VoltageOut = VoltageOut(0)

    rollerDemand: VoltageOut = VoltageOut(0)

    def __init__(self) -> None:
        self.pivotMotor = TalonFX(kPivotCANId)
        self.rollerMotor = TalonFX(kRollerCANId)

        self.pivotConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.pivotConfig.current_limits = kPivotCurrentLimit
        self.pivotConfig.feedback.sensor_to_mechanism_ratio = kPivotGearRatio
        self.pivotConfig.slot0 = (
            Slot0Configs()
            .with_k_p(kPivotKp)
            .with_k_i(kPivotKi)
            .with_k_d(kPivotKd)
            .with_k_v(kPivotKv)
            .with_k_s(kPivotKs)
            .with_k_a(kPivotKa)
            .with_k_g(kPivotKg)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )
        self.pivotConfig.motion_magic = (
            MotionMagicConfigs()
            .with_motion_magic_expo_k_v(kPivotKv * kRadiansPerRevolution)
            .with_motion_magic_expo_k_a(kPivotKa * kRadiansPerRevolution)
            .with_motion_magic_acceleration(
                kPivotMaxAcceleration / kRadiansPerRevolution
            )
            .with_motion_magic_cruise_velocity(
                kPivotMaxVelocity / kRadiansPerRevolution
            )
        )
        tryUntilOk(5, lambda: self.pivotMotor.configurator.apply(self.pivotConfig))

        self.rollerConfig.current_limits = kRollerCurrentLimit
        self.rollerConfig.feedback.sensor_to_mechanism_ratio = kRollerGearRatio
        tryUntilOk(5, lambda: self.rollerMotor.configurator.apply(self.rollerConfig))

        self.pivotPosition = self.pivotMotor.get_position()
        self.pivotVelocity = self.pivotMotor.get_velocity()
        self.pivotApplied = self.pivotMotor.get_motor_voltage()
        self.pivotSupply = self.pivotMotor.get_supply_current()

        self.rollerPosition = self.rollerMotor.get_position()
        self.rollerVelocity = self.rollerMotor.get_velocity()
        self.rollerApplied = self.rollerMotor.get_motor_voltage()
        self.rollerSupply = self.rollerMotor.get_supply_current()

        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.pivotPosition,
            self.pivotVelocity,
            self.pivotApplied,
            self.pivotSupply,
            self.rollerPosition,
            self.rollerVelocity,
            self.rollerApplied,
            self.rollerSupply,
        )
        PhoenixUtil.registerSignals(
            "",
            self.pivotPosition,
            self.pivotVelocity,
            self.pivotApplied,
            self.pivotSupply,
            self.rollerPosition,
            self.rollerVelocity,
            self.rollerApplied,
            self.rollerSupply,
        )

    def updateInputs(self, inputs: IntakeSubsystemIO.IntakeSubsystemInputs):
        inputs.pivotConnected = BaseStatusSignal.is_all_good(
            self.pivotPosition,
            self.pivotVelocity,
            self.pivotApplied,
            self.pivotSupply,
        )
        inputs.rollerConnected = BaseStatusSignal.is_all_good(
            self.rollerPosition,
            self.rollerVelocity,
            self.rollerApplied,
            self.rollerSupply,
        )

        inputs.pivotPosition = self.pivotPosition.value * kRadiansPerRevolution
        inputs.pivotVelocity = self.pivotVelocity.value * kRadiansPerRevolution
        inputs.rollerPosition = self.rollerPosition.value * kRadiansPerRevolution
        inputs.rollerVelocity = self.rollerVelocity.value * kRadiansPerRevolution

        inputs.pivotAppliedVolts = self.pivotApplied.value
        inputs.pivotSupplyAmps = self.pivotSupply.value

        inputs.rollerAppliedVolts = self.rollerApplied.value
        inputs.rollerSupplyAmps = self.rollerSupply.value

    def setIntakeAngle(self, position: Rotation2d) -> None:
        self.pivotMotor.set_position(position.radians() / kRadiansPerRevolution)

    def setIntakeTarget(self, roller: float, position: Rotation2d):
        clampedOutput = clampRotation(position, kPivotMinAngle, kPivotMaxAngle)
        self.rollerMotor.set_control(self.rollerDemand.with_output(roller))
        self.pivotMotor.set_control(
            self.pivotDemand.with_position(
                clampedOutput.radians() / kRadiansPerRevolution
            )
        )

    def setPivotVolts(self, volt: float) -> None:
        self.pivotMotor.set_control(self.pivotVoltageRequest.with_output(volt))
