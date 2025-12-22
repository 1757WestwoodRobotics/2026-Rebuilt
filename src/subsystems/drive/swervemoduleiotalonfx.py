import typing
from phoenix6 import BaseStatusSignal
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration
from phoenix6.configs.talon_fx_configs import (
    InvertedValue,
    NeutralModeValue,
    Slot0Configs,
    StaticFeedforwardSignValue,
)
from phoenix6.controls import (
    MotionMagicTorqueCurrentFOC,
    VelocityTorqueCurrentFOC,
)
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.sim.cancoder_sim_state import CANcoderSimState
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState
from pykit.logger import Logger
from wpimath.geometry import Rotation2d
from subsystems.drive.swervemoduleio import SwerveModuleIO, SwerveModuleConfigParams
from util.phoenixutil import PhoenixUtil, tryUntilOk

from constants import kRobotUpdateFrequency, kRobotUpdatePeriod
from constants.drive import (
    kDrivePGain,
    kDriveIGain,
    kDriveDGain,
    kDriveVGain,
    kSteerPGain,
    kSteerIGain,
    kSteerDGain,
    kSteerVGain,
    kSteerSGain,
    kDriveCurrentLimit,
    kSteerCurrentLimit,
    kSteerGearingRatio,
    kDriveGearingRatio,
)

from constants.math import kRadiansPerRevolution


class SwerveModuleIOCTRE(SwerveModuleIO):
    """
    Implementation of SwerveModule for the SDS swerve modules
    https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module
        driveMotor: Kraken X60 Motor (with built-in encoder) attached to wheel through gearing
        steerMotor: Falcon 500 Motor (with built-in encoder) attached to swerve through gearing
        swerveEncoder: CANCoder
    """

    driveConfig: TalonFXConfiguration = TalonFXConfiguration()
    steerConfig: TalonFXConfiguration = TalonFXConfiguration()

    steerRequest: MotionMagicTorqueCurrentFOC = MotionMagicTorqueCurrentFOC(0)
    driveRequest: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0)

    def __init__(self, name: str, config: SwerveModuleConfigParams) -> None:
        SwerveModuleIO.__init__(self, name)
        print(f"Initializing swerve module: {self.name}")
        self.driveMotor = TalonFX(config.driveMotorID, config.canbus)
        self.steerMotor = TalonFX(config.steerMotorID, config.canbus)
        self.swerveEncoder = CANcoder(config.swerveEncoderID, config.canbus)

        self.encoderOffset = config.swerveEncoderOffset

        self.driveConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self.driveConfig.slot0 = (
            Slot0Configs()
            .with_k_p(kDrivePGain)
            .with_k_i(kDriveIGain)
            .with_k_d(kDriveDGain)
            .with_k_v(kDriveVGain)
        )
        self.driveConfig.feedback.sensor_to_mechanism_ratio = kDriveGearingRatio
        self.driveConfig.current_limits = kDriveCurrentLimit
        self.driveConfig.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if config.driveMotorInverted
            else InvertedValue.CLOCKWISE_POSITIVE
        )
        tryUntilOk(
            5, lambda: self.driveMotor.configurator.apply(self.driveConfig, 0.25)
        )
        tryUntilOk(5, lambda: self.driveMotor.set_position(0, 0.25))

        self.steerConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        self.steerConfig.slot0 = (
            Slot0Configs()
            .with_k_p(kSteerPGain)
            .with_k_i(kSteerIGain)
            .with_k_d(kSteerDGain)
            .with_k_v(kSteerVGain)
            .with_k_s(kSteerSGain)
            .with_k_a(0)
            .with_static_feedforward_sign(
                StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            )
        )
        self.steerConfig.torque_current.peak_forward_torque_current = 40
        self.steerConfig.torque_current.peak_reverse_torque_current = 40
        self.steerConfig.motion_magic.motion_magic_cruise_velocity = (
            100 / kSteerGearingRatio
        )
        self.steerConfig.motion_magic.motion_magic_acceleration = (
            self.steerConfig.motion_magic.motion_magic_cruise_velocity / 0.1
        )
        self.steerConfig.motion_magic.motion_magic_expo_k_v = 0.12 * kSteerGearingRatio
        self.steerConfig.motion_magic.motion_magic_expo_k_a = 0.1
        self.steerConfig.closed_loop_ramps.torque_closed_loop_ramp_period = (
            kRobotUpdatePeriod
        )
        # optionally fuse the cancoder to the steer motor
        self.steerConfig.feedback.sensor_to_mechanism_ratio = kSteerGearingRatio
        self.steerConfig.closed_loop_general.continuous_wrap = True
        self.steerConfig.current_limits = kSteerCurrentLimit
        self.steerConfig.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if config.steerMotorInverted
            else InvertedValue.CLOCKWISE_POSITIVE
        )
        tryUntilOk(
            5, lambda: self.steerMotor.configurator.apply(self.steerConfig, 0.25)
        )

        self.cancoderConfig = CANcoderConfiguration()
        self.cancoderConfig.magnet_sensor.magnet_offset = -config.swerveEncoderOffset
        self.cancoderConfig.magnet_sensor.absolute_sensor_discontinuity_point = 0.5

        tryUntilOk(
            5, lambda: self.swerveEncoder.configurator.apply(self.cancoderConfig, 0.25)
        )
        tryUntilOk(
            5,
            lambda: self.steerMotor.set_position(
                self.swerveEncoder.get_position().value, 0.25
            ),
        )

        self.drivePosition = self.driveMotor.get_position()
        self.driveVelocity = self.driveMotor.get_velocity()
        self.driveApplied = self.driveMotor.get_motor_voltage()
        self.driveSupplyCurrent = self.driveMotor.get_supply_current()
        self.driveTorqueCurrent = self.driveMotor.get_torque_current()

        self.steerPosition = self.steerMotor.get_position()
        self.steerVelocity = self.steerMotor.get_velocity()
        self.steerApplied = self.steerMotor.get_motor_voltage()
        self.steerSupplyCurrent = self.steerMotor.get_supply_current()
        self.steerTorqueCurrent = self.steerMotor.get_torque_current()

        self.steerAbsolutePosition = self.swerveEncoder.get_absolute_position()

        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.drivePosition,
            self.driveVelocity,
            self.driveApplied,
            self.driveSupplyCurrent,
            self.driveTorqueCurrent,
            self.steerPosition,
            self.steerVelocity,
            self.steerApplied,
            self.steerSupplyCurrent,
            self.steerTorqueCurrent,
            self.steerAbsolutePosition,
        )

        PhoenixUtil.registerSignals(
            config.canbus,
            self.drivePosition,
            self.driveVelocity,
            self.driveApplied,
            self.driveSupplyCurrent,
            self.driveTorqueCurrent,
            self.steerPosition,
            self.steerVelocity,
            self.steerApplied,
            self.steerSupplyCurrent,
            self.steerTorqueCurrent,
            self.steerAbsolutePosition,
        )

        print("... Done")

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        steerEncoderPulses = swerveAngle.radians() / kRadiansPerRevolution
        self.steerMotor.set_position(steerEncoderPulses)

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        steerEncoderTarget = swerveAngleTarget.radians() / kRadiansPerRevolution
        Logger.recordOutput(f"Drive/{self.name}/AngleTarget", steerEncoderTarget)
        self.steerMotor.set_control(self.steerRequest.with_position(steerEncoderTarget))

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        driveEncoderTarget = wheelLinearVelocityTarget / kRadiansPerRevolution
        Logger.recordOutput(f"Drive/{self.name}/DriveTarget", driveEncoderTarget)
        self.driveMotor.set_control(self.driveRequest.with_velocity(driveEncoderTarget))

    def getSimulator(
        self,
    ) -> tuple[
        typing.Callable[[], TalonFXSimState],
        typing.Callable[[], TalonFXSimState],
        typing.Callable[[], CANcoderSimState],
    ]:
        return (
            (lambda: self.driveMotor.sim_state),
            (lambda: self.steerMotor.sim_state),
            (lambda: self.swerveEncoder.sim_state),
        )

    def updateInputs(self, inputs: SwerveModuleIO.SwerveModuleIOInputs) -> None:
        inputs.driveconnected = BaseStatusSignal.is_all_good(
            self.drivePosition,
            self.driveVelocity,
            self.driveApplied,
            self.driveSupplyCurrent,
            self.driveTorqueCurrent,
        )
        inputs.steerconnected = BaseStatusSignal.is_all_good(
            self.steerPosition,
            self.steerVelocity,
            self.steerApplied,
            self.steerSupplyCurrent,
            self.steerTorqueCurrent,
        )
        inputs.encoderconnected = BaseStatusSignal.is_all_good(
            self.steerAbsolutePosition
        )

        inputs.drive_position = self.drivePosition.value * kRadiansPerRevolution
        inputs.drive_velocity = self.driveVelocity.value * kRadiansPerRevolution
        inputs.drive_applied = self.driveApplied.value
        inputs.drive_supply_current = self.driveSupplyCurrent.value
        inputs.drive_torque_current = self.driveTorqueCurrent.value

        inputs.turn_position = self.steerPosition.value * kRadiansPerRevolution
        inputs.turn_velocity = self.steerVelocity.value * kRadiansPerRevolution
        inputs.turn_applied = self.steerApplied.value
        inputs.turn_supply_current = self.steerSupplyCurrent.value
        inputs.turn_torque_current = self.steerTorqueCurrent.value

        inputs.turn_absolute_position = (
            self.steerAbsolutePosition.value * kRadiansPerRevolution
        )
