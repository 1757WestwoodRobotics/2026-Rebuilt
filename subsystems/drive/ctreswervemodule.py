import typing
from phoenix6.sim.cancoder_sim_state import CANcoderSimState
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState
from wpilib import DataLogManager, RobotBase
from wpimath.geometry import Rotation2d
from subsystems.drive.swervemodule import SwerveModule, SwerveModuleConfigParams
from util.simcoder import CTREEncoder
from util.simtalon import Talon

from constants.drive import (
    kDrivePGain,
    kDriveIGain,
    kDriveDGain,
    kDriveVGain,
    kSteerPGain,
    kSteerIGain,
    kSteerDGain,
    kDriveCurrentLimit,
    kSteerGearingRatio,
    kDriveGearingRatio,
    kWheelRadius,
)

from constants.math import kRadiansPerRevolution


class CTRESwerveModule(SwerveModule):
    """
    Implementation of SwerveModule for the SDS swerve modules
    https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module
        driveMotor: Kraken X60 Motor (with built-in encoder) attached to wheel through gearing
        steerMotor: Falcon 500 Motor (with built-in encoder) attached to swerve through gearing
        swerveEncoder: CANCoder
    """

    def __init__(self, name: str, config: SwerveModuleConfigParams) -> None:
        SwerveModule.__init__(self, name)
        DataLogManager.log(f"Initializing swerve module: {self.name}")
        DataLogManager.log(f"   Configuring drive motor: CAN ID: {config.driveMotorID}")
        self.driveMotor = Talon(
            config.driveMotorID,
            f"Drive Motor {name}",
            kDrivePGain,
            kDriveIGain,
            kDriveDGain,
            config.driveMotorInverted,
            config.canbus,
            kDriveVGain,
        )
        self.driveMotor.setNeutralMode(Talon.NeutralMode.Brake)
        if RobotBase.isReal():
            self.driveMotor.setCurrentLimit(kDriveCurrentLimit)
        DataLogManager.log("   ... Done")
        DataLogManager.log(f"   Configuring steer motor: CAN ID: {config.steerMotorID}")
        self.steerMotor = Talon(
            config.steerMotorID,
            f"Steer Motor {name}",
            kSteerPGain,
            kSteerIGain,
            kSteerDGain,
            config.steerMotorInverted,
            config.canbus,
        )
        DataLogManager.log("   ... Done")
        DataLogManager.log(
            f"   Configuring swerve encoder: CAN ID: {config.swerveEncoderID}"
        )
        self.swerveEncoder = CTREEncoder(
            config.swerveEncoderID, config.swerveEncoderOffset, config.canbus
        )
        DataLogManager.log("   ... Done")
        DataLogManager.log("... Done")

    def getSwerveAngle(self) -> Rotation2d:
        steerRotation = self.steerMotor.get(Talon.ControlMode.Position)
        swerveAngle = steerRotation / kSteerGearingRatio * kRadiansPerRevolution
        return Rotation2d(swerveAngle)

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        steerEncoderPulses = (
            (swerveAngle.radians()) / kRadiansPerRevolution * kSteerGearingRatio
        )
        self.steerMotor.setEncoderPosition(steerEncoderPulses)

    def getSwerveEncoderAngle(self) -> Rotation2d:
        return self.swerveEncoder.getPosition()

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        steerEncoderPulsesTarget = (
            swerveAngleTarget.radians() / kRadiansPerRevolution * kSteerGearingRatio
        )
        self.steerMotor.set(Talon.ControlMode.Position, steerEncoderPulsesTarget)

    def getWheelLinearVelocity(self) -> float:
        driveEncoderPulsesPerSecond = self.driveMotor.get(Talon.ControlMode.Velocity)
        wheelLinearVelocity = (
            driveEncoderPulsesPerSecond
            * kWheelRadius
            * kRadiansPerRevolution
            / kDriveGearingRatio
        )
        return wheelLinearVelocity

    def getWheelTotalPosition(self) -> float:
        driveEncoderPulses = self.driveMotor.get(Talon.ControlMode.Position)
        driveDistance = (
            driveEncoderPulses
            * kWheelRadius
            * kRadiansPerRevolution
            / kDriveGearingRatio
        )
        return driveDistance

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        driveEncoderPulsesPerSecond = (
            wheelLinearVelocityTarget
            / kWheelRadius
            / kRadiansPerRevolution
            * kDriveGearingRatio
        )
        self.driveMotor.set(
            Talon.ControlMode.Velocity,
            driveEncoderPulsesPerSecond,
        )

    def reset(self) -> None:
        if RobotBase.isReal():
            self.setSwerveAngle(self.swerveEncoder.getPosition())

    def getSimulator(
        self,
    ) -> tuple[
        typing.Callable[[], TalonFXSimState],
        typing.Callable[[], TalonFXSimState],
        typing.Callable[[], CANcoderSimState],
    ]:
        return (
            self.driveMotor.getSimCollection,
            self.steerMotor.getSimCollection,
            self.swerveEncoder.getSim,
        )
