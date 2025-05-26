from enum import Enum, auto

from typing import Tuple
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from pathplannerlib.path import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder

from phoenix6.configs.pigeon2_configs import Pigeon2Configuration
from phoenix6.hardware.pigeon2 import Pigeon2
from wpilib import (
    RobotBase,
    Timer,
    DriverStation,
)

from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
)
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

from subsystems.drive.ctreswervemodule import CTRESwerveModule
from subsystems.drive.robotposeestimator import OdometryObservation, RobotPoseEstimator
from subsystems.drive.swervemodule import SwerveModule, SwerveModuleConfigParams
from util import convenientmath

from constants.math import kRadiansPerDegree

from constants.logging import kSwerveExpectedStatesKey, kSwerveActualStatesKey

from constants.drive import (
    kFrontLeftModuleName,
    kFrontLeftDriveMotorId,
    kFrontLeftDriveInverted,
    kFrontLeftSteerMotorId,
    kFrontLeftSteerInverted,
    kFrontLeftSteerEncoderId,
    kFrontLeftAbsoluteEncoderOffset,
    kCANivoreName,
    kFrontRightModuleName,
    kFrontRightDriveMotorId,
    kFrontRightDriveInverted,
    kFrontRightSteerMotorId,
    kFrontRightSteerInverted,
    kFrontRightSteerEncoderId,
    kFrontRightAbsoluteEncoderOffset,
    kBackLeftModuleName,
    kBackLeftDriveMotorId,
    kBackLeftDriveInverted,
    kBackLeftSteerMotorId,
    kBackLeftSteerInverted,
    kBackLeftSteerEncoderId,
    kBackLeftAbsoluteEncoderOffset,
    kBackRightModuleName,
    kBackRightDriveMotorId,
    kBackRightDriveInverted,
    kBackRightSteerMotorId,
    kBackRightSteerInverted,
    kBackRightSteerEncoderId,
    kBackRightAbsoluteEncoderOffset,
    kFrontLeftWheelPosition,
    kFrontRightWheelPosition,
    kBackLeftWheelPosition,
    kBackRightWheelPosition,
    kPigeonCANId,
    kDriveAccelLimit,
    kRobotPoseArrayKeys,
    kDriveVelocityKeys,
    kMaxWheelLinearVelocity,
    kMaxForwardLinearVelocity,
    kMaxSidewaysLinearVelocity,
    kMaxRotationAngularVelocity,
    kDriveAngularVelocityCoeff,
)

from constants.sim import kSimRobotVelocityArrayKey
from constants.trajectory import (
    kPathFollowingTranslationConstantsAuto,
    kPathFollowingRotationConstants,
)
from constants import kRobotUpdatePeriod


# pylint: disable-next=too-many-instance-attributes
class DriveSubsystem(Subsystem):
    class CoordinateMode(Enum):
        RobotRelative = auto()
        FieldRelative = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.rotationOffset = 0

        self.frontLeftModule = CTRESwerveModule(
            kFrontLeftModuleName,
            SwerveModuleConfigParams(
                kFrontLeftDriveMotorId,
                kFrontLeftDriveInverted,
                kFrontLeftSteerMotorId,
                kFrontLeftSteerInverted,
                kFrontLeftSteerEncoderId,
                kFrontLeftAbsoluteEncoderOffset,
                kCANivoreName,
            ),
        )
        self.frontRightModule = CTRESwerveModule(
            kFrontRightModuleName,
            SwerveModuleConfigParams(
                kFrontRightDriveMotorId,
                kFrontRightDriveInverted,
                kFrontRightSteerMotorId,
                kFrontRightSteerInverted,
                kFrontRightSteerEncoderId,
                kFrontRightAbsoluteEncoderOffset,
                kCANivoreName,
            ),
        )
        self.backLeftModule = CTRESwerveModule(
            kBackLeftModuleName,
            SwerveModuleConfigParams(
                kBackLeftDriveMotorId,
                kBackLeftDriveInverted,
                kBackLeftSteerMotorId,
                kBackLeftSteerInverted,
                kBackLeftSteerEncoderId,
                kBackLeftAbsoluteEncoderOffset,
                kCANivoreName,
            ),
        )
        self.backRightModule = CTRESwerveModule(
            kBackRightModuleName,
            SwerveModuleConfigParams(
                kBackRightDriveMotorId,
                kBackRightDriveInverted,
                kBackRightSteerMotorId,
                kBackRightSteerInverted,
                kBackRightSteerEncoderId,
                kBackRightAbsoluteEncoderOffset,
                kCANivoreName,
            ),
        )

        self.modules = (
            self.frontLeftModule,
            self.frontRightModule,
            self.backLeftModule,
            self.backRightModule,
        )

        self.kinematics = SwerveDrive4Kinematics(
            kFrontLeftWheelPosition,
            kFrontRightWheelPosition,
            kBackLeftWheelPosition,
            kBackRightWheelPosition,
        )

        # Create the gyro, a sensor which can indicate the heading of the robot relative
        # to a customizable position.
        self.gyro = Pigeon2(kPigeonCANId, kCANivoreName)

        toApply = Pigeon2Configuration()
        self.gyro.configurator.apply(toApply)
        self.gyro.get_yaw().set_update_frequency(100)

        self.estimator = RobotPoseEstimator(
            self.kinematics,
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            Pose2d(),
            (0.05, 0.05, 5 * kRadiansPerDegree),
        )
        # standard deviations stolen from 2910

        # Create the an object for our odometry, which will utilize sensor data to
        # keep a record of our position on the field.
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            Pose2d(),
        )
        self.printTimer = Timer()
        self.printTimer.start()
        self.vxLimiter = SlewRateLimiter(kDriveAccelLimit)
        self.vyLimiter = SlewRateLimiter(kDriveAccelLimit)

        self.visionEstimate = Pose2d()

        self.pastTime = self.printTimer.get()

        self.config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose,
            self.resetDriveAtPosition,
            self.getRobotRelativeSpeeds,
            self.drivePathPlanned,
            PPHolonomicDriveController(
                kPathFollowingTranslationConstantsAuto,
                kPathFollowingRotationConstants,
            ),
            # controller
            self.config,
            # robot_config
            (lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed),
            self,
        )

        self.swerveStatePublisher = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic(kSwerveActualStatesKey, SwerveModuleState)
            .publish()
        )
        self.swerveStateExpectedPublisher = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic(kSwerveExpectedStatesKey, SwerveModuleState)
            .publish()
        )
        self.robotPosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(kRobotPoseArrayKeys.valueKey, Pose2d)
            .publish()
        )
        self.robotPoseValidPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(kRobotPoseArrayKeys.validKey)
            .publish()
        )
        self.robotPoseValidPublisher.set(True)

        self.driveVelocityPublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(kDriveVelocityKeys, ChassisSpeeds)
            .publish()
        )

        if RobotBase.isSimulation():
            self.simVelocityGetter = (
                NetworkTableInstance.getDefault()
                .getStructTopic(kSimRobotVelocityArrayKey, ChassisSpeeds)
                .subscribe(ChassisSpeeds())
            )

    def defenseState(self):
        def setModuleTo(module: SwerveModule, angle: Rotation2d):
            module.setWheelLinearVelocityTarget(0)
            module.setSwerveAngleTarget(module.optimizedAngle(angle))

        setModuleTo(self.frontLeftModule, Rotation2d.fromDegrees(45))
        setModuleTo(self.frontRightModule, Rotation2d.fromDegrees(-45))
        setModuleTo(self.backLeftModule, Rotation2d.fromDegrees(135))
        setModuleTo(self.backRightModule, Rotation2d.fromDegrees(-135))

    def resetDriveAtPosition(self, pose: Pose2d):
        self.resetSwerveModules()
        self.resetGyro(pose)

    def getRobotRelativeSpeeds(self):
        return self.kinematics.toChassisSpeeds(self.getModuleStates())

    def getModuleStates(self):
        return (
            self.frontLeftModule.getState(),
            self.frontRightModule.getState(),
            self.backLeftModule.getState(),
            self.backRightModule.getState(),
        )

    def resetSwerveModules(self):
        for module in self.modules:
            module.reset()
        self.resetGyro(Pose2d())

    def setOdometryPosition(self, pose: Pose2d):
        # self.gyro.setAngleAdjustment(pose.rotation().degrees())
        self.rotationOffset = pose.rotation().degrees()
        self.resetOdometryAtPosition(pose)

    def resetGyro(self, pose: Pose2d):
        self.gyro.set_yaw(0)
        # self.gyro.setAngleAdjustment(pose.rotation().degrees())
        self.rotationOffset = pose.rotation().degrees()
        self.resetOdometryAtPosition(pose)

        # if RobotBase.isSimulation():
        #     self.resetSimPosition(pose)

    def getPose(self) -> Pose2d:
        translation = self.estimator.estimatedPose.translation()
        rotation = self.getRotation()
        return Pose2d(translation, rotation)

    def applyStates(
        self,
        moduleStates: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        (
            frontLeftState,
            frontRightState,
            backLeftState,
            backRightState,
        ) = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates, kMaxWheelLinearVelocity
        )

        self.swerveStateExpectedPublisher.set(
            [frontLeftState, frontRightState, backLeftState, backRightState]
        )
        self.frontLeftModule.applyState(frontLeftState)
        self.frontRightModule.applyState(frontRightState)
        self.backLeftModule.applyState(backLeftState)
        self.backRightModule.applyState(backRightState)

    def getRotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value + self.rotationOffset)

    def getAngularVelocity(self) -> float:
        """radians"""
        if RobotBase.isSimulation():
            value: ChassisSpeeds = self.simVelocityGetter.get()
            return value.omega
        return self.gyro.get_angular_velocity_z_world().value * kRadiansPerDegree

    def getPitch(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.gyro.get_pitch().value + 180)

    def resetOdometryAtPosition(self, pose: Pose2d):
        self.odometry.resetPosition(
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            pose,
        )
        self.estimator.resetPosition(
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            pose,
        )

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's
        odometry with sensor data.
        """

        swervePositions = (
            self.frontLeftModule.getPosition(),
            self.frontRightModule.getPosition(),
            self.backLeftModule.getPosition(),
            self.backRightModule.getPosition(),
        )
        self.odometry.update(self.getRotation(), swervePositions)
        robotPose = self.getPose()

        self.swerveStatePublisher.set(
            [
                self.frontLeftModule.getState(),
                self.frontRightModule.getState(),
                self.backLeftModule.getState(),
                self.backRightModule.getState(),
            ]
        )

        # robotPoseArray = [robotPose.X(), robotPose.Y(), robotPose.rotation().radians()]

        self.robotPosePublisher.set(robotPose)
        self.robotPoseValidPublisher.set(True)

        odoMeasure = OdometryObservation(
            [*swervePositions], self.getRotation(), self.printTimer.getFPGATimestamp()
        )
        self.estimator.addOdometryMeasurement(odoMeasure)
        self.visionEstimate = self.estimator.estimatedPose

    def arcadeDriveWithFactors(
        self,
        forwardSpeedFactor: float,
        sidewaysSpeedFactor: float,
        rotationSpeedFactor: float,
        coordinateMode: CoordinateMode,
    ) -> None:
        """
        Drives the robot using arcade controls.

        :param forwardSpeedFactor: the commanded forward movement
        :param sidewaysSpeedFactor: the commanded sideways movement
        :param rotationSpeedFactor: the commanded rotation
        """

        forwardSpeedFactor = convenientmath.clamp(forwardSpeedFactor, -1, 1)
        sidewaysSpeedFactor = convenientmath.clamp(sidewaysSpeedFactor, -1, 1)
        rotationSpeedFactor = convenientmath.clamp(rotationSpeedFactor, -1, 1)

        combinedLinearFactor = Translation2d(
            forwardSpeedFactor, sidewaysSpeedFactor
        ).norm()

        # prevent combined forward & sideways inputs from exceeding the max linear velocity
        if combinedLinearFactor > 1.0:
            forwardSpeedFactor = forwardSpeedFactor / combinedLinearFactor
            sidewaysSpeedFactor = sidewaysSpeedFactor / combinedLinearFactor

        chassisSpeeds = ChassisSpeeds(
            forwardSpeedFactor * kMaxForwardLinearVelocity,
            sidewaysSpeedFactor * kMaxSidewaysLinearVelocity,
            rotationSpeedFactor * kMaxRotationAngularVelocity,
        )

        self.arcadeDriveWithSpeeds(chassisSpeeds, coordinateMode)

    def drivePathPlanned(self, chassisSpeeds: ChassisSpeeds, _feedForward):
        return self.arcadeDriveWithSpeeds(
            chassisSpeeds, DriveSubsystem.CoordinateMode.RobotRelative
        )

    def arcadeDriveWithSpeeds(
        self, chassisSpeeds: ChassisSpeeds, coordinateMode: CoordinateMode
    ) -> None:
        discritizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, kRobotUpdatePeriod)

        robotChassisSpeeds = None
        if coordinateMode is DriveSubsystem.CoordinateMode.RobotRelative:
            robotChassisSpeeds = discritizedSpeeds
        elif coordinateMode is DriveSubsystem.CoordinateMode.FieldRelative:
            robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                discritizedSpeeds.vx,
                discritizedSpeeds.vy,
                discritizedSpeeds.omega,
                self.getRotation()
                + Rotation2d(self.getAngularVelocity() * kDriveAngularVelocityCoeff),
            )

        fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            robotChassisSpeeds.vx,
            robotChassisSpeeds.vy,
            robotChassisSpeeds.omega,
            -self.getRotation(),
        )

        self.driveVelocityPublisher.set(fieldSpeeds)

        moduleStates = self.kinematics.toSwerveModuleStates(robotChassisSpeeds)
        self.applyStates(moduleStates)
