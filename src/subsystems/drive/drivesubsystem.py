from enum import Enum, auto

from typing import Tuple
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from pathplannerlib.path import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder

from pykit.logger import Logger
from pykit.autolog import autolog_output, autologgable_output

from wpilib import (
    RobotBase,
    DriverStation,
)

from wpimath.geometry import (
    Rotation2d,
    Translation2d,
)
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModulePosition,
    SwerveModuleState,
    SwerveDrive4Kinematics,
)

from robotstate import RobotState
from subsystems.drive.swervemodule import SwerveModule
from subsystems.drive.driveio import DriveIO
from subsystems.drive.swervemoduleio import SwerveModuleIO


from constants.drive import (
    kFrontLeftModuleName,
    kFrontRightModuleName,
    kBackLeftModuleName,
    kBackRightModuleName,
    kMaxWheelLinearVelocity,
    kMaxForwardLinearVelocity,
    kMaxSidewaysLinearVelocity,
    kMaxRotationAngularVelocity,
    kDriveAngularVelocityCoeff,
    kDriveKinematics,
    kFrontLeftWheelPosition,
    kFrontRightWheelPosition,
    kBackLeftWheelPosition,
    kBackRightWheelPosition,
)

from constants.sim import kSimRobotVelocityArrayKey
from constants.trajectory import (
    kPathFollowingTranslationConstantsAuto,
    kPathFollowingRotationConstants,
)
from constants import kRobotUpdatePeriod
from util.logtracer import LogTracer
from util import convenientmath


# pylint: disable-next=too-many-instance-attributes
@autologgable_output
class DriveSubsystem(Subsystem):
    class CoordinateMode(Enum):
        RobotRelative = auto()
        FieldRelative = auto()

    def __init__(
        self,
        io: DriveIO,
        module_io: Tuple[
            SwerveModuleIO, SwerveModuleIO, SwerveModuleIO, SwerveModuleIO
        ],
    ) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.io = io
        self.inputs = DriveIO.DriveIOInputs()

        front_left_io, front_right_io, back_left_io, back_right_io = module_io
        self.frontLeftModule = SwerveModule(
            kFrontLeftModuleName,
            front_left_io,
        )
        self.frontRightModule = SwerveModule(
            kFrontRightModuleName,
            front_right_io,
        )
        self.backLeftModule = SwerveModule(
            kBackLeftModuleName,
            back_left_io,
        )
        self.backRightModule = SwerveModule(
            kBackRightModuleName,
            back_right_io,
        )

        self.modules = (
            self.frontLeftModule,
            self.frontRightModule,
            self.backLeftModule,
            self.backRightModule,
        )

        self.rawGyroRotation = Rotation2d()
        self.lastModulePositions = self.getModulePositions()
        self.fieldSpeeds = ChassisSpeeds()

        AutoBuilder.configure(
            RobotState.getPose,
            RobotState.resetPose,
            self.getRobotRelativeSpeeds,
            self.drivePathPlanned,
            PPHolonomicDriveController(
                kPathFollowingTranslationConstantsAuto,
                kPathFollowingRotationConstants,
            ),
            # controller
            RobotConfig.fromGUISettings(),
            # robot_config
            (lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed),
            self,
        )

        self.expectedSwerveStates: tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ] = (
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
        )

        if RobotBase.isSimulation():
            self.simVelocityGetter = (
                NetworkTableInstance.getDefault()
                .getStructTopic(kSimRobotVelocityArrayKey, ChassisSpeeds)
                .subscribe(ChassisSpeeds())
            )

    @autolog_output(key="drive/swerve/expected")
    def getSwerveExpectedState(
        self,
    ) -> Tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        return self.expectedSwerveStates

    def getRawRotation(self) -> Rotation2d:
        return self.rawGyroRotation

    def defenseState(self):
        def setModuleTo(module: SwerveModule, angle: Rotation2d):
            module.setWheelLinearVelocityTarget(0)
            module.setSwerveAngleTarget(angle)

        setModuleTo(self.frontLeftModule, kFrontLeftWheelPosition.angle())
        setModuleTo(self.frontRightModule, kFrontRightWheelPosition.angle())
        setModuleTo(self.backLeftModule, kBackLeftWheelPosition.angle())
        setModuleTo(self.backRightModule, kBackRightWheelPosition.angle())

    @autolog_output(key="Robot/relSpeeds")
    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        return kDriveKinematics.toChassisSpeeds(self.getModuleStates())

    @autolog_output(key="Robot/speeds")
    def getFieldRelativeSpeeds(self) -> ChassisSpeeds:
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            self.getRobotRelativeSpeeds(), RobotState.getRotation()
        )

    @autolog_output(key="drive/swerve/real")
    def getModuleStates(
        self,
    ) -> Tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        return (
            self.frontLeftModule.getState(),
            self.frontRightModule.getState(),
            self.backLeftModule.getState(),
            self.backRightModule.getState(),
        )

    def resetSwerveModules(self):
        for module in self.modules:
            module.reset()

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

        self.expectedSwerveStates = (
            frontLeftState,
            frontRightState,
            backLeftState,
            backRightState,
        )
        self.frontLeftModule.applyState(frontLeftState)
        self.frontRightModule.applyState(frontRightState)
        self.backLeftModule.applyState(backLeftState)
        self.backRightModule.applyState(backRightState)

    @autolog_output(key="Robot/velocity")
    def getAngularVelocity(self) -> float:
        """radians"""
        if RobotBase.isSimulation() and not Logger.isReplay():
            value: ChassisSpeeds = self.simVelocityGetter.get()
            return value.omega
        return self.inputs.gyro_yaw_rate_rad_per_sec

    def getModulePositions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return (
            self.frontLeftModule.getPosition(),
            self.frontRightModule.getPosition(),
            self.backLeftModule.getPosition(),
            self.backRightModule.getPosition(),
        )

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's
        odometry with sensor data.
        """

        LogTracer.resetOuter("DriveSubsystemPeriodic")
        self.io.updateInputs(self.inputs)
        LogTracer.record("IOUpdate")
        if self.inputs.connected:
            self.rawGyroRotation = Rotation2d(self.inputs.gyro_yaw_rad)
        else:
            self.rawGyroRotation += Rotation2d(
                kDriveKinematics.toTwist2d(
                    self.lastModulePositions, self.getModulePositions()
                ).dtheta
            )  # base the gyro off of the swerves
        self.lastModulePositions = self.getModulePositions()
        LogTracer.record("StateUpdate")
        Logger.processInputs("Drive", self.inputs)
        LogTracer.record("LoggerProcessInputs")

        for module in self.modules:
            module.periodic()

        LogTracer.record("ModulesPeriodic")
        LogTracer.recordTotal()

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

    @autolog_output(key="drive/fieldSpeeds")
    def chassisSpeeds(self) -> ChassisSpeeds:
        return self.fieldSpeeds

    def arcadeDriveWithSpeeds(
        self, chassisSpeeds: ChassisSpeeds, coordinateMode: CoordinateMode
    ) -> None:
        Logger.recordOutput("drive/swerve/commandedSpeeds", chassisSpeeds)
        discritizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, kRobotUpdatePeriod)

        robotChassisSpeeds = None
        if coordinateMode is DriveSubsystem.CoordinateMode.RobotRelative:
            robotChassisSpeeds = discritizedSpeeds
        elif coordinateMode is DriveSubsystem.CoordinateMode.FieldRelative:
            robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                discritizedSpeeds.vx,
                discritizedSpeeds.vy,
                discritizedSpeeds.omega,
                RobotState.getRotation()
                + Rotation2d(self.getAngularVelocity() * kDriveAngularVelocityCoeff),
            )

        self.fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            robotChassisSpeeds.vx,
            robotChassisSpeeds.vy,
            robotChassisSpeeds.omega,
            -RobotState.getRotation(),
        )

        moduleStates = kDriveKinematics.toSwerveModuleStates(robotChassisSpeeds)
        self.applyStates(moduleStates)
