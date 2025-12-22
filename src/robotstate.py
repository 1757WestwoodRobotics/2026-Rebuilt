from typing import Callable
from pykit.logger import Logger
from wpilib import DataLogManager, RobotBase
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition

from subsystems.drive.robotposeestimator import (
    OdometryObservation,
    RobotPoseEstimator,
    VisionObservation,
)

from constants.drive import kDriveKinematics
from constants.auto import kAutoDistanceTolerance, kAutoRotationTolerance
from util.logtracer import LogTracer


class RobotState:
    headingOffset: Rotation2d = Rotation2d()
    robotHeading: Rotation2d = Rotation2d()

    modulePositions: tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ] = (
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition(),
        SwerveModulePosition(),
    )

    estimator: RobotPoseEstimator = RobotPoseEstimator(
        kDriveKinematics, Rotation2d(), modulePositions, Pose2d(), (0.1, 0.1, 0.1)
    )
    odometry: SwerveDrive4Odometry = SwerveDrive4Odometry(
        kDriveKinematics, Rotation2d(), modulePositions, Pose2d()
    )

    simResetPoseConsumers: list[Callable[[Pose2d], None]] = []
    simPoseRecieverConsumers: list[Callable[[], Pose2d]] = []

    robotFieldVelocity: ChassisSpeeds = ChassisSpeeds()

    targetAutonomousStartingLocation: Pose2d = Pose2d()

    @classmethod
    def setAutonomousStartingLogation(cls, location: Pose2d):
        cls.targetAutonomousStartingLocation = location

    @classmethod
    def addVisionMeasurement(cls, measurement: VisionObservation):
        cls.estimator.addVisionMeasurement(measurement)

    @classmethod
    def periodic(
        cls,
        heading: Rotation2d,
        headingTimestamp: float,
        robotYawVelocity: float,
        fieldRelativeRobotVelocity: ChassisSpeeds,
        modulePositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
    ) -> None:
        LogTracer.resetOuter("RobotState")
        cls.robotHeading = heading
        cls.modulePositions = modulePositions
        cls.odometry.update(heading, modulePositions)

        cls.robotFieldVelocity = fieldRelativeRobotVelocity
        LogTracer.record("OdometryUpdate")
        cls.estimator.addOdometryMeasurement(
            OdometryObservation(modulePositions, heading, headingTimestamp)
        )
        LogTracer.record("EstimatorUpdate")

        estimatedFieldPose = cls.getPose()
        Logger.recordOutput("Robot/Pose/EstimatorPose", estimatedFieldPose)
        Logger.recordOutput("Robot/Pose/OdometryPose", cls.odometry.getPose())
        Logger.recordOutput("Robot/Heading", cls.robotHeading)
        Logger.recordOutput("Robot/HeadingVelocity", robotYawVelocity)
        Logger.recordOutput("Robot/Velocity", fieldRelativeRobotVelocity)
        Logger.recordOutput("Robot/HeadingOffset", cls.headingOffset)

        autoPositionDelta = estimatedFieldPose - cls.targetAutonomousStartingLocation
        Logger.recordOutput("Auto/PositionOffset", autoPositionDelta)
        Logger.recordOutput(
            "Auto/PositionCorrect",
            autoPositionDelta.translation().norm() < kAutoDistanceTolerance,
        )
        Logger.recordOutput(
            "Auto/RotationCorrect",
            abs(autoPositionDelta.rotation().radians()) < kAutoRotationTolerance,
        )
        Logger.recordOutput("Auto/StartingPose", cls.targetAutonomousStartingLocation)

        if not RobotBase.isReal():
            Logger.recordOutput("Robot/SimPose", cls.getSimPose())

        LogTracer.recordTotal()

    @classmethod
    def getPose(cls) -> Pose2d:
        return cls.estimator.estimatedPose

    @classmethod
    def getRotation(cls) -> Rotation2d:
        return cls.getPose().rotation()

    @classmethod
    def resetPose(cls, pose: Pose2d = Pose2d()) -> None:
        cls.headingOffset = cls.robotHeading - pose.rotation()
        cls.odometry.resetPosition(cls.robotHeading, cls.modulePositions, pose)
        cls.estimator.resetPosition(cls.robotHeading, cls.modulePositions, pose)

        if RobotBase.isSimulation() and not Logger.isReplay():
            cls.resetSimPose(pose)

    @classmethod
    def resetSimPose(cls, pose: Pose2d):
        if len(cls.simResetPoseConsumers) > 0:
            for consumer in cls.simResetPoseConsumers:
                consumer(pose)
            return
        DataLogManager.log("This is not supposed to happen")

    @classmethod
    def registerSimPoseResetConsumer(cls, consumer: Callable[[Pose2d], None]) -> None:
        cls.simResetPoseConsumers.append(consumer)

    @classmethod
    def getSimPose(cls) -> Pose2d:
        if len(cls.simPoseRecieverConsumers) == 1:
            return cls.simPoseRecieverConsumers[0]()
        DataLogManager.log("This is not supposed to happen")
        return cls.getPose()

    @classmethod
    def registerSimPoseRecieverConsumer(cls, consumer: Callable[[], Pose2d]) -> None:
        cls.simPoseRecieverConsumers.append(consumer)
