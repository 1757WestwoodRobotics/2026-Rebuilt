from typing import Callable
from commands2.button import Trigger
from pykit.logger import Logger
from wpilib import RobotBase, DriverStation
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Transform3d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition

from util.convenientmath import pose3dFrom2d
from util.robotposeestimator import (
    OdometryObservation,
    TurretObservation,
    TurretedRobotPoseEstimator,
    TurretedVisionObservation,
    VisionObservation,
)
from util.logtracer import LogTracer

from constants.drive import kDriveKinematics
from constants.turret import kTurretLocation
from constants.auto import kAutoDistanceTolerance, kAutoRotationTolerance


class RobotState:
    headingOffset: Rotation2d = Rotation2d()
    robotHeading: Rotation2d = Rotation2d()
    turretRotation: Rotation2d = Rotation2d()

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

    estimator: TurretedRobotPoseEstimator = TurretedRobotPoseEstimator(
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
    def addTurretedVisionMeasurement(cls, measurement: TurretedVisionObservation):
        cls.estimator.addTurretedVisionMeasurement(measurement)

    @classmethod
    def getTurretPose(cls) -> Pose3d:
        return (
            pose3dFrom2d(cls.getPose())
            + kTurretLocation
            + Transform3d(0, 0, 0, Rotation3d(0, 0, cls.turretRotation.radians()))
        )

    @classmethod
    def getAutoWinner(cls) -> DriverStation.Alliance | None:
        match DriverStation.getGameSpecificMessage():
            case "R":
                return DriverStation.Alliance.kRed
            case "B":
                return DriverStation.Alliance.kBlue
            case None | "":
                return None
            case _:
                raise ValueError("Invalid game specific message")

    @classmethod
    def didWinAuto(cls) -> bool:
        autoWinner = cls.getAutoWinner()
        if autoWinner is None:  # we are likely in auto currently
            return False
        return autoWinner == DriverStation.getAlliance()

    @classmethod
    def hubActive(cls) -> bool:
        isAuto = DriverStation.isAutonomous()
        if isAuto:
            return True
        else:
            time = DriverStation.getMatchTime()
            if time <= 30:  # endgame, both hubs active
                return True
            elif time >= 210:  # first 10 seconds, both hubs active (transition shift)
                return True
            didWinAuto = cls.didWinAuto()
            if time <= 55:  # shift 4
                return didWinAuto
            elif time <= 80:  # shift 3
                return not didWinAuto
            elif time <= 105:  # shift 2
                return didWinAuto
            else:  # 105 < time < 210, shift 1
                return not didWinAuto

    @classmethod
    def shiftTrigger(cls) -> Trigger:
        return Trigger(cls.hubActive)

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
        turretRotation: Rotation2d,
    ) -> None:
        LogTracer.resetOuter("RobotState")
        cls.turretRotation = turretRotation
        cls.robotHeading = heading
        cls.modulePositions = modulePositions
        cls.odometry.update(heading, modulePositions)

        cls.robotFieldVelocity = fieldRelativeRobotVelocity
        LogTracer.record("OdometryUpdate")
        cls.estimator.addOdometryMeasurement(
            OdometryObservation(modulePositions, heading, headingTimestamp)
        )
        cls.estimator.addTurretMeasurement(
            TurretObservation(turretRotation, headingTimestamp)
        )
        LogTracer.record("EstimatorUpdate")

        estimatedFieldPose = cls.getPose()
        Logger.recordOutput("Robot/Pose/EstimatorPose", estimatedFieldPose)
        Logger.recordOutput("Robot/Pose/OdometryPose", cls.odometry.getPose())
        Logger.recordOutput("Robot/TurretRotation", cls.turretRotation)
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
        Logger.recordOutput("Game/WonAuto", cls.didWinAuto())
        Logger.recordOutput("Game/HubActive", cls.hubActive())

        if not RobotBase.isReal():
            Logger.recordOutput("Robot/SimPose", cls.getSimPose())
            Logger.recordOutput("Robot/SimTurretPose", cls.getSimTurretPose())

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
        print("This is not supposed to happen")

    @classmethod
    def registerSimPoseResetConsumer(cls, consumer: Callable[[Pose2d], None]) -> None:
        cls.simResetPoseConsumers.append(consumer)

    @classmethod
    def getSimPose(cls) -> Pose2d:
        if len(cls.simPoseRecieverConsumers) == 1:
            return cls.simPoseRecieverConsumers[0]()
        print("This is not supposed to happen")
        return cls.getPose()

    @classmethod
    def getSimTurretPose(cls) -> Pose3d:
        return (
            pose3dFrom2d(cls.getSimPose())
            + kTurretLocation
            + Transform3d(0, 0, 0, Rotation3d(0, 0, cls.turretRotation.radians()))
        )

    @classmethod
    def registerSimPoseRecieverConsumer(cls, consumer: Callable[[], Pose2d]) -> None:
        cls.simPoseRecieverConsumers.append(consumer)
