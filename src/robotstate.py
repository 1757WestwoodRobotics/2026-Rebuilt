from typing import Callable
from commands2.button import Trigger
from pykit.logger import Logger
from wpilib import RobotBase, DriverStation
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Translation2d,
)
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition

from util.convenientmath import pose3dFrom2d
from util.fliputil import FlipUtil
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
from constants.field import kEndgameDuration, kShiftDuration, kCloseHubLocation
from constants.vision import kRedHubAprilTags, kBlueHubAprilTags


# pylint: disable-next=too-many-public-methods
class RobotState:
    headingOffset: Rotation2d = Rotation2d()
    robotHeading: Rotation2d = Rotation2d()
    turretRotation: Rotation2d = Rotation2d()
    intakeRotation: Rotation2d = Rotation2d()

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

    fieldEstimator: TurretedRobotPoseEstimator = TurretedRobotPoseEstimator(
        kDriveKinematics, Rotation2d(), modulePositions, Pose2d(), (0.1, 0.1, 0.1)
    )
    hubEstimator: TurretedRobotPoseEstimator = TurretedRobotPoseEstimator(
        kDriveKinematics, Rotation2d(), modulePositions, Pose2d(), (0.1, 0.1, 0.1)
    )
    """hub estimator is a pose estimator that cares about being relative to the hub.
    Its poses will be returned as full field, but only use apriltags that are on the hub."""
    odometry: SwerveDrive4Odometry = SwerveDrive4Odometry(
        kDriveKinematics, Rotation2d(), modulePositions, Pose2d()
    )

    simResetPoseConsumers: list[Callable[[Pose2d], None]] = []
    simPoseReceiverConsumers: list[Callable[[], Pose2d]] = []

    robotFieldVelocity: ChassisSpeeds = ChassisSpeeds()

    targetAutonomousStartingLocation: Pose2d = Pose2d()

    @classmethod
    def hubTags(cls) -> list[int]:
        """
        Returns the april tag IDs of the hubs we are scoring on
        This is determined by the alliance color
        Defaults to blue hub tags if alliance is unknown
        """
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return kRedHubAprilTags
        else:
            return kBlueHubAprilTags

    @classmethod
    def setAutonomousStartingLocation(cls, location: Pose2d):
        cls.targetAutonomousStartingLocation = location

    @classmethod
    def addVisionMeasurement(cls, measurement: VisionObservation):
        cls.fieldEstimator.addVisionMeasurement(measurement)
        if len(measurement.tagsUsed) == 0:
            return
        if set(measurement.tagsUsed).issubset(set(cls.hubTags())):
            cls.hubEstimator.addVisionMeasurement(measurement)

    @classmethod
    def addTurretedVisionMeasurement(cls, measurement: TurretedVisionObservation):
        cls.fieldEstimator.addTurretedVisionMeasurement(measurement)
        if len(measurement.tagsUsed) == 0:
            return
        if set(measurement.tagsUsed).issubset(set(cls.hubTags())):
            cls.hubEstimator.addTurretedVisionMeasurement(measurement)

    @classmethod
    def getTurretPose(cls) -> Pose3d:
        return (
            pose3dFrom2d(cls.getFieldPose())
            + kTurretLocation
            + Transform3d(0, 0, 0, Rotation3d(0, 0, cls.turretRotation.radians()))
        )

    @classmethod
    def getAutoWinner(cls) -> DriverStation.Alliance | None:
        """
        Returns the alliance that won autonomous, or None if unknown
        This is determined by the game specific message sent by the field
        https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        """
        match DriverStation.getGameSpecificMessage():
            case "R":
                return DriverStation.Alliance.kRed
            case "B":
                return DriverStation.Alliance.kBlue
            case None | "":
                return None
            case _:
                print(
                    "Invalid game specific message:",
                    DriverStation.getGameSpecificMessage(),
                )
                return None

    @classmethod
    def didWinAuto(cls) -> bool:
        """
        Returns true if our alliance won autonomous, false otherwise
        If unknown, returns false
        If the current alliance is unknown, returns false
        """
        autoWinner = cls.getAutoWinner()
        if autoWinner is None:  # we are likely in auto currently
            return False
        return autoWinner == DriverStation.getAlliance()

    @classmethod
    def hubActive(cls) -> bool:
        """
        Returns true if the active hub is the one we are scoring on
        The active hub is determined by the match time and whether we won autonomous
        0-20 seconds: Autonomous, both hubs active
        21-110 seconds: Shift periods, only one hub active
            Shift 1 (86-110s): Hub determined by autonomous winner
            Shift 2 (61-85s):  Hub opposite of autonomous winner
            Shift 3 (36-60s):  Hub determined by autonomous winner
            Shift 4 (21-35s):  Hub opposite of autonomous winner
        111-140 seconds: Endgame, both hubs active
        """
        if DriverStation.isAutonomous():
            return True
        else:
            time = (
                DriverStation.getMatchTime()
            )  # this is the time remaining in the match, not elapsed time
            if time <= 0:
                return True  # safety net for negative time, assume its active (testing)
            if time <= kEndgameDuration:  # endgame, both hubs active
                return True
            elif (
                time >= kEndgameDuration + kShiftDuration * 4
            ):  # transition period, before shift 4, all hubs active
                return True
            didWinAuto = cls.didWinAuto()
            if (
                time <= kEndgameDuration + kShiftDuration
            ):  # shift 4, 1 shift and endgame remain
                return didWinAuto
            elif (
                time <= kEndgameDuration + kShiftDuration * 2
            ):  # shift 3, 2 shifts and endgame remain
                return not didWinAuto
            elif (
                time <= kEndgameDuration + kShiftDuration * 3
            ):  # shift 2, 3 shifts and endgame remain
                return didWinAuto
            else:  # 105 < time < 210, shift 1, 4 shifts and endgame remain
                return not didWinAuto

    @classmethod
    def shiftTrigger(cls) -> Trigger:
        """
        Returns a trigger that is active when the hub we are scoring on is active
        This is used for command based programming to enable/disable commands based on hub activity
        See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
        """
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
        intakeRotation: Rotation2d,
    ) -> None:
        LogTracer.resetOuter("RobotState")
        cls.turretRotation = turretRotation
        cls.intakeRotation = intakeRotation
        cls.robotHeading = heading
        cls.modulePositions = modulePositions
        cls.odometry.update(heading, modulePositions)

        cls.robotFieldVelocity = fieldRelativeRobotVelocity
        LogTracer.record("OdometryUpdate")
        cls.fieldEstimator.addOdometryMeasurement(
            OdometryObservation(modulePositions, heading, headingTimestamp)
        )
        cls.fieldEstimator.addTurretMeasurement(
            TurretObservation(turretRotation, headingTimestamp)
        )
        cls.hubEstimator.addOdometryMeasurement(
            OdometryObservation(modulePositions, heading, headingTimestamp)
        )
        cls.hubEstimator.addTurretMeasurement(
            TurretObservation(turretRotation, headingTimestamp)
        )
        LogTracer.record("EstimatorUpdate")

        estimatedFieldPose = cls.getFieldPose()
        Logger.recordOutput("Robot/Pose/Estimator/Pose", estimatedFieldPose)
        Logger.recordOutput(
            "Robot/Pose/Estimator/LastVisionUpdate",
            cls.fieldEstimator.lastMeasurementTime,
        )
        Logger.recordOutput("Robot/Pose/OdometryPose", cls.odometry.getPose())
        Logger.recordOutput("Robot/Pose/HubEstimator/Pose", cls.getHubPose())
        Logger.recordOutput(
            "Robot/Pose/HubEstimator/LastVisionUpdate",
            cls.hubEstimator.lastMeasurementTime,
        )
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
    def getFieldPose(cls) -> Pose2d:
        return cls.fieldEstimator.estimatedPose

    @classmethod
    def getHubPose(cls) -> Pose2d:
        return cls.hubEstimator.estimatedPose

    @classmethod
    def hubLocation(cls) -> Translation2d:
        """
        Returns the global field-space location of the hub as a Translation2d from the origin, based on the alliance color
        """
        return FlipUtil.fieldTranslation(kCloseHubLocation)

    @classmethod
    def distanceToHub(cls) -> float:
        """
        Returns the distance from the robot to the hub in meters, based on the hub estimator
        """
        return cls.hubLocation().distance(cls.getHubPose().translation())

    @classmethod
    def getRotation(cls) -> Rotation2d:
        return cls.getFieldPose().rotation()

    @classmethod
    def resetPose(cls, pose: Pose2d = Pose2d()) -> None:
        cls.headingOffset = cls.robotHeading - pose.rotation()
        cls.odometry.resetPosition(cls.robotHeading, cls.modulePositions, pose)
        cls.fieldEstimator.resetPosition(cls.robotHeading, cls.modulePositions, pose)
        cls.hubEstimator.resetPosition(cls.robotHeading, cls.modulePositions, pose)

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
        if len(cls.simPoseReceiverConsumers) == 1:
            return cls.simPoseReceiverConsumers[0]()
        print("This is not supposed to happen")
        return cls.getFieldPose()

    @classmethod
    def getSimTurretPose(cls) -> Pose3d:
        return (
            pose3dFrom2d(cls.getSimPose())
            + kTurretLocation
            + Transform3d(0, 0, 0, Rotation3d(0, 0, cls.turretRotation.radians()))
        )

    @classmethod
    def registerSimPoseReceiverConsumer(cls, consumer: Callable[[], Pose2d]) -> None:
        cls.simPoseReceiverConsumers.append(consumer)
