from math import sqrt
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d, Transform2d, Transform3d
from wpimath.interpolation import (
    TimeInterpolatablePose2dBuffer,
    TimeInterpolatableRotation2dBuffer,
)
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition

from constants.turret import kTurretLocation


# shamelessly reimplemented from 6328
class OdometryObservation:
    """
    Represents an odometry measurement for the robot pose estimator.
    """

    def __init__(
        self,
        wheelPositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        gyroAngle: Rotation2d,
        timestamp: float,
    ) -> None:
        self.wheelPositions = wheelPositions
        self.gyroAngle = gyroAngle
        self.timestamp = timestamp


class VisionObservation:
    """
    Represents a vision measurement for the robot pose estimator.
    """

    def __init__(
        self,
        visionPose: Pose2d,
        timestamp: float,
        std: list[float],
        tagsUsed: list[int],
    ) -> None:
        assert len(std) == 3
        self.visionPose = visionPose
        self.timestamp = timestamp
        self.std = std
        self.tagsUsed = tagsUsed


class TurretedVisionObservation:
    """
    Represents a vision measurement from a camera mounted on a turret for the robot pose estimator.
    """

    def __init__(
        self,
        fieldToTurretTransform: Transform3d,
        timestamp: float,
        std: list[float],
        tagsUsed: list[int],
    ) -> None:
        assert len(std) == 3
        self.fieldToTurretTransform = fieldToTurretTransform
        self.timestamp = timestamp
        self.std = std
        self.tagsUsed = tagsUsed


class TurretObservation:
    """
    Represents a turret angle measurement for the robot pose estimator.
    """

    def __init__(self, turretAngle: Rotation2d, timestamp: float) -> None:
        self.turretAngle = turretAngle
        self.timestamp = timestamp


class RobotPoseEstimator:
    """
    Estimates the robot pose using odometry and vision measurements.
    """

    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        kinematics: SwerveDrive4Kinematics,
        gyro: Rotation2d,
        startWheelPositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        startPose: Pose2d,
        odoStdDevs: tuple[float, float, float],
    ) -> None:
        self.kinematics = kinematics
        self.gyroOffset = gyro
        self.lastWheelPositions = startWheelPositions
        self.odometryPose = startPose
        self.estimatedPose = startPose

        self.odoStdDevs = odoStdDevs

        self.poseBuffer = TimeInterpolatablePose2dBuffer(2.0)

        self._lastMeasurementTime = 0.0

    @property
    def lastMeasurementTime(self) -> float:
        return self._lastMeasurementTime

    def addOdometryMeasurement(self, measurement: OdometryObservation):
        """
        Adds an odometry measurement to the pose estimator.
        Args:
            measurement: The odometry measurement to add. As given from a drive base
        Returns:
            None
        """
        lastOdoPose = self.odometryPose

        twist = self.kinematics.toTwist2d(
            self.lastWheelPositions, measurement.wheelPositions
        )
        self.odometryPose = self.odometryPose.exp(twist)
        self.odometryPose = Pose2d(
            self.odometryPose.translation(), measurement.gyroAngle + self.gyroOffset
        )
        self.poseBuffer.addSample(measurement.timestamp, self.odometryPose)
        finalTwist = lastOdoPose.log(self.odometryPose)

        self.estimatedPose = self.estimatedPose.exp(finalTwist)

        self.lastWheelPositions = measurement.wheelPositions

    def addVisionMeasurement(self, measurement: VisionObservation):
        """
        Adds a vision measurement to the pose estimator.
        Args:
            measurement: The vision measurement to add. As given from a vision system
        Returns:
            None
        """
        # check if measurement is valid enough to work with
        if self.poseBuffer.getInternalBuffer()[-1][0] - 2.0 > measurement.timestamp:
            return

        sample = self.poseBuffer.sample(measurement.timestamp)
        if sample is None:
            return

        sampleToOdometryTransform = Transform2d(sample, self.odometryPose)
        odometryToSampleTransform = Transform2d(self.odometryPose, sample)

        estimateAtTime = self.estimatedPose + odometryToSampleTransform

        # new vision matrix
        r = [i * i for i in measurement.std]

        # Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        # and C = I. See wpimath/algorithms.md.
        visionK = [0.0, 0.0, 0.0]

        for i in range(3):
            stdDev = self.odoStdDevs[i]
            if stdDev == 0.0:
                visionK[i] = 0.0
            else:
                visionK[i] = stdDev / (stdDev + sqrt(stdDev * r[i]))

        transform = Transform2d(estimateAtTime, measurement.visionPose)
        kTimesTransform = [
            visionK[i] * k
            for i, k in enumerate(
                [transform.X(), transform.Y(), transform.rotation().radians()]
            )
        ]

        scaledTransform = Transform2d(
            kTimesTransform[0], kTimesTransform[1], kTimesTransform[2]
        )

        self.estimatedPose = (
            estimateAtTime + scaledTransform + sampleToOdometryTransform
        )

        self._lastMeasurementTime = max(
            self._lastMeasurementTime, measurement.timestamp
        )

    def resetPosition(
        self,
        _gyro: Rotation2d,
        startWheelPositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        startPose: Pose2d,
    ):
        """
        Resets the pose estimator to a known pose.
        Args:
            _gyro: The current gyro angle.
            startWheelPositions: The current wheel positions.
            startPose: The pose to reset to.
        Returns:
            None
        """
        self.gyroOffset = startPose.rotation() - (
            self.odometryPose.rotation() - self.gyroOffset
        )
        self._lastMeasurementTime = 0.0  # reset measurement time on reset
        self.estimatedPose = startPose
        self.odometryPose = startPose
        self.lastWheelPositions = startWheelPositions
        self.poseBuffer.clear()


class TurretedRobotPoseEstimator(RobotPoseEstimator):
    """
    Estimates the robot pose using odometry and vision measurements from a turret-mounted camera.
    Supports both turreted and non-turreted vision measurements.
    """

    def __init__(
        self,
        kinematics: SwerveDrive4Kinematics,
        gyro: Rotation2d,
        startWheelPositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        startPose: Pose2d,
        odoStdDevs: tuple[float, float, float],
    ) -> None:
        super().__init__(kinematics, gyro, startWheelPositions, startPose, odoStdDevs)

        self.turretBuffer = TimeInterpolatableRotation2dBuffer(2.0)

    def addTurretMeasurement(self, measurement: TurretObservation):
        """
        Adds a turret angle measurement to the pose estimator.
        Args:
            measurement: The turret angle measurement to add. As given from a turret subsystem
        Returns:
            None
        """
        self.turretBuffer.addSample(measurement.timestamp, measurement.turretAngle)

    def addTurretedVisionMeasurement(self, measurement: TurretedVisionObservation):
        """
        Adds a turreted vision measurement to the pose estimator.
        Args:
            measurement: The turreted vision measurement to add. As given from a vision subsystem
        Returns:
            None
        """
        if self.poseBuffer.getInternalBuffer()[-1][0] - 2.0 > measurement.timestamp:
            return
        if self.turretBuffer.getInternalBuffer()[-1][0] - 2.0 > measurement.timestamp:
            return

        poseSample = self.poseBuffer.sample(measurement.timestamp)
        if poseSample is None:
            return

        turretSample = self.turretBuffer.sample(measurement.timestamp)
        if turretSample is None:
            return

        sampledRotationTrasnfrom = Transform3d(
            0, 0, 0, Rotation3d(0, 0, turretSample.radians())
        )
        measuredFieldToRobot = (
            measurement.fieldToTurretTransform
            + sampledRotationTrasnfrom.inverse()
            + kTurretLocation.inverse()
        )
        measuredFieldToRobot2d = Pose2d(
            measuredFieldToRobot.translation().toTranslation2d(),
            measuredFieldToRobot.rotation().toRotation2d(),
        )

        sampleToOdometryTransform = Transform2d(poseSample, self.odometryPose)
        odometryToSampleTransform = Transform2d(self.odometryPose, poseSample)

        estimateAtTime = self.estimatedPose + odometryToSampleTransform

        r = [i * i for i in measurement.std]

        visionK = [0.0, 0.0, 0.0]
        for i in range(3):
            stdDev = self.odoStdDevs[i]
            if stdDev == 0.0:
                visionK[i] = 0.0
            else:
                visionK[i] = stdDev / (stdDev + sqrt(stdDev * r[i]))
        transform = Transform2d(estimateAtTime, measuredFieldToRobot2d)
        kTimesTransform = [
            visionK[i] * k
            for i, k in enumerate(
                [transform.X(), transform.Y(), transform.rotation().radians()]
            )
        ]
        scaledTransform = Transform2d(
            kTimesTransform[0], kTimesTransform[1], kTimesTransform[2]
        )
        self.estimatedPose = (
            estimateAtTime + scaledTransform + sampleToOdometryTransform
        )

        self._lastMeasurementTime = max(
            self._lastMeasurementTime, measurement.timestamp
        )

    def resetPosition(
        self,
        _gyro: Rotation2d,
        startWheelPositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        startPose: Pose2d,
    ):
        """
        Resets the pose estimator to a known pose.
        Args:
            _gyro: The current gyro angle.
            startWheelPositions: The current wheel positions.
            startPose: The pose to reset to.
        Returns:
            None
        """
        super().resetPosition(_gyro, startWheelPositions, startPose)
        self.turretBuffer.clear()
