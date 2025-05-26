from math import sqrt
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Twist2d
from wpimath.interpolation import TimeInterpolatablePose2dBuffer
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition


# shamelessly reimplemented from 6328
class OdometryObservation:
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
    def __init__(self, visionPose: Pose2d, timestamp: float, std: list[float]) -> None:
        assert len(std) == 3
        self.visionPose = visionPose
        self.timestamp = timestamp
        self.std = std


class RobotPoseEstimator:
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
        self.lastGyroAngle = gyro
        self.lastWheelPositions = startWheelPositions
        self.odometryPose = startPose
        self.estimatedPose = startPose

        self.odoStdDevs = odoStdDevs

        self.poseBuffer = TimeInterpolatablePose2dBuffer(2.0)

    def addOdometryMeasurement(self, measurement: OdometryObservation):
        newPositions: list[SwerveModulePosition] = []
        for old, new in zip(self.lastWheelPositions, measurement.wheelPositions):
            newPositions.append(
                SwerveModulePosition(new.distance - old.distance, new.angle)
            )

        twist = self.kinematics.toTwist2d(newPositions)
        twist = Twist2d(
            twist.dx, twist.dy, (measurement.gyroAngle - self.lastGyroAngle).radians()
        )

        self.odometryPose = self.odometryPose.exp(twist)
        self.poseBuffer.addSample(measurement.timestamp, self.odometryPose)

        self.estimatedPose = self.estimatedPose.exp(twist)

        self.lastGyroAngle = measurement.gyroAngle
        self.lastWheelPositions = measurement.wheelPositions

    def addVisionMeasurement(self, measurement: VisionObservation):
        # check if measurement is valid enough to work with
        if self.poseBuffer.getInternalBuffer()[-1][0] - 2.0 > measurement.timestamp:
            return

        sample = self.poseBuffer.sample(measurement.timestamp)
        if sample is None:
            return

        sampleToOdometryTransform = Transform2d(self.odometryPose, sample)
        odometryToSampleTransform = Transform2d(sample, self.odometryPose)

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

    def resetPosition(
        self,
        gyro: Rotation2d,
        startWheelPositions: tuple[
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
            SwerveModulePosition,
        ],
        startPose: Pose2d,
    ):
        self.lastGyroAngle = gyro
        self.estimatedPose = startPose
        self.odometryPose = startPose
        self.lastWheelPositions = startWheelPositions
