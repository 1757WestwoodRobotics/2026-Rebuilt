from os import path
import wpilib
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Transform3d
from robotpy_apriltag import AprilTagFieldLayout

from .math import kRadiansPerDegree, kMetersPerInch

kLimelightTargetInvalidValue = 0.0
kLimelightTargetValidValue = 1.0
kLimelightMinHorizontalFoV = Rotation2d.fromDegrees(-29.8)
kLimelightMaxHorizontalFoV = Rotation2d.fromDegrees(29.8)
kLimelightMinVerticalFoV = Rotation2d.fromDegrees(-22.85)
kLimelightMaxVerticalFoV = Rotation2d.fromDegrees(22.85)
kLimelightNetworkTableName = "limelight"
kLimelightTargetValidKey = "tv"
kLimelightTargetHorizontalAngleKey = "tx"
kLimelightTargetVerticalAngleKey = "ty"
kLimelightLEDModeKey = "ledMode"
kLimelightTrackerModuleName = "limelight"
kLimelightRelativeToRobotTransform = Transform3d(
    Pose3d(),
    Pose3d(0.236, 0.206, 0.197, Rotation3d()),
)

# Photonvision related
kPhotonvisionCameraName = "camcam"
kPhotonvisionCameraArray = ["frontLeft", "frontRight", "backLeft", "backRight"]

kCameraFOVHorizontal = 75.9  # degrees
kCameraFOVVertical = 47.4  # degrees

kSimulationVariation = 0.0001  # meters, as a standard deviation


kCameraLocationPublisherKey = "camera/location"
kRobotToCameraTransformLL2 = Transform3d(
    Pose3d(),
    Pose3d(
        -11.602 * kMetersPerInch,
        -10.365 * kMetersPerInch,
        8.131 * kMetersPerInch,
        Rotation3d.fromDegrees(180, -14.755, 0),
    ),
)

kRobotToCamera1TransformLL3 = Transform3d(
    Pose3d(),
    Pose3d(
        -11.498 * kMetersPerInch,
        -10.365 * kMetersPerInch,
        8.192 * kMetersPerInch,
        Rotation3d.fromDegrees(180, -14.755, 0),
    ),
)

kRobotToCamera2TransformLL3 = Transform3d(
    Pose3d(),
    Pose3d(
        12.529978 * kMetersPerInch,
        8.455907 * kMetersPerInch,
        8.172675 * kMetersPerInch,
        Rotation3d.fromDegrees(180, -14.755, 0),
    ),
)

kRobotToCamera1Transform = (
    kRobotToCamera1TransformLL3  # NOTE: if/when we swap cameras this needs to change
)

kRobotToCamera2Transform = (
    kRobotToCamera2TransformLL3  # NOTE: if/when we swap cameras this needs to change
)

kTurretToCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(
        0.1,
        0.0,
        0.1,
        Rotation3d.fromDegrees(15, 0.0, 0.0),
    ),
)

kMaxVisionAmbiguity = 0.3
kMaxVisionZError = 0.75  # meters
kXyStdDevCoeff = 0.02  # meters
kThetaStdDevCoeff = 0.06  # radians

kTargetName = "Target"

kApriltagFieldLayout = AprilTagFieldLayout(
    path.join(wpilib.getDeployDirectory(), "apriltags", "2026-rebuilt-andymark.json")
)
kApriltagPositionDict = {
    1: Pose3d(
        (kMetersPerInch * 657.37),
        (kMetersPerInch * 25.80),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 126 * kRadiansPerDegree),
    ),
    2: Pose3d(
        (kMetersPerInch * 657.37),
        (kMetersPerInch * 291.20),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 234 * kRadiansPerDegree),
    ),
    3: Pose3d(
        (kMetersPerInch * 455.15),
        (kMetersPerInch * 317.15),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    4: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 241.64),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    5: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 75.39),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    6: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
    7: Pose3d(
        (kMetersPerInch * 546.87),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    8: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    9: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    10: Pose3d(
        (kMetersPerInch * 481.39),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    11: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    12: Pose3d(
        (kMetersPerInch * 33.51),
        (kMetersPerInch * 25.80),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 54 * kRadiansPerDegree),
    ),
    13: Pose3d(
        (kMetersPerInch * 33.51),
        (kMetersPerInch * 291.20),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 306 * kRadiansPerDegree),
    ),
    14: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 241.64),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    15: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 75.39),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    16: Pose3d(
        (kMetersPerInch * 235.73),
        (kMetersPerInch * -0.15),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 90 * kRadiansPerDegree),
    ),
    17: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    18: Pose3d(
        (kMetersPerInch * 144.00),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    19: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    20: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 186.83),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    21: Pose3d(
        (kMetersPerInch * 209.49),
        (kMetersPerInch * 158.50),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    22: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 130.17),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
}

kApriltagPositionDictAndyMark = {
    1: Pose3d(
        (kMetersPerInch * 656.98),
        (kMetersPerInch * 24.73),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 126 * kRadiansPerDegree),
    ),
    2: Pose3d(
        (kMetersPerInch * 656.98),
        (kMetersPerInch * 291.90),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 234 * kRadiansPerDegree),
    ),
    3: Pose3d(
        (kMetersPerInch * 452.40),
        (kMetersPerInch * 316.21),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    4: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 241.44),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    5: Pose3d(
        (kMetersPerInch * 365.20),
        (kMetersPerInch * 75.19),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 0.0),
    ),
    6: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 129.97),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
    7: Pose3d(
        (kMetersPerInch * 546.87),
        (kMetersPerInch * 158.30),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    8: Pose3d(
        (kMetersPerInch * 530.49),
        (kMetersPerInch * 186.63),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    9: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 186.63),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    10: Pose3d(
        (kMetersPerInch * 481.39),
        (kMetersPerInch * 158.30),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    11: Pose3d(
        (kMetersPerInch * 497.77),
        (kMetersPerInch * 129.97),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    12: Pose3d(
        (kMetersPerInch * 33.91),
        (kMetersPerInch * 24.73),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 54 * kRadiansPerDegree),
    ),
    13: Pose3d(
        (kMetersPerInch * 33.91),
        (kMetersPerInch * 291.90),
        (kMetersPerInch * 58.50),
        Rotation3d(0.0, 0.0, 306 * kRadiansPerDegree),
    ),
    14: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 241.44),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    15: Pose3d(
        (kMetersPerInch * 325.68),
        (kMetersPerInch * 75.19),
        (kMetersPerInch * 73.54),
        Rotation3d(0.0, 30 * kRadiansPerDegree, 180 * kRadiansPerDegree),
    ),
    16: Pose3d(
        (kMetersPerInch * 238.49),
        (kMetersPerInch * 0.42),
        (kMetersPerInch * 51.25),
        Rotation3d(0.0, 0.0, 90 * kRadiansPerDegree),
    ),
    17: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 129.97),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 240 * kRadiansPerDegree),
    ),
    18: Pose3d(
        (kMetersPerInch * 144.00),
        (kMetersPerInch * 158.30),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 180 * kRadiansPerDegree),
    ),
    19: Pose3d(
        (kMetersPerInch * 160.39),
        (kMetersPerInch * 186.63),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    20: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 186.63),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 60 * kRadiansPerDegree),
    ),
    21: Pose3d(
        (kMetersPerInch * 209.49),
        (kMetersPerInch * 158.30),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 0.0),
    ),
    22: Pose3d(
        (kMetersPerInch * 193.10),
        (kMetersPerInch * 129.97),
        (kMetersPerInch * 12.13),
        Rotation3d(0.0, 0.0, 300 * kRadiansPerDegree),
    ),
}
