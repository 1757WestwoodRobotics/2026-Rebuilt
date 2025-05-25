# Limelight
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Transform3d

from util.keyorganization import OptionalValueKeys

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

kSimulationVariation = 0.001  # meters, as a standard deviation


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
