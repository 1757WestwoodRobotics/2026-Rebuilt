# Trajectory Following
from pathplannerlib.controller import PIDConstants
from pathplannerlib.path import PathConstraints

from .drive import (
    kMaxWheelLinearVelocity,
    kMaxWheelLinearAcceleration,
    kMaxRotationAngularVelocity,
    kMaxRotationAngularAcceleration,
)

kTrajectoryPositionPGainAuto = 9
kTrajectoryPositionPGainVision = 5
kTrajectoryPositionIGain = 0
kTrajectoryPositionDGain = 0

kTrajectoryAnglePGain = 7
kTrajectoryAngleIGain = 0
kTrajectoryAngleDGain = 0

kPathFollowingTranslationConstantsAuto = PIDConstants(
    kTrajectoryPositionPGainAuto, kTrajectoryPositionIGain, kTrajectoryPositionDGain
)
kPathFollowingTranslationConstantsVision = PIDConstants(
    kTrajectoryPositionPGainVision, kTrajectoryPositionIGain, kTrajectoryPositionDGain
)
kPathFollowingRotationConstants = PIDConstants(
    kTrajectoryAnglePGain, kTrajectoryAngleIGain, kTrajectoryAngleDGain
)

kPathfindingConstraints = PathConstraints(
    kMaxWheelLinearVelocity / 4,
    kMaxWheelLinearAcceleration / 4,
    kMaxRotationAngularVelocity / 2,
    kMaxRotationAngularAcceleration / 2,
)


# waypoint
kMaxWaypointTranslationalVelocity = kMaxWheelLinearVelocity
kMaxWaypointTranslationalAcceleration = kMaxWaypointTranslationalVelocity * 3

kWaypointJoystickVariation = 0.1
"""meters"""
kWaypointActiveKey = "waypoint/active"
kWaypointAtTargetKey = "waypoint/atPosition"

kTargetWaypointPoseKey = "waypoint/target"
kTargetWaypointXControllerKey = "waypoint/x"
kTargetWaypointYControllerKey = "waypoint/y"
kTargetWaypointThetaControllerKey = "waypoint/theta"

kRotationPGain = 0.1
kRotationIGain = 0
kRotationDGain = 0.00
