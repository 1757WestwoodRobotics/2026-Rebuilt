import math

from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.geometry import Translation2d

from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor

from .math import (
    kMetersPerInch,
    kRadiansPerRevolution,
    kMillisecondsPerSecond,
)

# Robot Physical parameters
kRobotWidth = 28 * kMetersPerInch
"""meters"""

kRobotLength = 26 * kMetersPerInch
"""meters"""

kSwerveModuleCenterToRobotCenterWidth = 11.5 * kMetersPerInch
"""meters"""
kSwerveModuleCenterToRobotCenterLength = 11.5 * kMetersPerInch
"""meters"""

kSwerveModuleDistanceFromRobotCenter = pow(
    pow(kSwerveModuleCenterToRobotCenterWidth, 2)
    + pow(kSwerveModuleCenterToRobotCenterLength, 2),
    0.5,
)
"""meters (c = (a^2 + b^2) ^ 0.5)"""

kFrontLeftWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kFrontRightWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackLeftWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackRightWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kWheelDiameter = 4 * kMetersPerInch
"""meters"""

kWheelRadius = kWheelDiameter / 2
"""meters"""

kWheelCircumference = kWheelRadius * 2 * math.pi
"""meters"""

kWheelDistancePerRevolution = kWheelCircumference
"""meters / revolution"""

kWheelDistancePerRadian = kWheelDistancePerRevolution / kRadiansPerRevolution
"""meters / radian"""

kDriveGearingRatio = (54 / 14) * (25 / 32) * (30 / 15)
"""dimensionless"""

kSteerGearingRatioMk5i = 26 / 1
"""dimensionless"""

kSteerGearingRatioMk5n = 287 / 11
"""dimensionless"""

kMaxMotorAngularVelocity = DCMotor.krakenX60().freeSpeed
"""radians / second"""

kMaxWheelAngularVelocity = kMaxMotorAngularVelocity / kDriveGearingRatio
"""radians / second"""

kMaxWheelLinearVelocity = kWheelDistancePerRadian * kMaxWheelAngularVelocity
"""meters / second"""

kMinWheelLinearVelocity = 0.002
"""meters / second"""

kMaxSteerAngularVelocityMk5i = kMaxMotorAngularVelocity / kSteerGearingRatioMk5i
"""radians / second"""

kMaxSteerAngularVelocityMk5n = kMaxMotorAngularVelocity / kSteerGearingRatioMk5n
"""radians / second"""

kMaxForwardLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxSidewaysLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxRotationAngularVelocity = (
    kMaxWheelLinearVelocity / kSwerveModuleDistanceFromRobotCenter
)
"""radians / second (omega = v / r)"""

kMaxWheelLinearAcceleration = kMaxWheelLinearVelocity / 1
"""meters / second^2"""

kMaxForwardLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxForwardLinearAccelerationWaypoint = kMaxForwardLinearAcceleration * 1.2
"""meters / second^2"""

kMaxSidewaysLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxRotationAngularAcceleration = kMaxRotationAngularVelocity / 0.5
"""radians / second^2"""

kDriveAccelLimit = 7
"""for slew rate limiter"""

kFrontLeftModuleName = "front_left"
kFrontRightModuleName = "front_right"
kBackLeftModuleName = "back_left"
kBackRightModuleName = "back_right"


# CANivore
kCANivoreName = "Drive"

# Motors
kFrontLeftDriveMotorId = 1
kFrontLeftSteerMotorId = 2
kFrontRightDriveMotorId = 3
kFrontRightSteerMotorId = 4
kBackLeftDriveMotorId = 5
kBackLeftSteerMotorId = 6
kBackRightDriveMotorId = 7
kBackRightSteerMotorId = 8

kDriveCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(60)
    .with_supply_current_limit_enable(True)
)

kSteerCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

kDriveAngularVelocityCoeff = 0.01  # while translating and rotating, need a bit extra motion to compensate for moving reference frame

# Pigeon
kPigeonCANId = 50

# Encoders
kFrontLeftSteerEncoderId = 10
kFrontRightSteerEncoderId = 11
kBackLeftSteerEncoderId = 12
kBackRightSteerEncoderId = 13

kConfigurationTimeoutLimit = int(5 * kMillisecondsPerSecond)
"""milliseconds"""

kDrivePIDSlot = 0
kDrivePGain = 60.0
kDriveIGain = 0.0
kDriveDGain = 0.0
kDriveVGain = 0
kDriveSGain = 2.3611

kSteerPIDSlot = 0
kSteerPGain = 100.0
kSteerIGain = 0.0
kSteerDGain = 0.5
kSteerVGain = 1.91
kSteerSGain = 0.1

kFrontLeftDriveInverted = True
kFrontRightDriveInverted = False
kBackLeftDriveInverted = True
kBackRightDriveInverted = False

kFrontLeftSteerInverted = False
kFrontRightSteerInverted = False
kBackLeftSteerInverted = False
kBackRightSteerInverted = False

"""
To determine encoder offsets (with robot ON and DISABLED):
  1. Rotate all swerve modules so that the wheels:
     * are running in the forwards-backwards direction
     * have the wheel bevel gears facing inwards towards the
       center-line of the robot
  2. Run Phoenix Tuner
  3. Select desired encoder
  4. Go to "Config" tab
  5. Click "Factory Default"
  6. Go to "Self-Test Snapshot" tab
  7. Click "Self-Test Snapshot"
  8. Record value from line: "Absolute Position (unsigned):"
"""
kFrontLeftAbsoluteEncoderOffset = -0.139892578125
"""rotations"""

kFrontRightAbsoluteEncoderOffset = -0.477783203125
"""rotations"""

kBackLeftAbsoluteEncoderOffset = 0.46142578125
"""rotations"""

kBackRightAbsoluteEncoderOffset = 0.015625
"""rotations"""

kNormalSpeedMultiplier = 0.50  # half full on normal
kTurboSpeedMultiplier = 0.95  # full speed!!!


kDriveKinematics = SwerveDrive4Kinematics(
    kFrontLeftWheelPosition,
    kFrontRightWheelPosition,
    kBackLeftWheelPosition,
    kBackRightWheelPosition,
)
