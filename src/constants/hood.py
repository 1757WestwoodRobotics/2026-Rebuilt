from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from phoenix6.signals import InvertedValue
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor

kHoodMinAngle = Rotation2d.fromDegrees(0)
kHoodMaxAngle = Rotation2d.fromDegrees(37)
kHoodFeedAngle = Rotation2d.fromDegrees(36)
kHoodStartAngle = kHoodMinAngle
kHoodTolerance = Rotation2d.fromDegrees(1)
kHoodMaxVelocity = Rotation2d.fromDegrees(360)
kHoodMaxAcceleration = Rotation2d.fromDegrees(360 * 4)

kHoodFudgeAmount = Rotation2d.fromDegrees(1)

kHoodGearRatio = (48 / 16) * (24 / 18) * (170 / 15)

kHoodCANId = 8

kHoodPGain = 200.0
kHoodIGain = 0.0
kHoodDGain = 0.0
kHoodSGain = 0.326
kHoodVGain = 0.0
kHoodAGain = 0.0

kHoodCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(20)
    .with_supply_current_limit_enable(True)
)
kHoodInvertedValue = InvertedValue.CLOCKWISE_POSITIVE

kHoodSimMotor = DCMotor.krakenX44(1)
kHoodSimInertia = 0.002  # kg m^2
