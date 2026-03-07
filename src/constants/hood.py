from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor

kHoodMinAngle = Rotation2d.fromDegrees(0)
kHoodMaxAngle = Rotation2d.fromDegrees(37)
kHoodStartAngle = kHoodMinAngle
kHoodTolerance = Rotation2d.fromDegrees(1)
kHoodMaxVelocity = Rotation2d.fromDegrees(90)
kHoodMaxAcceleration = Rotation2d.fromDegrees(360)

kHoodFudgeAmount = Rotation2d.fromDegrees(1)

kHoodGearRatio = (4 / 1) * (360 / 13)

kHoodCANId = 8

kHoodPGain = 0.1
kHoodIGain = 0.0
kHoodDGain = 0.0
kHoodSGain = 0.0
kHoodVGain = 0.0
kHoodAGain = 0.0

kHoodCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(20)
    .with_supply_current_limit_enable(True)
)

kHoodSimMotor = DCMotor.krakenX44(1)
kHoodSimInertia = 0.002  # kg m^2
