from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.geometry import Rotation2d, Rotation3d, Translation3d, Transform3d
from wpimath.system.plant import DCMotor

kTurretMinAngle = Rotation2d.fromDegrees(-160)
kTurretMaxAngle = Rotation2d.fromDegrees(160)

kTurretGearRatio = (1 / 5) * (13 / 360)

kTurretMaxVelocity = Rotation2d.fromDegrees(180)  # per second
kTurretMaxAcceleration = Rotation2d.fromDegrees(360)  # per second squared
kTurretTolerance = Rotation2d.fromDegrees(1.0)

kTurretCanId = 22
kTurretPGain = 0.1
kTurretIGain = 0.0
kTurretDGain = 0.0
kTurretSGain = 0.0
kTurretVGain = 0.0
kTurretAGain = 0.0

kTurretCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

kTurretSimMotor = DCMotor.falcon500(1)
kTurretSimInertia = 0.005  # kg m^2

kTurretLocation = Transform3d(
    Translation3d(0.25, 0.25, 0.25), Rotation3d()
)  # FILL IN LATER
