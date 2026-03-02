from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.geometry import Rotation2d, Rotation3d, Translation3d, Transform3d
from wpimath.system.plant import DCMotor

kTurretStartingAngle = Rotation2d.fromDegrees(90)
kTurretMinAngle = Rotation2d.fromDegrees(0)
kTurretMaxAngle = Rotation2d.fromDegrees(270)

kTurretGearRatio = (50 / 12) * (80 / 15)

kTurretMaxVelocity = Rotation2d.fromDegrees(180)  # per second
kTurretMaxAcceleration = Rotation2d.fromDegrees(360)  # per second squared
kTurretTolerance = Rotation2d.fromDegrees(1.0)

kTurretSafetyTolerance = Rotation2d.fromDegrees(5.0)

kTurretCanId = 9
kTurretPGain = 89.321
kTurretIGain = 0.0
kTurretDGain = 0.80233
kTurretSGain = 0.015852
kTurretVGain = 0.32494
kTurretAGain = 0.0039108

kTurretCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

kTurretSimMotor = DCMotor.krakenX44FOC(1)
kTurretSimInertia = 0.005  # kg m^2

kTurretLocation = Transform3d(
    Translation3d(-0.102, 0.178, 0.368),
    Rotation3d(),  # In cad this is the center of the top most plate on the turret
)
