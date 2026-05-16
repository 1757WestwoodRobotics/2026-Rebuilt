from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from phoenix6.signals import InvertedValue
from wpimath.geometry import Rotation2d, Rotation3d, Translation3d, Transform3d
from wpimath.system.plant import DCMotor

kTurretStartingAngle = Rotation2d.fromDegrees(90)
kTurretMinAngle = Rotation2d.fromDegrees(45)
kTurretMaxAngle = Rotation2d.fromDegrees(270)

kTurretGearRatio = (50 / 12) * (80 / 15)

kTurretMaxVelocity = Rotation2d.fromDegrees(600)  # per second
kTurretMaxAcceleration = Rotation2d.fromDegrees(360 * 4)  # per second squared
kTurretTolerance = Rotation2d.fromDegrees(5.0)

kTurretSafetyTolerance = Rotation2d.fromDegrees(5.0)

kTurretCanId = 9
kTurretPGain = 40.321
kTurretIGain = 0.0
kTurretDGain = 0.80233
kTurretSGain = 0.5
kTurretVGain = 0.5
kTurretAGain = 0.0039108


kTurretCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

kTurretSimMotor = DCMotor.krakenX44FOC(1)
kTurretSimInertia = 0.2  # kg m^2

kTurretInvertedValue = InvertedValue.CLOCKWISE_POSITIVE

kTurretLocation = Transform3d(
    Translation3d(-0.102, 0.178, 0.368),
    Rotation3d(),  # In cad this is the center of the top most plate on the turret
)
