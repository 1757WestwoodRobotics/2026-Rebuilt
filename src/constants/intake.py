from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor

from .math import kKilogramToLbs, kMetersPerInch

kPivotMotor = DCMotor.krakenX60(1)

kPivotMass = 7 * kKilogramToLbs  # kg
kPivotGearRatio = 64 / 1  # dimensionless
kPivotArmLength = 4.843 * kMetersPerInch  # meters

kPivotCANId = 1

kPivotKp = 210.68
kPivotKi = 0
kPivotKd = 2.849
kPivotKs = 0.03105
kPivotKv = 4.4882
kPivotKa = 0.061551
kPivotKg = 0.33261

kPivotMaxVelocity = 2.0  # rad / sec
kPivotMaxAcceleration = 2.0  # rad / sec ^2
kPivotConstraints = TrapezoidProfileRadians.Constraints(
    kPivotMaxVelocity, kPivotMaxAcceleration
)

kPivotMinAngle = Rotation2d()
kPivotMaxAngle = Rotation2d.fromDegrees(90)

kPivotTolerance = Rotation2d.fromDegrees(2)

kPivotCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

kRollerMotor = DCMotor.krakenX60FOC(1)
kRollerGearRatio = 18 / 24

kPivotRetractedPosition = Rotation2d()
kPivotExtendedPosition = Rotation2d.fromDegrees(90)

kRollerCANId = 2

kRollerForwardVoltage = 6.0
kRollerReverseVoltage = -6.0

kRollerCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(60)
    .with_supply_current_limit_enable(True)
)
