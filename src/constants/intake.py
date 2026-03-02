from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor

from .math import kKilogramsPerLb, kMetersPerInch

kPivotMotor = DCMotor.krakenX60FOC(1)

kPivotMass = 7 * kKilogramsPerLb  # kg
kPivotGearRatio = 64 / 1  # dimensionless
kPivotArmLength = 4.843 * kMetersPerInch  # meters

kPivotCANId = 1

# to tune SysID, use the autonomous command for the pivot, and then
# use https://docs.advantagekit.org/data-flow/sysid-compatibility/#loading-data for how to export
# from pykit and load into SysID. Make sure your units are proper and your controller gain preset
# is set to phoenix6
kPivotKp = 129.07
kPivotKi = 0
kPivotKd = 2.1043
kPivotKs = 0.024737
kPivotKv = 1.1902
kPivotKa = 0.011424
kPivotKg = 0.036956

kPivotMaxVelocity = 4.0  # rad / sec
kPivotMaxAcceleration = 4.0  # rad / sec ^2
kPivotConstraints = TrapezoidProfileRadians.Constraints(
    kPivotMaxVelocity, kPivotMaxAcceleration
)

# Positive angle is upwards, 0 angle is horizontal, and negative angle is downwards
kPivotMinAngle = Rotation2d()
kPivotMaxAngle = Rotation2d.fromDegrees(125.5)
kPivotStartAngle = Rotation2d.fromDegrees(125.5)

kPivotTolerance = Rotation2d.fromDegrees(2)

kPivotCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

kRollerMotor = DCMotor.krakenX60FOC(1)
kRollerGearRatio = 24 / 18

kPivotRetractedPosition = Rotation2d.fromDegrees(125.5)
kPivotDepotPosition = Rotation2d.fromDegrees(5)
kPivotExtendedPosition = Rotation2d()

kRollerCANId = 2

kRollerForwardVoltage = 6.0
kRollerReverseVoltage = -6.0

kRollerCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(60)
    .with_supply_current_limit_enable(True)
)

kPivotBumpAmount = Rotation2d.fromDegrees(5)
