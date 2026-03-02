from math import pi

from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.system.plant import DCMotor
from .math import kMetersPerInch

kClimberPositionTolerance = 0.05  # meters

kClimberMinHeight = 0.0  # meters
kClimberMaxHeight = 8.75 * kMetersPerInch  # meters


kClimberCANId = 10
kClimberGearRatio = (5 / 1) * (4 / 1)
kSpoolDiameter = 0.75 * kMetersPerInch

kClimberSpoolCircumference = kSpoolDiameter * pi

kClimberPGain = 10
kClimberIGain = 0.0
kClimberDGain = 0.0
kClimberSGain = 0.0
kClimberVGain = 0.0
kClimberAGain = 0.0

kClimberCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(40)
    .with_supply_current_limit_enable(True)
)

kClimberSimMotor = DCMotor.falcon500(1)
kClimberSimInertia = 0.005  # kg m^2

kDeployedHeight = 8.75 * kMetersPerInch  # meters
kRetractedHeight = 0.0  # meters

kClimberBumpAmount = 1 * kMetersPerInch  # meters
