from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.system.plant import DCMotor

kSpindexerCANId = 3
kSpindexerCANId = 4
kKickerCANId = 5
kKicker1CANId = 6

kSpindexerCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(30)
    .with_supply_current_limit_enable(True)
)
kKickerCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(30)
    .with_supply_current_limit_enable(True)
)

kSpindexerGearRatio = 42 / 60
kKickerGearRatio = 50 / 24

kSpindexerMotor = DCMotor.falcon500(1)
kSpindexerMotor2 = DCMotor.falcon500(1)
kKickerMotor = DCMotor.falcon500(1)
kKickerMotor2 = DCMotor.falcon500(1)

kIndexerForwardVoltage = 6.0
kIndexerReverseVoltage = -6.0
