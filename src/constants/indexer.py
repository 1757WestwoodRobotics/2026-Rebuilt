from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.system.plant import DCMotor

kSpindexerCANId = 22
kKickerCANId = 23

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
kKickerMotor = DCMotor.falcon500(1)

kSpindexerForwardVoltage = 6.0
kSpindexerReverseVoltage = -6.0

kKickForwardVoltage = 6.0
kKickReverseVoltage = -6.0
