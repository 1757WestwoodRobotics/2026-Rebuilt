from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.system.plant import DCMotor

kSpindexer1CANId = 3
kSpindexer2CANId = 4
kKickerLowerCANId = 5
kKickerUpperCANId = 6

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

kSpindexerSystem = DCMotor.falcon500(2)
kKickerSystem = DCMotor.falcon500(2)

kSpindexer1ForwardVoltage = 6.0
kSpindexer1ReverseVoltage = -6.0
kSpindexer2ForwardVoltage = 6.0
kSpindexer2ReverseVoltage = -6.0

kKickLowerForwardVoltage = 6.0
kKickLowerReverseVoltage = -6.0
kKickUpperForwardVoltage = -6.0
kKickUpperReverseVoltage = 6.0
