from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from phoenix6.signals import InvertedValue
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
kSpindexerInvertedValue = InvertedValue.CLOCKWISE_POSITIVE
kKickerCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(30)
    .with_supply_current_limit_enable(True)
)

kSpindexerGearRatio = 42 / 60
kKickerGearRatio = 50 / 24

kSpindexerSystem = DCMotor.falcon500(2)
kKickerSystem = DCMotor.falcon500(2)

kSpindexer1ForwardVoltage = 5.0
kSpindexer1ReverseVoltage = -3.0
kSpindexer2ForwardVoltage = 5.0
kSpindexer2ReverseVoltage = -3.0

kSpindexerReverseSlowVoltage = -1.0

kKickLowerForwardVoltage = 3.0
kKickLowerReverseVoltage = -3.0
kKickUpperForwardVoltage = -3.0
kKickUpperReverseVoltage = 3.0
