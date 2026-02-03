from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.system.plant import DCMotor


kIndexerCANId = 22
kFeederCANId = 23

kIndexerCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(30)
    .with_supply_current_limit_enable(True)
)
kFeederCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(30)
    .with_supply_current_limit_enable(True)
)

kIndexerGearRatio = 1.0
kFeederGearRatio = 1.0

kIndexerMotor = DCMotor.falcon500(1)
kFeederMotor = DCMotor.falcon500(1)

kIndexerForwardVoltage = 6.0
kIndexerReverseVoltage = -6.0

kFeedForwardVoltage = 6.0
kFeedReverseVoltage = -6.0
