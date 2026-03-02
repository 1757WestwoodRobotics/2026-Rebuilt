from phoenix6.configs.talon_fx_configs import CurrentLimitsConfigs
from wpimath.system.plant import DCMotor

kFlywheelCANId = 7
kFlywheelMinSpeed = 0.2  # rad/s

kFlywheelPGain = 1.0976
kFlywheelIGain = 0.0
kFlywheelDGain = 0.0
kFlywheelSGain = 0.0
kFlywheelVGain = 0.01944
kFlywheelAGain = 0.0053331

kFlywheelCurrentLimit = (
    CurrentLimitsConfigs()
    .with_supply_current_limit(80)
    .with_supply_current_limit_enable(True)
)

kFlywheelGearing = 24 / 24  # dimensionless
kFlywheelTolerance = 5  # rad/s

kFlywheelSimMotor = DCMotor.krakenX60FOC(1)
kFlywheelSimInertia = 2 * 0.0011705586  # kg m^2

kFlywheelMaxAcceleration = (
    kFlywheelSimMotor.torque(kFlywheelCurrentLimit.supply_current_limit)
    * kFlywheelGearing
    / kFlywheelSimInertia
)  # rad/s^2
kFlywheelMaxVelocity = kFlywheelSimMotor.freeSpeed * kFlywheelGearing  # rad/s
