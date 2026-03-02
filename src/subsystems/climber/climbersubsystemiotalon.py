from phoenix6 import BaseStatusSignal
from phoenix6.configs.talon_fx_configs import Slot0Configs, TalonFXConfiguration
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware.talon_fx import TalonFX
from subsystems.climber.climbersubsystemio import ClimberSubsystemIO

from constants import kRobotUpdateFrequency
from constants.climber import (
    kClimberCANId,
    kClimberCurrentLimit,
    kClimberGearRatio,
    kClimberSpoolCircumference,
    kClimberPGain,
    kClimberIGain,
    kClimberDGain,
    kClimberSGain,
    kClimberVGain,
    kClimberAGain,
)
from util.phoenixutil import PhoenixUtil, tryUntilOk


class ClimberSubsystemIOTalon(ClimberSubsystemIO):
    config: TalonFXConfiguration = TalonFXConfiguration()

    closedDemand: PositionVoltage = PositionVoltage(0)
    openDemand: VoltageOut = VoltageOut(0)

    def __init__(self) -> None:
        self.motor = TalonFX(kClimberCANId)

        self.config.current_limits = kClimberCurrentLimit
        self.config.feedback.sensor_to_mechanism_ratio = kClimberGearRatio
        self.config.slot0 = (
            Slot0Configs()
            .with_k_p(kClimberPGain)
            .with_k_i(kClimberIGain)
            .with_k_d(kClimberDGain)
            .with_k_s(kClimberSGain)
            .with_k_v(kClimberVGain)
            .with_k_a(kClimberAGain)
        )

        tryUntilOk(5, lambda: self.motor.configurator.apply(self.config))

        self.position = self.motor.get_position()
        self.velocity = self.motor.get_velocity()

        self.applied = self.motor.get_motor_voltage()
        self.supply = self.motor.get_supply_current()
        self.torque = self.motor.get_torque_current()

        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency,
            self.position,
            self.velocity,
            self.applied,
            self.supply,
            self.torque,
        )
        PhoenixUtil.registerSignals(
            "", self.position, self.velocity, self.applied, self.supply, self.torque
        )

    def updateInputs(self, inputs: ClimberSubsystemIO.ClimberSubsystemIOInputs):
        inputs.climberConnected = BaseStatusSignal.is_all_good(
            self.position,
            self.velocity,
            self.applied,
            self.supply,
            self.torque,
        )
        inputs.climberPosition = self.position.value * kClimberSpoolCircumference
        inputs.climberSpeed = self.velocity.value * kClimberSpoolCircumference
        inputs.climberAppliedVolts = self.applied.value
        inputs.climberSupplyAmps = self.supply.value
        inputs.climberTorqueAmps = self.torque.value

    def set_climber_position(self, position: float):
        self.motor.set_control(
            self.closedDemand.with_position(position / kClimberSpoolCircumference)
        )

    def set_climber_volts(self, volts: float):
        self.motor.set_control(self.openDemand.with_output(volts))
