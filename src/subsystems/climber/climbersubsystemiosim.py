from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.climber.climbersubsystemio import ClimberSubsystemIO
from subsystems.climber.climbersubsystemiotalon import ClimberSubsystemIOTalon

from constants import kRobotUpdatePeriod

from constants.climber import (
    kClimberSimMotor,
    kClimberDistancePerRevolution,
    kClimberSimInertia,
)
from util.convenientmath import clamp


class ClimberSubsystemIOSim(ClimberSubsystemIOTalon):
    def __init__(self) -> None:
        super().__init__()
        self.climberSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                kClimberSimMotor, kClimberSimInertia, kClimberDistancePerRevolution
            ),
            kClimberSimMotor,
        )

    def updateInputs(self, inputs: ClimberSubsystemIO.ClimberSubsystemIOInputs):
        climberMotorSim = self.motor.sim_state
        simVoltage = RobotController.getInputVoltage()

        climberAppliedVoltage = clamp(climberMotorSim.motor_voltage, -12.0, 12.0)

        self.climberSimModel.setInputVoltage(climberAppliedVoltage)
        self.climberSimModel.update(kRobotUpdatePeriod)

        climberMotorSim.set_raw_rotor_position(
            self.climberSimModel.getAngularPositionRotations()
            * kClimberDistancePerRevolution
        )
        climberMotorSim.set_rotor_velocity(
            self.climberSimModel.getAngularVelocity() * kClimberDistancePerRevolution
        )
        climberMotorSim.set_supply_voltage(
            clamp(
                simVoltage - climberMotorSim.supply_current * kClimberSimMotor.R,
                0,
                simVoltage,
            )
        )
        super().updateInputs(inputs)
