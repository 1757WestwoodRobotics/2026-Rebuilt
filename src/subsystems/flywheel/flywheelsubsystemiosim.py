from wpilib import RobotController
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import LinearSystemId
from subsystems.flywheel.flywheelsubsystemio import FlywheelSubsystemIO
from subsystems.flywheel.flywheelsubsystemiotalon import FlywheelSubsystemIOTalon

from constants.flywheel import kFlywheelSimMotor, kFlywheelSimInertia, kFlywheelGearing

from constants.math import kRadiansPerRevolution
from constants import kRobotUpdatePeriod

from util.convenientmath import clamp


class FlywheelSubsystemIOSim(FlywheelSubsystemIOTalon):
    def __init__(self) -> None:
        super().__init__()

        self.flywheelSimModel = FlywheelSim(
            LinearSystemId.flywheelSystem(
                kFlywheelSimMotor, kFlywheelSimInertia, kFlywheelGearing
            ),
            kFlywheelSimMotor,
        )

    def updateInputs(
        self, inputs: FlywheelSubsystemIO.FlywheelSubsystemIOInputs
    ) -> None:
        flywheelMotorSim = self.motor.sim_state

        simVoltage = RobotController.getInputVoltage()

        flywheelAppliedVoltage = clamp(flywheelMotorSim.motor_voltage, -12.0, 12.0)

        self.flywheelSimModel.setInputVoltage(flywheelAppliedVoltage)
        self.flywheelSimModel.update(kRobotUpdatePeriod)

        flywheelVelocity = (
            self.flywheelSimModel.getAngularVelocity()
            * kFlywheelGearing
            / kRadiansPerRevolution
        )
        flywheelMotorSim.set_rotor_velocity(flywheelVelocity)
        flywheelMotorSim.add_rotor_position(flywheelVelocity * kRobotUpdatePeriod)

        flywheelMotorSim.set_supply_voltage(
            clamp(
                simVoltage
                - self.flywheelSimModel.getCurrentDraw() * kFlywheelSimMotor.R,
                0,
                12.0,
            )
        )

        super().updateInputs(inputs)
