from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.hood.hoodsubsystemio import HoodSubsystemIO
from subsystems.hood.hoodsubsystemiotalon import HoodSubsystemIOTalon

from constants.hood import kHoodSimMotor, kHoodGearRatio, kHoodSimInertia
from constants.math import kRadiansPerRevolution
from constants import kRobotUpdatePeriod
from util.convenientmath import clamp


class HoodSubsystemIOSim(HoodSubsystemIOTalon):
    def __init__(self) -> None:
        super().__init__()
        self.hoodSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                kHoodSimMotor, kHoodSimInertia, kHoodGearRatio
            ),
            kHoodSimMotor,
        )

    def updateInputs(self, inputs: HoodSubsystemIO.HoodSubsystemIOInputs) -> None:
        hoodMotorSim = self.motor.sim_state
        simVoltage = RobotController.getInputVoltage()

        hoodAppliedVoltage = clamp(hoodMotorSim.motor_voltage, -12.0, 12.0)

        self.hoodSimModel.setInputVoltage(hoodAppliedVoltage)
        self.hoodSimModel.update(kRobotUpdatePeriod)

        hoodMotorSim.set_raw_rotor_position(
            self.hoodSimModel.getAngularPositionRotations() * kHoodGearRatio
        )
        hoodMotorSim.set_rotor_velocity(
            self.hoodSimModel.getAngularVelocity()
            / kRadiansPerRevolution
            * kHoodGearRatio
        )
        hoodMotorSim.set_supply_voltage(
            clamp(
                simVoltage - hoodMotorSim.supply_current * kHoodSimMotor.R,
                0,
                simVoltage,
            )
        )
        super().updateInputs(inputs)
