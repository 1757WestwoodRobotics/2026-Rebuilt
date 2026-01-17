from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import LinearSystemId
from subsystems.turret.turretsubsystemio import TurretSubsystemIO
from subsystems.turret.turretsubsystemiotalon import TurretSubsystemIOTalon
from util.convenientmath import clamp

from constants.turret import kTurretSimMotor, kTurretGearRatio, kTurretSimInertia
from constants.math import kRadiansPerRevolution
from constants import kRobotUpdatePeriod
from util.convenientmath import clamp


class TurretSubsystemIOSim(TurretSubsystemIOTalon):
    def __init__(self) -> None:
        super().__init__()
        self.turretSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                kTurretSimMotor, kTurretSimInertia, kTurretGearRatio
            ),
            kTurretSimMotor,
        )

    def updateInputs(self, inputs: TurretSubsystemIO.TurretSubsystemIOInputs) -> None:
        turretMotorSim = self.motor.sim_state
        simVoltage = RobotController.getInputVoltage()

        turretAppliedVoltage = clamp(turretMotorSim.motor_voltage, -12.0, 12.0)

        self.turretSimModel.setInputVoltage(turretAppliedVoltage)
        self.turretSimModel.update(kRobotUpdatePeriod)

        turretMotorSim.set_raw_rotor_position(
            self.turretSimModel.getAngularPositionRotations() * kTurretGearRatio
        )
        turretMotorSim.set_rotor_velocity(
            self.turretSimModel.getAngularVelocity()
            / kRadiansPerRevolution
            * kTurretGearRatio
        )
        turretMotorSim.set_supply_voltage(
            clamp(
                simVoltage - turretMotorSim.supply_current * kTurretSimMotor.R,
                0,
                simVoltage,
            )
        )
        super().updateInputs(inputs)
