from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from subsystems.drive.swervemoduleiotalonfx import SwerveModuleIOCTRE
from subsystems.drive.swervemoduleio import SwerveModuleConfigParams, SwerveModuleIO

from constants.math import kRadiansPerRevolution
from constants.sim import kSimMotorResistance
from constants import kRobotUpdatePeriod
from util.convenientmath import clamp


class SwerveModuleIOSim(SwerveModuleIOCTRE):
    def __init__(self, name: str, config: SwerveModuleConfigParams) -> None:
        super().__init__(name, config)

        self.driveMotorModel = DCMotor.krakenX60(1)
        self.steerMotorModel = DCMotor.falcon500(1)

        self.driveSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self.driveMotorModel, 0.025, config.driveGearing
            ),
            self.driveMotorModel,
        )
        self.steerSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self.steerMotorModel, 0.004, config.steerGearing
            ),
            self.steerMotorModel,
        )

        self.wheelMotorSim, self.steerMotorSim, self.steerEncoderSim = (
            self.getSimulator()
        )

    def updateInputs(self, inputs: SwerveModuleIO.SwerveModuleIOInputs) -> None:
        wheelSim = self.wheelMotorSim()
        steerSim = self.steerMotorSim()
        encoderSim = self.steerEncoderSim()

        simVoltage = RobotController.getInputVoltage()

        self.driveSim.setInputVoltage(clamp(wheelSim.motor_voltage, -12, 12))
        self.steerSim.setInputVoltage(clamp(steerSim.motor_voltage, -12, 12))
        self.driveSim.update(kRobotUpdatePeriod)
        self.steerSim.update(kRobotUpdatePeriod)

        wheelSim.set_raw_rotor_position(
            self.driveSim.getAngularPositionRotations() * self.config.driveGearing
        )  # since the robot position is before mechanism ratio, we have to add the ratio ourselves
        wheelSim.set_rotor_velocity(
            self.driveSim.getAngularVelocity()
            / kRadiansPerRevolution
            * self.config.driveGearing
        )
        wheelSim.set_supply_voltage(
            clamp(
                simVoltage - wheelSim.supply_current * kSimMotorResistance,
                0,
                simVoltage,
            )
        )

        steerSim.set_raw_rotor_position(
            self.steerSim.getAngularPositionRotations() * self.config.steerGearing
        )
        steerSim.set_rotor_velocity(
            self.steerSim.getAngularVelocity()
            / kRadiansPerRevolution
            * self.config.steerGearing
        )
        steerSim.set_supply_voltage(
            clamp(
                simVoltage - steerSim.supply_current * kSimMotorResistance,
                0,
                simVoltage,
            )
        )

        encoderSim.set_raw_position(
            -self.steerSim.getAngularPositionRotations() + self.encoderOffset
        )
        encoderSim.set_velocity(
            -self.steerSim.getAngularVelocity() / kRadiansPerRevolution
        )

        return super().updateInputs(inputs)
