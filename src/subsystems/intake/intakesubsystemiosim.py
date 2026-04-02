from wpilib import RobotController
from wpilib.simulation import DCMotorSim, SingleJointedArmSim
from wpimath.system.plant import LinearSystemId


from constants import kRobotUpdatePeriod
from constants.math import kRadiansPerRevolution
from constants.intake import (
    kPivotMass,
    kPivotGearRatio,
    kPivotMinAngle,
    kPivotMaxAngle,
    kPivotStartAngle,
    kPivotMotor,
    kPivotArmLength,
    kRollerMotor,
    kRollerGearRatio,
)
from subsystems.intake.intakesubsystemiotalon import IntakeSubsystemIOTalon
from subsystems.intake.intakesubsystemio import IntakeSubsystemIO
from util.convenientmath import clamp


class IntakeSubsystemIOSim(IntakeSubsystemIOTalon):
    def __init__(self) -> None:
        super().__init__()

        self.pivotSimModel = SingleJointedArmSim(
            LinearSystemId.singleJointedArmSystem(
                kPivotMotor,
                kPivotMass * kPivotArmLength * kPivotArmLength / 3,
                kPivotGearRatio,  # Approximate moment of inertia as long as a rod rotating about one end
            ),
            kPivotMotor,
            kPivotGearRatio,
            kPivotArmLength,
            -kPivotMaxAngle.radians(),
            kPivotMinAngle.radians(),
            True,
            -kPivotStartAngle.radians(),  # negate such that the zero offset is set properly
        )
        self.rollerSimModel = DCMotorSim(
            LinearSystemId.DCMotorSystem(kRollerMotor, 0.04, kRollerGearRatio),
            kRollerMotor,
        )

    def updateInputs(self, inputs: IntakeSubsystemIO.IntakeSubsystemIOInputs) -> None:
        pivotMotorSim = self.pivotMotor.sim_state
        rollerMotorSim = self.rollerMotor.sim_state

        simVoltage = RobotController.getInputVoltage()

        pivotAppliedVoltage = clamp(pivotMotorSim.motor_voltage, -12.0, 12.0)
        rollerAppliedVoltage = clamp(rollerMotorSim.motor_voltage, -12.0, 12.0)

        self.pivotSimModel.setInputVoltage(pivotAppliedVoltage)
        self.rollerSimModel.setInputVoltage(rollerAppliedVoltage)
        self.pivotSimModel.update(kRobotUpdatePeriod)
        self.rollerSimModel.update(kRobotUpdatePeriod)

        pivotMotorSim.set_raw_rotor_position(
            self.pivotSimModel.getAngle() * kPivotGearRatio / kRadiansPerRevolution
        )
        pivotMotorSim.set_rotor_velocity(
            self.pivotSimModel.getVelocity() * kPivotGearRatio / kRadiansPerRevolution
        )
        pivotMotorSim.set_supply_voltage(
            clamp(
                simVoltage - pivotMotorSim.supply_current * kPivotMotor.R,
                0,
                simVoltage,
            )
        )

        rollerMotorSim.set_raw_rotor_position(
            self.rollerSimModel.getAngularPositionRotations() * kRollerGearRatio
        )
        rollerMotorSim.set_rotor_velocity(
            self.rollerSimModel.getAngularVelocity()
            / kRadiansPerRevolution
            * kRollerGearRatio
        )
        rollerMotorSim.set_supply_voltage(
            clamp(
                simVoltage - rollerMotorSim.supply_current * kRollerMotor.R,
                0,
                simVoltage,
            )
        )

        super().updateInputs(inputs)
