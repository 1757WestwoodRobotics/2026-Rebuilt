#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import typing
from ntcore import NetworkTableInstance
from phoenix6.sim.cancoder_sim_state import CANcoderSimState
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState
from phoenix6.unmanaged import feed_enable
from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.system.plant import DCMotor, LinearSystemId
import wpimath.kinematics
from pyfrc.physics.core import PhysicsInterface
from robot import MentorBot
from subsystems.drive.drivesubsystem import DriveSubsystem
from util.convenientmath import clamp
from util.motorsimulator import MotorSimulator

from constants.sim import (
    kSimulationRotationalInertia,
    kSimDefaultRobotLocation,
    kSimRobotVelocityArrayKey,
    kSimMotorResistance,
    kSimRobotPoseArrayKey,
)
from constants.drive import (
    kWheelRadius,
    kDriveGearingRatio,
    kSteerGearingRatio,
    kFrontLeftWheelPosition,
    kFrontRightWheelPosition,
    kBackLeftWheelPosition,
    kBackRightWheelPosition,
    kFrontLeftAbsoluteEncoderOffset,
    kFrontLeftDriveInverted,
    kFrontRightAbsoluteEncoderOffset,
    kFrontRightDriveInverted,
    kBackLeftAbsoluteEncoderOffset,
    kBackLeftDriveInverted,
    kBackRightAbsoluteEncoderOffset,
    kBackRightDriveInverted,
)
from constants.math import kRadiansPerRevolution


class SwerveModuleSim:
    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        position: Translation2d,
        wheelMotorType: DCMotor,
        wheelMotorSim: typing.Callable[[], TalonFXSimState],
        driveMotorGearing: float,
        swerveMotorType: DCMotor,
        swerveMotorSim: typing.Callable[[], TalonFXSimState],
        steerMotorGearing: float,
        swerveEncoderSim: typing.Callable[[], CANcoderSimState],
        encoderOffset: float,
        inverted: bool,
    ) -> None:
        self.position = position
        self.wheelMotorSim = wheelMotorSim
        self.wheelMotorType = wheelMotorType
        self.driveMotorGearing = driveMotorGearing
        self.wheelMotorInternalSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self.wheelMotorType,
                kSimulationRotationalInertia,
                self.driveMotorGearing,
            ),
            self.wheelMotorType,
        )
        self.swerveMotorSim = swerveMotorSim
        self.swerveMotorType = swerveMotorType
        self.steerMotorGearing = steerMotorGearing
        self.steerMotorIntenalSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                self.swerveMotorType,
                kSimulationRotationalInertia,
                self.steerMotorGearing,
            ),
            self.swerveMotorType,
        )
        self.swerveEncoderSim = swerveEncoderSim
        self.encoderOffset = encoderOffset + 0.25

        self.multiplier = -1 if inverted else 1

    def __str__(self) -> str:
        return f"pos: x={self.position.X():.2f} y={self.position.Y():.2f}"


class SwerveDriveSim:
    def __init__(self, swerveModuleSims: typing.Tuple[SwerveModuleSim, ...]) -> None:
        self.swerveModuleSims = swerveModuleSims
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            *(module.position for module in swerveModuleSims)
        )
        self.pose = kSimDefaultRobotLocation
        self.outputs = None

        self.robotVelocityPublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(kSimRobotVelocityArrayKey, wpimath.kinematics.ChassisSpeeds)
            .publish()
        )

    def resetPose(self, pose) -> None:
        self.pose = pose

    def getPose(self) -> Pose2d:
        return self.pose

    def getHeading(self) -> Rotation2d:
        return self.pose.rotation()

    def update(self, tm_diff: float, robotVoltage: float) -> None:
        deltaT = tm_diff

        states = []
        for module in self.swerveModuleSims:
            module.wheelMotorInternalSim.setInputVoltage(
                module.wheelMotorSim().motor_voltage
            )
            # print(module.wheelMotorSim().motor_voltage)
            module.wheelMotorInternalSim.update(tm_diff)
            wheel_position_rot = (
                module.wheelMotorInternalSim.getAngularPosition()
                / kRadiansPerRevolution
                * module.driveMotorGearing
            )
            wheel_velocity_rps = (
                module.wheelMotorInternalSim.getAngularVelocity()
                / kRadiansPerRevolution
                * module.driveMotorGearing
            )
            module.wheelMotorSim().set_raw_rotor_position(wheel_position_rot)
            module.wheelMotorSim().set_rotor_velocity(wheel_velocity_rps)
            module.wheelMotorSim().set_supply_voltage(
                clamp(
                    robotVoltage
                    - module.wheelMotorSim().supply_current * kSimMotorResistance,
                    0,
                    robotVoltage,
                )
            )

            module.steerMotorIntenalSim.setInputVoltage(
                module.swerveMotorSim().motor_voltage
            )
            module.steerMotorIntenalSim.update(tm_diff)
            swerve_position_rot = (
                module.steerMotorIntenalSim.getAngularPosition()
                / kRadiansPerRevolution
                * module.steerMotorGearing
            )
            swerve_velocity_rps = (
                module.steerMotorIntenalSim.getAngularVelocity()
                / kRadiansPerRevolution
                * module.steerMotorGearing
            )
            module.swerveMotorSim().set_raw_rotor_position(swerve_position_rot)
            module.swerveMotorSim().set_rotor_velocity(swerve_velocity_rps)
            module.swerveMotorSim().set_supply_voltage(
                clamp(
                    robotVoltage
                    - module.swerveMotorSim().supply_current * kSimMotorResistance,
                    0,
                    robotVoltage,
                )
            )
            module.swerveEncoderSim().set_raw_position(
                -swerve_position_rot / module.steerMotorGearing + module.encoderOffset
            )
            module.swerveEncoderSim().set_velocity(
                -swerve_velocity_rps / module.steerMotorGearing
            )

            wheelLinearVelocity = (
                wheel_velocity_rps
                * module.multiplier
                * kWheelRadius
                * kRadiansPerRevolution
                / kDriveGearingRatio
            )

            state = wpimath.kinematics.SwerveModuleState(
                -wheelLinearVelocity,
                Rotation2d(
                    -swerve_position_rot
                    / module.steerMotorGearing
                    * kRadiansPerRevolution
                ),
            )
            states.append(state)

        chassisSpeed = self.kinematics.toChassisSpeeds(states)
        deltaHeading = chassisSpeed.omega * deltaT
        deltaX = chassisSpeed.vx * deltaT
        deltaY = chassisSpeed.vy * deltaT

        self.robotVelocityPublisher.set(chassisSpeed)

        deltaTrans = Transform2d(deltaX, deltaY, deltaHeading)

        newPose = self.pose + deltaTrans
        self.pose = newPose


class PhysicsEngine:
    """
    Simulates a drivetrain
    """

    # pylint: disable-next=unused-argument
    def __init__(self, physics_controller: PhysicsInterface, robot: MentorBot):
        assert robot.container is not None
        self.physics_controller = physics_controller
        self.bot = robot

        driveSubsystem: DriveSubsystem = robot.container.drive

        frontLeftSim = driveSubsystem.frontLeftModule.getSimulator()
        self.frontLeftModuleSim = SwerveModuleSim(
            kFrontLeftWheelPosition,
            DCMotor.krakenX60(),
            frontLeftSim[0],
            kDriveGearingRatio,
            DCMotor.falcon500(),
            frontLeftSim[1],
            kSteerGearingRatio,
            frontLeftSim[2],
            kFrontLeftAbsoluteEncoderOffset,
            kFrontLeftDriveInverted,
        )
        frontRightSim = driveSubsystem.frontRightModule.getSimulator()
        self.frontRightModuleSim = SwerveModuleSim(
            kFrontRightWheelPosition,
            DCMotor.krakenX60(),
            frontRightSim[0],
            kDriveGearingRatio,
            DCMotor.falcon500(),
            frontRightSim[1],
            kSteerGearingRatio,
            frontRightSim[2],
            kFrontRightAbsoluteEncoderOffset,
            kFrontRightDriveInverted,
        )
        backLeftSim = driveSubsystem.backLeftModule.getSimulator()
        self.backSimLeftModule = SwerveModuleSim(
            kBackLeftWheelPosition,
            DCMotor.krakenX60(),
            backLeftSim[0],
            kDriveGearingRatio,
            DCMotor.falcon500(),
            backLeftSim[1],
            kSteerGearingRatio,
            backLeftSim[2],
            kBackLeftAbsoluteEncoderOffset,
            kBackLeftDriveInverted,
        )
        backRightSim = driveSubsystem.backRightModule.getSimulator()
        self.backSimRightModule = SwerveModuleSim(
            kBackRightWheelPosition,
            DCMotor.krakenX60(),
            backRightSim[0],
            kDriveGearingRatio,
            DCMotor.falcon500(),
            backRightSim[1],
            kSteerGearingRatio,
            backRightSim[2],
            kBackRightAbsoluteEncoderOffset,
            kBackRightDriveInverted,
        )

        self.swerveModuleSims = [
            self.frontLeftModuleSim,
            self.frontRightModuleSim,
            self.backSimLeftModule,
            self.backSimRightModule,
        ]

        self.driveSim = SwerveDriveSim(tuple(self.swerveModuleSims))

        driveSubsystem.resetSimPosition = self.driveSim.resetPose

        self.gyroSim = driveSubsystem.gyro.sim_state

        self.sim_initialized = False

        self.motorsim = MotorSimulator()

        self.simRobotPosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(kSimRobotPoseArrayKey, Pose2d)
            .publish()
        )

    # pylint: disable-next=unused-argument
    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        feed_enable(1 / 50)

        if not self.sim_initialized:
            self.sim_initialized = True
            # self.physics_controller.field, is not set until simulation_init

        self.gyroSim.set_raw_yaw(self.driveSim.getHeading().degrees())

        # Simulate the drivetrain
        voltage = RobotController.getInputVoltage()

        self.motorsim.update(tm_diff, voltage)
        self.driveSim.update(tm_diff, voltage)

        simRobotPose = self.driveSim.getPose()
        self.physics_controller.field.setRobotPose(simRobotPose)

        # publish the simulated robot pose to nt
        self.simRobotPosePublisher.set(simRobotPose)
