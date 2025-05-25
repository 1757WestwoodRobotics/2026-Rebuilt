from math import pi

from commands2 import Command
from pathplannerlib.config import ChassisSpeeds
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d
from wpilib import DataLogManager, RobotState, SmartDashboard
from ntcore import NetworkTableInstance

from subsystems.drivesubsystem import DriveSubsystem
from operatorinterface import AnalogInput
import constants


class DriveWaypoint(Command):
    def __init__(
        self, drive: DriveSubsystem, xOffset: AnalogInput, yOffset: AnalogInput
    ) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive

        self.command = Command()

        self.running = False
        self.targetPose = Pose2d()
        self.addRequirements(self.drive)

        self.xoff = xOffset
        self.yoff = yOffset

        self.driveVelocity = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kDriveVelocityKeys, ChassisSpeeds)
            .subscribe(ChassisSpeeds())
        )
        self.waypointAtTarget = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(constants.kWaypointAtTargetKey)
            .publish()
        )

        self.xController = ProfiledPIDController(
            constants.kTrajectoryPositionPGainVision,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
            TrapezoidProfile.Constraints(
                constants.kMaxForwardLinearVelocity,
                constants.kMaxForwardLinearAccelerationWaypoint,
            ),
        )
        self.yController = ProfiledPIDController(
            constants.kTrajectoryPositionPGainVision,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain,
            TrapezoidProfile.Constraints(
                constants.kMaxForwardLinearVelocity,
                constants.kMaxForwardLinearAccelerationWaypoint,
            ),
        )
        self.thetaController = ProfiledPIDControllerRadians(
            constants.kTrajectoryAnglePGain,
            constants.kTrajectoryAngleIGain,
            constants.kTrajectoryAngleDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        SmartDashboard.putData(
            constants.kTargetWaypointXControllerKey, self.xController
        )
        SmartDashboard.putData(
            constants.kTargetWaypointYControllerKey, self.yController
        )
        SmartDashboard.putData(
            constants.kTargetWaypointThetaControllerKey, self.thetaController
        )

        self.thetaController.enableContinuousInput(-pi, pi)

    def initialize(self):
        self.running = True
        # pylint: disable=W0201

        currentPose = self.drive.getPose()
        self.xController.reset(currentPose.X())
        self.yController.reset(currentPose.Y())

        self.thetaController.reset(self.drive.getRotation().radians(), 0)

    def execute(self) -> None:
        currentPose = self.drive.getPose()

        absoluteOutput = ChassisSpeeds(
            self.xController.calculate(
                currentPose.X(),
                self.targetPose.X()
                + self.xoff() * constants.kWaypointJoystickVariation,
            ),
            self.yController.calculate(
                currentPose.Y(),
                self.targetPose.Y()
                + self.yoff() * constants.kWaypointJoystickVariation,
            ),
            self.thetaController.calculate(
                self.drive.getRotation().radians(), self.targetPose.rotation().radians()
            ),
        )

        self.drive.arcadeDriveWithSpeeds(
            absoluteOutput, DriveSubsystem.CoordinateMode.FieldRelative
        )
        self.waypointAtTarget.set(self.atPosition())


    def atPosition(self) -> bool:
        return (
            self.targetPose.translation().distance(self.drive.getPose().translation())
            < (1 if RobotState.isAutonomous() else 2) * constants.kMetersPerInch
        )

    def isFinished(self) -> bool:
        return self.atPosition() if RobotState.isAutonomous() else False

    def end(self, _interrupted: bool) -> None:
        # pylint: disable=W0212
        DataLogManager.log("... DONE")


