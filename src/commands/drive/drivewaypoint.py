from math import pi

from commands2 import Command
from pathplannerlib.config import ChassisSpeeds
from pykit.autolog import autolog_output
from pykit.logger import Logger
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d
from wpilib import DataLogManager, DriverStation

from robotstate import RobotState
from subsystems.drive.drivesubsystem import DriveSubsystem

from util.controltype import AnalogInput


from constants.trajectory import (
    kWaypointJoystickVariation,
    kTrajectoryAnglePGain,
    kTrajectoryAngleIGain,
    kTrajectoryAngleDGain,
    kTrajectoryPositionPGainVision,
    kTrajectoryPositionIGain,
    kTrajectoryPositionDGain,
)

from constants.drive import (
    kMaxForwardLinearVelocity,
    kMaxForwardLinearAccelerationWaypoint,
    kMaxRotationAngularVelocity,
    kMaxRotationAngularAcceleration,
)
from constants.math import kMetersPerInch


class DriveWaypoint(Command):
    def __init__(
        self, drive: DriveSubsystem, xOffset: AnalogInput, yOffset: AnalogInput
    ) -> None:
        Command.__init__(self)
        self.setName(type(self).__name__)

        self.drive = drive

        self.command = Command()

        self.running = False
        self.targetPose = Pose2d()
        self.addRequirements(self.drive)

        self.xoff = xOffset
        self.yoff = yOffset

        self.xController = ProfiledPIDController(
            kTrajectoryPositionPGainVision,
            kTrajectoryPositionIGain,
            kTrajectoryPositionDGain,
            TrapezoidProfile.Constraints(
                kMaxForwardLinearVelocity,
                kMaxForwardLinearAccelerationWaypoint,
            ),
        )
        self.yController = ProfiledPIDController(
            kTrajectoryPositionPGainVision,
            kTrajectoryPositionIGain,
            kTrajectoryPositionDGain,
            TrapezoidProfile.Constraints(
                kMaxForwardLinearVelocity,
                kMaxForwardLinearAccelerationWaypoint,
            ),
        )
        self.thetaController = ProfiledPIDControllerRadians(
            kTrajectoryAnglePGain,
            kTrajectoryAngleIGain,
            kTrajectoryAngleDGain,
            TrapezoidProfileRadians.Constraints(
                kMaxRotationAngularVelocity,
                kMaxRotationAngularAcceleration,
            ),
        )

        self.thetaController.enableContinuousInput(-pi, pi)

    def initialize(self):
        self.running = True
        # pylint: disable=W0201

        currentPose = RobotState.getPose()
        self.xController.reset(currentPose.X())
        self.yController.reset(currentPose.Y())

        self.thetaController.reset(RobotState.getRotation().radians(), 0)

    def execute(self) -> None:
        currentPose = RobotState.getPose()

        absoluteOutput = ChassisSpeeds(
            self.xController.calculate(
                currentPose.X(),
                self.targetPose.X() + self.xoff() * kWaypointJoystickVariation,
            ),
            self.yController.calculate(
                currentPose.Y(),
                self.targetPose.Y() + self.yoff() * kWaypointJoystickVariation,
            ),
            self.thetaController.calculate(
                RobotState.getRotation().radians(), self.targetPose.rotation().radians()
            ),
        )
        Logger.recordOutput("waypoint/output", absoluteOutput)

        self.drive.arcadeDriveWithSpeeds(
            absoluteOutput, DriveSubsystem.CoordinateMode.FieldRelative
        )

    @autolog_output("waypoint/atPosition")
    def atPosition(self) -> bool:
        return (
            self.targetPose.translation().distance(RobotState.getPose().translation())
            < (1 if DriverStation.isAutonomous() else 2) * kMetersPerInch
        )

    def isFinished(self) -> bool:
        return self.atPosition() if DriverStation.isAutonomous() else False

    def end(self, _interrupted: bool) -> None:
        # pylint: disable=W0212
        DataLogManager.log("... DONE")
