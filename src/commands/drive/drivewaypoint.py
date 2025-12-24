from math import pi
from typing import Callable

from commands2 import Command
from pathplannerlib.config import ChassisSpeeds
from pykit.autolog import autolog_output
from pykit.logger import Logger
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d
from wpilib import DriverStation

from robotstate import RobotState
from subsystems.drive.drivesubsystem import DriveSubsystem

from util.controltype import AnalogInput
from util.logtunablenumber import LoggedTunableNumber

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
from constants.math import kMetersPerInch, kRadiansPerDegree


class DriveWaypoint(Command):
    driveKp = LoggedTunableNumber(
        "DriveWaypoint/driveKp", kTrajectoryPositionPGainVision
    )
    driveKi = LoggedTunableNumber("DriveWaypoint/driveKi", kTrajectoryPositionIGain)
    driveKd = LoggedTunableNumber("DriveWaypoint/driveKd", kTrajectoryPositionDGain)
    angleKp = LoggedTunableNumber("DriveWaypoint/angleKp", kTrajectoryAnglePGain)
    angleKi = LoggedTunableNumber("DriveWaypoint/angleKi", kTrajectoryAngleIGain)
    angleKd = LoggedTunableNumber("DriveWaypoint/angleKd", kTrajectoryAngleDGain)

    driveMaxVel = LoggedTunableNumber(
        "DriveWaypoint/driveMaxVel", kMaxForwardLinearVelocity
    )
    driveMaxAccel = LoggedTunableNumber(
        "DriveWaypoint/driveMaxAccel", kMaxForwardLinearAccelerationWaypoint
    )
    angleMaxVel = LoggedTunableNumber(
        "DriveWaypoint/angleMaxVel", kMaxRotationAngularVelocity
    )
    angleMaxAccel = LoggedTunableNumber(
        "DriveWaypoint/angleMaxAccel", kMaxRotationAngularAcceleration
    )

    driveTolerance = LoggedTunableNumber(
        "DriveWaypoint/driveTolerance", 2 * kMetersPerInch
    )
    angleTolerance = LoggedTunableNumber("DriveWaypoint/angleToleranceDegrees", 3.0)

    def __init__(
        self,
        drive: DriveSubsystem,
        xOffset: AnalogInput,
        yOffset: AnalogInput,
        targetSupplier: Callable[[], Pose2d],
    ) -> None:
        Command.__init__(self)
        self.setName(type(self).__name__)

        self.drive = drive

        self.command = Command()

        self.running = False
        self.targetSupplier = targetSupplier
        self.addRequirements(self.drive)

        self.xoff = xOffset
        self.yoff = yOffset

        self.xController = ProfiledPIDController(
            self.driveKp.get(),
            self.driveKi.get(),
            self.driveKd.get(),
            TrapezoidProfile.Constraints(
                self.driveMaxVel.get(), self.driveMaxAccel.get()
            ),
        )
        self.yController = ProfiledPIDController(
            self.driveKp.get(),
            self.driveKi.get(),
            self.driveKd.get(),
            TrapezoidProfile.Constraints(
                self.driveMaxVel.get(), self.driveMaxAccel.get()
            ),
        )
        self.thetaController = ProfiledPIDControllerRadians(
            self.angleKp.get(),
            self.angleKi.get(),
            self.angleKd.get(),
            TrapezoidProfileRadians.Constraints(
                self.angleMaxVel.get(), self.angleMaxAccel.get()
            ),
        )

        self.thetaController.enableContinuousInput(-pi, pi)
        self.xController.setTolerance(self.driveTolerance.get())
        self.yController.setTolerance(self.driveTolerance.get())
        self.thetaController.setTolerance(self.angleTolerance.get() * kRadiansPerDegree)

        self.driveKp.onChange(self.xController.setP)
        self.driveKi.onChange(self.xController.setI)
        self.driveKd.onChange(self.xController.setD)

        self.driveKp.onChange(self.yController.setP)
        self.driveKi.onChange(self.yController.setI)
        self.driveKd.onChange(self.yController.setD)

        self.angleKp.onChange(self.thetaController.setP)
        self.angleKi.onChange(self.thetaController.setI)
        self.angleKd.onChange(self.thetaController.setD)

        self.driveMaxVel.onChange(
            lambda value: self.xController.setConstraints(
                TrapezoidProfile.Constraints(value, self.driveMaxAccel.get())
            )
        )
        self.driveMaxAccel.onChange(
            lambda value: self.xController.setConstraints(
                TrapezoidProfile.Constraints(self.driveMaxVel.get(), value)
            )
        )
        self.driveMaxVel.onChange(
            lambda value: self.yController.setConstraints(
                TrapezoidProfile.Constraints(value, self.driveMaxAccel.get())
            )
        )
        self.driveMaxAccel.onChange(
            lambda value: self.yController.setConstraints(
                TrapezoidProfile.Constraints(self.driveMaxVel.get(), value)
            )
        )

        self.angleMaxVel.onChange(
            lambda value: self.thetaController.setConstraints(
                TrapezoidProfileRadians.Constraints(value, self.angleMaxAccel.get())
            )
        )
        self.angleMaxAccel.onChange(
            lambda value: self.thetaController.setConstraints(
                TrapezoidProfileRadians.Constraints(self.angleMaxVel.get(), value)
            )
        )

        self.driveTolerance.onChange(self.xController.setTolerance)
        self.driveTolerance.onChange(self.yController.setTolerance)

        self.angleTolerance.onChange(self.thetaController.setTolerance)

        self.targetPose = self.targetSupplier()

    def initialize(self):
        self.running = True
        # pylint: disable=W0201

        currentPose = RobotState.getPose()
        self.xController.reset(currentPose.X())
        self.yController.reset(currentPose.Y())

        self.thetaController.reset(RobotState.getRotation().radians(), 0)

    def execute(self) -> None:
        currentPose = RobotState.getPose()
        self.targetPose = self.targetSupplier()

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
            self.xController.atGoal()
            and self.yController.atGoal()
            and self.thetaController.atGoal()
        )

    def isFinished(self) -> bool:
        return self.atPosition() if DriverStation.isAutonomous() else False

    def end(self, _interrupted: bool) -> None:
        # pylint: disable=W0212
        print("... DONE")
