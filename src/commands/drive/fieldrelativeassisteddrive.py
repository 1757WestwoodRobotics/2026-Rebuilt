from math import pi
import typing

from pykit.logger import Logger
from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.geometry import Translation2d
from commands.drive.fieldrelativedrive import FieldRelativeDrive
from subsystems.drive.drivesubsystem import DriveSubsystem
from robotstate import RobotState

from constants.field import kTrenchCenters, kTrenchDepth
from constants.trajectory import (
    kTrenchAlignmentPGain,
    kTrenchAlignmentIGain,
    kTrenchAlignmentDGain,
    kRotationPGain,
    kRotationIGain,
    kRotationDGain,
)


class FieldRelativeAssistedDrive(FieldRelativeDrive):
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        sideways: typing.Callable[[], float],
        rotation: typing.Callable[[], float],
    ) -> None:
        super().__init__(drive, forward, sideways, rotation)

        self.rotationPid = PIDController(kRotationPGain, kRotationIGain, kRotationDGain)
        self.rotationPid.enableContinuousInput(-pi, pi)

        self.lineupPid = PIDController(
            kTrenchAlignmentPGain, kTrenchAlignmentIGain, kTrenchAlignmentDGain
        )
        self.setName(type(self).__name__)

    def execute(self) -> None:
        robotPose = RobotState.getPose()
        requestedRotation = self.rotation()

        closestTrench = robotPose.translation().nearest(kTrenchCenters)
        nearTrenchOpening = closestTrench - Translation2d(kTrenchDepth / 2, 0)
        farTrenchOpening = closestTrench + Translation2d(kTrenchDepth / 2, 0)

        nearstTrenchOpening = robotPose.translation().nearest(
            [nearTrenchOpening, farTrenchOpening]
        )

        Logger.recordOutput("Assist/NearestOpening", nearstTrenchOpening)

        shouldAssist = (
            robotPose.translation().distance(nearstTrenchOpening) < kTrenchDepth / 2
        )
        Logger.recordOutput("Assist/ShouldAssist", shouldAssist)

        if shouldAssist:
            # inside trench assist zone
            currentRotation = RobotState.getRotation().radians()
            assistedRotation = self.rotationPid.calculate(
                currentRotation, round(currentRotation / (pi / 2)) * (pi / 2)
            )  # face at nearest 90 deg
            sideways = self.lineupPid.calculate(
                robotPose.translation().y, closestTrench.y
            )
        else:
            assistedRotation = requestedRotation
            sideways = self.sideways()

        if (
            abs(self.forward()) < 0.01
            and abs(self.sideways()) < 0.01
            and abs(assistedRotation) < 0.01
        ):
            self.drive.defenseState()
        else:
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                # if we're on the other side, switch the controls around
                self.drive.arcadeDriveWithFactors(
                    -self.forward(),
                    -sideways,
                    assistedRotation,
                    DriveSubsystem.CoordinateMode.FieldRelative,
                )
            else:
                self.drive.arcadeDriveWithFactors(
                    self.forward(),
                    sideways,
                    assistedRotation,
                    DriveSubsystem.CoordinateMode.FieldRelative,
                )
