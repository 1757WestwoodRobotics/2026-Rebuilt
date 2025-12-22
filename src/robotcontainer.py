import os

from pykit.logger import Logger
from pykit.alertlogger import AlertLogger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser

import wpilib
from wpimath.geometry import Pose2d
import commands2
from pathplannerlib.auto import PathPlannerAuto

from commands.drive.fieldrelativedrive import FieldRelativeDrive
from commands.drive.anglealign import AngleAlignDrive
from commands.defensestate import DefenseState

from commands.resetgyro import ResetGyro
from robotmechanism import RobotMechanism
from robotstate import RobotState
from subsystems.drive.driveiopigeon import DriveIOPigeon
from subsystems.drive.drivesubsystem import DriveSubsystem
from subsystems.drive.swervemoduleio import SwerveModuleConfigParams, SwerveModuleIO
from subsystems.drive.swervemoduleiosim import SwerveModuleIOSim
from subsystems.drive.swervemoduleiotalonfx import SwerveModuleIOCTRE
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniolimelight import VisionSubsystemIOLimelight
from subsystems.vision.visioniophotonsim import VisionSubsystemIOPhotonSim
from subsystems.vision.visionsubsystem import VisionSubsystem
from subsystems.drive.driveio import DriveIO

from operatorinterface import OperatorInterface

from constants.auto import kAutoDuration
from constants.vision import kRobotToCamera1Transform, kRobotToCamera2Transform
from constants.drive import (
    kTurboSpeedMultiplier,
    kNormalSpeedMultiplier,
    kFrontLeftModuleName,
    kFrontLeftDriveMotorId,
    kFrontLeftDriveInverted,
    kFrontLeftSteerMotorId,
    kFrontLeftSteerInverted,
    kFrontLeftSteerEncoderId,
    kFrontLeftAbsoluteEncoderOffset,
    kCANivoreName,
    kFrontRightModuleName,
    kFrontRightDriveMotorId,
    kFrontRightDriveInverted,
    kFrontRightSteerMotorId,
    kFrontRightSteerInverted,
    kFrontRightSteerEncoderId,
    kFrontRightAbsoluteEncoderOffset,
    kBackLeftModuleName,
    kBackLeftDriveMotorId,
    kBackLeftDriveInverted,
    kBackLeftSteerMotorId,
    kBackLeftSteerInverted,
    kBackLeftSteerEncoderId,
    kBackLeftAbsoluteEncoderOffset,
    kBackRightModuleName,
    kBackRightDriveMotorId,
    kBackRightDriveInverted,
    kBackRightSteerMotorId,
    kBackRightSteerInverted,
    kBackRightSteerEncoderId,
    kBackRightAbsoluteEncoderOffset,
)
from constants import RobotModes, kRobotMode
from util.fliputil import FlipUtil


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        match kRobotMode:
            case RobotModes.REAL:
                wpilib.DataLogManager.log("Starting REAL")
                self.drive = DriveSubsystem(
                    DriveIOPigeon(),
                    (
                        SwerveModuleIOCTRE(
                            kFrontLeftModuleName,
                            SwerveModuleConfigParams(
                                kFrontLeftDriveMotorId,
                                kFrontLeftDriveInverted,
                                kFrontLeftSteerMotorId,
                                kFrontLeftSteerInverted,
                                kFrontLeftSteerEncoderId,
                                kFrontLeftAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                        SwerveModuleIOCTRE(
                            kFrontRightModuleName,
                            SwerveModuleConfigParams(
                                kFrontRightDriveMotorId,
                                kFrontRightDriveInverted,
                                kFrontRightSteerMotorId,
                                kFrontRightSteerInverted,
                                kFrontRightSteerEncoderId,
                                kFrontRightAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                        SwerveModuleIOCTRE(
                            kBackLeftModuleName,
                            SwerveModuleConfigParams(
                                kBackLeftDriveMotorId,
                                kBackLeftDriveInverted,
                                kBackLeftSteerMotorId,
                                kBackLeftSteerInverted,
                                kBackLeftSteerEncoderId,
                                kBackLeftAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                        SwerveModuleIOCTRE(
                            kBackRightModuleName,
                            SwerveModuleConfigParams(
                                kBackRightDriveMotorId,
                                kBackRightDriveInverted,
                                kBackRightSteerMotorId,
                                kBackRightSteerInverted,
                                kBackRightSteerEncoderId,
                                kBackRightAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                    ),
                )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    [
                        VisionSubsystemIOLimelight(
                            "limelight-br",
                            kRobotToCamera1Transform,
                            RobotState.getRotation,
                        ),
                        VisionSubsystemIOLimelight(
                            "limelight-fl",
                            kRobotToCamera2Transform,
                            RobotState.getRotation,
                        ),
                    ],
                )

            case RobotModes.SIMULATION:
                self.drive = DriveSubsystem(
                    DriveIO(),
                    (
                        SwerveModuleIOSim(
                            kFrontLeftModuleName,
                            SwerveModuleConfigParams(
                                kFrontLeftDriveMotorId,
                                kFrontLeftDriveInverted,
                                kFrontLeftSteerMotorId,
                                kFrontLeftSteerInverted,
                                kFrontLeftSteerEncoderId,
                                kFrontLeftAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                        SwerveModuleIOSim(
                            kFrontRightModuleName,
                            SwerveModuleConfigParams(
                                kFrontRightDriveMotorId,
                                kFrontRightDriveInverted,
                                kFrontRightSteerMotorId,
                                kFrontRightSteerInverted,
                                kFrontRightSteerEncoderId,
                                kFrontRightAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                        SwerveModuleIOSim(
                            kBackLeftModuleName,
                            SwerveModuleConfigParams(
                                kBackLeftDriveMotorId,
                                kBackLeftDriveInverted,
                                kBackLeftSteerMotorId,
                                kBackLeftSteerInverted,
                                kBackLeftSteerEncoderId,
                                kBackLeftAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                        SwerveModuleIOSim(
                            kBackRightModuleName,
                            SwerveModuleConfigParams(
                                kBackRightDriveMotorId,
                                kBackRightDriveInverted,
                                kBackRightSteerMotorId,
                                kBackRightSteerInverted,
                                kBackRightSteerEncoderId,
                                kBackRightAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                    ),
                )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    [
                        VisionSubsystemIOPhotonSim(
                            "camera-br",
                            kRobotToCamera1Transform,
                            # pylint: disable-next=unnecessary-lambda
                            lambda: RobotState.getSimPose(),
                        ),
                        VisionSubsystemIOPhotonSim(
                            "camera-fl",
                            kRobotToCamera2Transform,
                            # pylint: disable-next=unnecessary-lambda
                            lambda: RobotState.getSimPose(),
                        ),
                    ],
                )

            case _:
                self.drive = DriveSubsystem(
                    DriveIO(),
                    (
                        SwerveModuleIO("Front Left"),
                        SwerveModuleIO("Front Right"),
                        SwerveModuleIO("Back Left"),
                        SwerveModuleIO("Back Right"),
                    ),
                )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    [VisionSubsystemIO(), VisionSubsystemIO()],
                )

        # Alerts
        AlertLogger.registerGroup("Alerts")
        self.driverDisconnected = wpilib.Alert(
            "Driver controller disconnected (port 0)", wpilib.Alert.AlertType.kWarning
        )
        self.operatorDisconnected = wpilib.Alert(
            "Operator controller disconnected (port 1)", wpilib.Alert.AlertType.kWarning
        )
        self.deadInTheWaterAlert = wpilib.Alert(
            "No auto selected!!!", wpilib.Alert.AlertType.kWarning
        )

        # Autonomous routines

        self.nothingAuto = commands2.WaitCommand(kAutoDuration)

        # Chooser
        self.chooser: LoggedDashboardChooser[commands2.Command] = (
            LoggedDashboardChooser("Autonomous")
        )

        pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        for file in os.listdir(pathsPath):
            relevantName = file.split(".")[0]
            auton = PathPlannerAuto(relevantName)
            self.chooser.addOption(relevantName, auton)

        self.chooser.setDefaultOption("Do Nothing Auto", self.nothingAuto)

        def changeStart(newAuto: commands2.Command):
            if isinstance(newAuto, PathPlannerAuto):
                # pylint: disable-next=protected-access
                startingLocation = FlipUtil.fieldPose(newAuto._startingPose)
                RobotState.setAutonomousStartingLogation(startingLocation)

        self.chooser.onChange(changeStart)

        # Put the chooser on the dashboard
        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            FieldRelativeDrive(
                self.drive,
                lambda: OperatorInterface.Drive.ChassisControls.Translation.y()
                * kTurboSpeedMultiplier,
                lambda: OperatorInterface.Drive.ChassisControls.Translation.x()
                * kTurboSpeedMultiplier,
                OperatorInterface.Drive.ChassisControls.Rotation.x,
            )
        )

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def robotPeriodic(self) -> None:
        RobotState.periodic(
            self.drive.getRawRotation(),
            wpilib.RobotController.getFPGATime() / 1e6,
            self.drive.getAngularVelocity(),
            self.drive.getFieldRelativeSpeeds(),
            self.drive.getModulePositions(),
        )
        self.updateAlerts()
        Logger.recordOutput("Component Poses", RobotMechanism.getPoses())

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        OperatorInterface.Drive.align_angle().whileTrue(
            AngleAlignDrive(
                self.drive,
                lambda: OperatorInterface.Drive.ChassisControls.Translation.y()
                * kNormalSpeedMultiplier,
                lambda: OperatorInterface.Drive.ChassisControls.Translation.x()
                * kNormalSpeedMultiplier,
            ).repeatedly()
        )

        OperatorInterface.Drive.reset_gyro().onTrue(
            ResetGyro(self.drive, Pose2d(0, 0, 0)).andThen(
                OperatorInterface.rumbleControllers().withTimeout(0.5)
            )
        )

        OperatorInterface.Drive.defense_state().whileTrue(DefenseState(self.drive))

    def updateAlerts(self):
        self.driverDisconnected.set(not wpilib.DriverStation.isJoystickConnected(0))
        self.operatorDisconnected.set(not wpilib.DriverStation.isJoystickConnected(1))
        self.deadInTheWaterAlert.set(self.chooser.getSelected() == self.nothingAuto)

    def getAutonomousCommand(self) -> commands2.Command:
        selected = self.chooser.getSelected()
        if selected is None:
            return self.nothingAuto
        assert isinstance(selected, commands2.Command)
        return selected
