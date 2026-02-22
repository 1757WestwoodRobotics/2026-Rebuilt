import os

from pykit.logger import Logger
from pykit.alertlogger import AlertLogger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser

import wpilib
from wpimath.geometry import Pose2d, Rotation2d
import commands2
import commands2.cmd as Commands
from pathplannerlib.auto import PathPlannerAuto

from commands.drive.fieldrelativeassisteddrive import FieldRelativeAssistedDrive
from commands.drive.anglealign import AngleAlignDrive
from commands.defensestate import DefenseState
import commands.turretcommands as TurretCommands  # module, not class

from commands.resetgyro import ResetGyro
from robotmechanism import RobotMechanism
from robotstate import RobotState
from subsystems.drive.driveiopigeon import DriveIOPigeon
from subsystems.drive.drivesubsystem import DriveSubsystem
from subsystems.drive.swervemoduleio import SwerveModuleConfigParams, SwerveModuleIO
from subsystems.drive.swervemoduleiosim import SwerveModuleIOSim
from subsystems.drive.swervemoduleiotalonfx import SwerveModuleIOCTRE
from subsystems.turret.turretsubsystem import TurretSubsystem
from subsystems.turret.turretsubsystemio import TurretSubsystemIO
from subsystems.turret.turretsubsystemiosim import TurretSubsystemIOSim
from subsystems.turret.turretsubsystemiotalon import TurretSubsystemIOTalon
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniolimelight import VisionSubsystemIOLimelight
from subsystems.vision.visionsubsystem import VisionSubsystem
from subsystems.drive.driveio import DriveIO

from operatorinterface import OperatorInterface

from constants.vision import (
    kRobotToCamera1Transform,
    kRobotToCamera2Transform,
    kTurretToCameraTransform,
)
from constants.field import kAutoDuration
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
    kDriveGearingRatio,
    kSteerGearingRatioMk5i,
    kSteerGearingRatioMk5n,
)
from constants import RobotModes, kRobotMode
from util.fliputil import FlipUtil
from util.logtunablenumber import AutoUpdateGroup, LoggedTunableNumber

if kRobotMode == RobotModes.SIMULATION: # required since opencv can't go on rio
    from subsystems.vision.visioniophotonsim import VisionSubsystemIOPhotonSim

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
                self.drive = DriveSubsystem(
                    DriveIOPigeon(),
                    (
                        SwerveModuleIOCTRE(
                            kFrontLeftModuleName,
                            SwerveModuleConfigParams(
                                kFrontLeftDriveMotorId,
                                kFrontLeftDriveInverted,
                                kDriveGearingRatio,
                                kFrontLeftSteerMotorId,
                                kFrontLeftSteerInverted,
                                kSteerGearingRatioMk5i,
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
                                kDriveGearingRatio,
                                kFrontRightSteerMotorId,
                                kFrontRightSteerInverted,
                                kSteerGearingRatioMk5i,
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
                                kDriveGearingRatio,
                                kBackLeftSteerMotorId,
                                kBackLeftSteerInverted,
                                kSteerGearingRatioMk5i,
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
                                kDriveGearingRatio,
                                kBackRightSteerMotorId,
                                kBackRightSteerInverted,
                                kSteerGearingRatioMk5i,
                                kBackRightSteerEncoderId,
                                kBackRightAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                    ),
                )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    RobotState.addTurretedVisionMeasurement,
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
                self.turret = TurretSubsystem(TurretSubsystemIO())

            case RobotModes.SIMULATION:
                self.drive = DriveSubsystem(
                    DriveIO(),
                    (
                        SwerveModuleIOSim(
                            kFrontLeftModuleName,
                            SwerveModuleConfigParams(
                                kFrontLeftDriveMotorId,
                                kFrontLeftDriveInverted,
                                kDriveGearingRatio,
                                kFrontLeftSteerMotorId,
                                kFrontLeftSteerInverted,
                                kSteerGearingRatioMk5i,
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
                                kDriveGearingRatio,
                                kFrontRightSteerMotorId,
                                kFrontRightSteerInverted,
                                kSteerGearingRatioMk5i,
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
                                kDriveGearingRatio,
                                kBackLeftSteerMotorId,
                                kBackLeftSteerInverted,
                                kSteerGearingRatioMk5n,
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
                                kDriveGearingRatio,
                                kBackRightSteerMotorId,
                                kBackRightSteerInverted,
                                kSteerGearingRatioMk5n,
                                kBackRightSteerEncoderId,
                                kBackRightAbsoluteEncoderOffset,
                                kCANivoreName,
                            ),
                        ),
                    ),
                )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    RobotState.addTurretedVisionMeasurement,
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
                        VisionSubsystemIOPhotonSim(
                            "camera-turret",
                            kTurretToCameraTransform,
                            RobotState.getSimTurretPose,
                            True,
                        ),
                    ],
                )
                self.turret = TurretSubsystem(TurretSubsystemIOSim())

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
                    RobotState.addTurretedVisionMeasurement,
                    [VisionSubsystemIO(), VisionSubsystemIO()],
                )
                self.turret = TurretSubsystem(TurretSubsystemIO())

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

        self.shiftActiveAlert = wpilib.Alert(
            "SHIFT ACTIVE!", wpilib.Alert.AlertType.kInfo
        )
        self.shiftActiveAlert.set(True)

        self.usbAlert = wpilib.Alert(
            "No USB Drive in robot!", wpilib.Alert.AlertType.kError
        )
        if wpilib.RobotBase.isReal() and not os.path.exists("/U/logs"):
            self.usbAlert.set(True)

        # Initialize as active at startup;
        # this initial value may be updated later based on the actual shift state

        # Autonomous routines

        self.nothingAuto = commands2.WaitCommand(kAutoDuration)

        # Chooser
        self.chooser: LoggedDashboardChooser[commands2.Command] = (
            LoggedDashboardChooser("Autonomous")
        )
        self.chooser.addOption("Turret SysID", self.turret.sysIdRoutine(self.turret))

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
        self.oi = OperatorInterface()
        self.configureButtonBindings()

        self.turret.setDefaultCommand(TurretCommands.trackedTurret(self.turret))

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def robotPeriodic(self) -> None:
        RobotState.periodic(
            self.drive.getRawRotation(),
            wpilib.RobotController.getFPGATime() / 1e6,
            self.drive.getAngularVelocity(),
            self.drive.getFieldRelativeSpeeds(),
            self.drive.getModulePositions(),
            Rotation2d(
                wpilib.Timer.getTimestamp() / 20
            ),  # Simulated turret rotation, just go spin
        )
        LoggedTunableNumber.updateAll()
        AutoUpdateGroup.updateAll()
        self.updateAlerts()
        Logger.recordOutput("Component Poses", RobotMechanism.getPoses())

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        self.drive.setDefaultCommand(
            FieldRelativeAssistedDrive(
                self.drive,
                lambda: self.oi.driverY() * kTurboSpeedMultiplier,
                lambda: self.oi.driverX() * kTurboSpeedMultiplier,
                self.oi.driverRotation,
            )
        )
        self.oi.driverController.rightBumper().whileTrue(
            AngleAlignDrive(
                self.drive,
                lambda: self.oi.driverY() * kNormalSpeedMultiplier,
                lambda: self.oi.driverX() * kNormalSpeedMultiplier,
            ).repeatedly()
        )

        self.oi.driverController.povDown().onTrue(
            ResetGyro(self.drive, Pose2d(0, 0, 0)).andThen(
                self.oi.rumbleControllersCommand().withTimeout(0.5)
            )
        )

        self.oi.driverController.x().whileTrue(DefenseState(self.drive))

        RobotState.shiftTrigger().onTrue(
            Commands.runOnce(lambda: self.shiftActiveAlert.set(True))
        ).onFalse(Commands.runOnce(lambda: self.shiftActiveAlert.set(False)))

    def updateAlerts(self):
        self.driverDisconnected.set(
            not wpilib.DriverStation.isJoystickConnected(
                self.oi.driverController.getHID().getPort()
            )
        )
        self.operatorDisconnected.set(
            not wpilib.DriverStation.isJoystickConnected(
                self.oi.operatorController.getHID().getPort()
            )
        )
        self.deadInTheWaterAlert.set(self.chooser.getSelected() == self.nothingAuto)

    def getAutonomousCommand(self) -> commands2.Command:
        selected = self.chooser.getSelected()
        if selected is None:
            return self.nothingAuto
        assert isinstance(selected, commands2.Command)
        return selected
