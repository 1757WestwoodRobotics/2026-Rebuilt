from functools import partial
import os

from commands2.waitcommand import WaitCommand
import wpilib
from wpimath.geometry import Pose2d
import commands2
import commands2.cmd as Commands
from pathplannerlib.auto import PathPlannerAuto, NamedCommands

from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from commands.compositecommands import CompositeCommands
from commands.drive.fieldrelativedrive import FieldRelativeDrive
from commands.drive.anglealign import AngleAlignDrive
from commands.defensestate import DefenseState

from commands.hoodcommands import HoodCommands
from commands.indexercommands import IndexerCommands
from commands.intakecommands import IntakeCommands
from commands.resetgyro import ResetGyro
from commands.shootercommands import ShooterCommands
from commands.turretcommands import TurretCommands
from robotstate import RobotState
from subsystems.drive.driveiopigeon import DriveIOPigeon
from subsystems.drive.drivesubsystem import DriveSubsystem
from subsystems.drive.swervemoduleio import SwerveModuleConfigParams, SwerveModuleIO
from subsystems.drive.swervemoduleiosim import SwerveModuleIOSim
from subsystems.drive.swervemoduleiotalonfx import SwerveModuleIOCTRE
from subsystems.hood.hoodsubsystem import HoodSubsystem
from subsystems.hood.hoodsubsystemio import HoodSubsystemIO
from subsystems.hood.hoodsubsystemiosim import HoodSubsystemIOSim
from subsystems.hood.hoodsubsystemiotalon import HoodSubsystemIOTalon
from subsystems.indexer.indexersubsystem import (
    IndexerSubsystem,
    IndexerSubsystemTarget,
)
from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from subsystems.indexer.indexersubsystemiosim import IndexerSubsystemIOSIM
from subsystems.indexer.indexersubsystemiotalon import IndexerSubsystemIOTalon
from subsystems.intake.intakesubsystem import IntakeSubsystem, IntakeSubsystemGoal
from subsystems.intake.intakesubsystemio import IntakeSubsystemIO
from subsystems.intake.intakesubsystemiosim import IntakeSubsystemIOSIM
from subsystems.intake.intakesubsystemiotalon import IntakeSubsystemIOTalon
from subsystems.shooter.shootersubsystem import ShooterSubsystem
from subsystems.shooter.shootersubsystemio import ShooterSubsystemIO
from subsystems.shooter.shootersubsystemiosim import ShooterSubsystemIOSim
from subsystems.shooter.shootersubsystemiotalon import ShooterSubsystemIOTalon
from subsystems.turret.turretsubsystem import TurretSubsystem
from subsystems.turret.turretsubsystemio import TurretSubsystemIO
from subsystems.turret.turretsubsystemiosim import TurretSubsystemIOSim
from subsystems.turret.turretsubsystemiotalon import TurretSubsystemIOTalon
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniolimelight import VisionSubsystemIOLimelight
from subsystems.vision.visioniosim import VisionSubsystemIOSim
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
                wpilib.DataLogManager.log("We have drive")
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
                self.intake = IntakeSubsystem(IntakeSubsystemIOTalon())
                self.indexer = IndexerSubsystem(IndexerSubsystemIOTalon())
                self.shooter = ShooterSubsystem(ShooterSubsystemIOTalon())
                self.hood = HoodSubsystem(HoodSubsystemIOTalon())
                self.turret = TurretSubsystem(TurretSubsystemIOTalon())

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
                        VisionSubsystemIOSim(
                            "camera-br",
                            kRobotToCamera1Transform,
                            # pylint: disable-next=unnecessary-lambda
                            lambda: RobotState.getSimPose(),
                        ),
                        VisionSubsystemIOSim(
                            "camera-fl",
                            kRobotToCamera2Transform,
                            # pylint: disable-next=unnecessary-lambda
                            lambda: RobotState.getSimPose(),
                        ),
                    ],
                )
                self.intake = IntakeSubsystem(IntakeSubsystemIOSIM())
                self.indexer = IndexerSubsystem(IndexerSubsystemIOSIM())
                self.shooter = ShooterSubsystem(ShooterSubsystemIOSim())
                self.hood = HoodSubsystem(HoodSubsystemIOSim())
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
                    [VisionSubsystemIO(), VisionSubsystemIO()],
                )
                self.intake = IntakeSubsystem(IntakeSubsystemIO())
                self.indexer = IndexerSubsystem(IndexerSubsystemIO())
                self.shooter = ShooterSubsystem(ShooterSubsystemIO())
                self.hood = HoodSubsystem(HoodSubsystemIO())
                self.turret = TurretSubsystem(TurretSubsystemIO())

        # Autonomous routines

        self.nothingAuto = commands2.WaitCommand(kAutoDuration)

        # Chooser
        self.chooser: LoggedDashboardChooser[commands2.Command] = (
            LoggedDashboardChooser("Autonomous")
        )
        self.chooser.addOption("Intake SysId", self.intake.sysIdRoutine(self.intake))
        self.chooser.addOption("Shooter SysId", self.shooter.sysIdRoutine(self.shooter))
        self.chooser.addOption(
            "Shooter Test run",
            Commands.runOnce(lambda: self.shooter.setClosedLoop(False), self.shooter)
            .andThen(
                Commands.run(lambda: self.shooter.setShooterRawVolts(6.0), self.shooter)
            )
            .andThen(
                Commands.runOnce(lambda: self.shooter.setClosedLoop(True), self.shooter)
            ),
        )

        # Add commands to the autonomous command chooser
        NamedCommands.registerCommand("exampleWait", WaitCommand(2))

        pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        for file in os.listdir(pathsPath):
            relevantName = file.split(".")[0]
            auton = PathPlannerAuto(relevantName)
            self.chooser.addOption(relevantName, auton)

        self.chooser.setDefaultOption("Do Nothing Auto", self.nothingAuto)

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
        self.intake.setDefaultCommand(IntakeCommands.retractIntake(self.intake))
        self.indexer.setDefaultCommand(IndexerCommands.holdIndexer(self.indexer))
        self.shooter.setDefaultCommand(ShooterCommands.flywheelTracking(self.shooter))
        self.hood.setDefaultCommand(HoodCommands.hoodTracking(self.hood))
        self.turret.setDefaultCommand(TurretCommands.trackedTurret(self.turret))

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def robotPeriodic(self) -> None:
        RobotState.periodic(
            self.drive.getRawRotation(),
            wpilib.RobotController.getFPGATime(),
            self.drive.getAngularVelocity(),
            self.drive.getFieldRelativeSpeeds(),
            self.drive.getModulePositions(),
        )

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
            ResetGyro(self.drive, Pose2d(0, 0, 0))
        )

        OperatorInterface.Drive.defense_state().whileTrue(DefenseState(self.drive))

        OperatorInterface.intake().onTrue(IntakeCommands.deployIntake(self.intake))

        (OperatorInterface.intake() and OperatorInterface.eject()).whileTrue(
            CompositeCommands.ejectBall(self.intake, self.indexer)
        )

        OperatorInterface.feed().whileTrue(IndexerCommands.feedIndexer(self.indexer))

    def getAutonomousCommand(self) -> commands2.Command:
        selected = self.chooser.getSelected()
        if selected is None:
            return self.nothingAuto
        assert isinstance(selected, commands2.Command)
        return selected
