import os

from pykit.logger import Logger
from pykit.alertlogger import AlertLogger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser

import wpilib
from wpimath.geometry import Pose2d
import commands2
import commands2.cmd as Commands
from pathplannerlib.auto import PathPlannerAuto

from commands.drive.fieldrelativeassisteddrive import FieldRelativeAssistedDrive
from commands.drive.anglealign import AngleAlignDrive
from commands.defensestate import DefenseState
import commands.intakecommands as IntakeCommands
import commands.turretcommands as TurretCommands  # module, not class
import commands.climbcommands as ClimbCommands  # module, not class
import commands.indexercommands as IndexerCommands  # module, not class
import commands.hoodcommands as HoodCommands
import commands.overridecommands as OverrideCommands
import commands.shootingcommands as ShootingCommands

from commands.resetgyro import ResetGyro
from preflight import PreflightChecklist
from robotmechanism import RobotMechanism
from robotstate import RobotState
from subsystems.climber.climbersubsystem import ClimberSubsystem
from subsystems.climber.climbersubsystemio import ClimberSubsystemIO
from subsystems.climber.climbersubsystemiosim import ClimberSubsystemIOSim
from subsystems.climber.climbersubsystemiotalon import ClimberSubsystemIOTalon
from subsystems.drive.driveiopigeon import DriveIOPigeon
from subsystems.drive.drivesubsystem import DriveSubsystem
from subsystems.drive.swervemoduleio import SwerveModuleConfigParams, SwerveModuleIO
from subsystems.drive.swervemoduleiosim import SwerveModuleIOSim
from subsystems.drive.swervemoduleiotalonfx import SwerveModuleIOCTRE
from subsystems.intake.intakesubsystem import IntakeSubsystem
from subsystems.intake.intakesubsystemio import IntakeSubsystemIO
from subsystems.intake.intakesubsystemiosim import IntakeSubsystemIOSim
from subsystems.intake.intakesubsystemiotalon import IntakeSubsystemIOTalon
from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem
from subsystems.flywheel.flywheelsubsystemio import FlywheelSubsystemIO
from subsystems.flywheel.flywheelsubsystemiosim import FlywheelSubsystemIOSim
from subsystems.flywheel.flywheelsubsystemiotalon import FlywheelSubsystemIOTalon
from subsystems.leds.ledsubsystem import LEDSubsystem
from subsystems.turret.turretsubsystem import TurretSubsystem
from subsystems.turret.turretsubsystemio import TurretSubsystemIO
from subsystems.turret.turretsubsystemiotalon import TurretSubsystemIOTalon
from subsystems.turret.turretsubsystemiosim import TurretSubsystemIOSim
from subsystems.indexer.indexersubsystem import IndexerSubsystem
from subsystems.indexer.indexersubsystemio import IndexerSubsystemIO
from subsystems.indexer.indexersubsystemiosim import IndexerSubsystemIOSIM
from subsystems.indexer.indexersubsystemiotalon import IndexerSubsystemIOTalon
from subsystems.hood.hoodsubsystem import HoodSubsystem
from subsystems.hood.hoodsubsystemio import HoodSubsystemIO
from subsystems.hood.hoodsubsystemiosim import HoodSubsystemIOSim
from subsystems.hood.hoodsubsystemiotalon import HoodSubsystemIOTalon
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniophoton import VisionSubsystemIOPhotonVision
from subsystems.vision.visionsubsystem import VisionSubsystem
from subsystems.drive.driveio import DriveIO

from operatorinterface import OperatorInterface

from constants.vision import (
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
    kCANivoreCANBus,
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
)
from constants import RobotModes, kRobotMode
from util.fliputil import FlipUtil
from util.helpfultriggerwrappers import NetworkTableButton
from util.logtunablenumber import AutoUpdateGroup, LoggedTunableNumber

if kRobotMode == RobotModes.SIMULATION:  # required since opencv can't go on rio
    # pylint:disable-next=ungrouped-imports
    from subsystems.vision.visioniophotonsim import VisionSubsystemIOPhotonSim


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    # pylint: disable-next=too-many-statements
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
                                kCANivoreCANBus,
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
                                kCANivoreCANBus,
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
                                kCANivoreCANBus,
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
                                kCANivoreCANBus,
                            ),
                        ),
                    ),
                )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    RobotState.addTurretedVisionMeasurement,
                    [
                        VisionSubsystemIOPhotonVision(
                            "TURRET_CAM", kTurretToCameraTransform, True
                        )
                    ],
                )
                self.turret = TurretSubsystem(TurretSubsystemIOTalon())
                self.climber = ClimberSubsystem(ClimberSubsystemIOTalon())
                self.intake = IntakeSubsystem(IntakeSubsystemIOTalon())
                self.indexer = IndexerSubsystem(IndexerSubsystemIOTalon())
                self.flywheel = FlywheelSubsystem(FlywheelSubsystemIOTalon())
                self.hood = HoodSubsystem(HoodSubsystemIOTalon())
                self.leds = LEDSubsystem()

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
                                kCANivoreCANBus,
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
                                kCANivoreCANBus,
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
                                kSteerGearingRatioMk5i,
                                kBackLeftSteerEncoderId,
                                kBackLeftAbsoluteEncoderOffset,
                                kCANivoreCANBus,
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
                                kSteerGearingRatioMk5i,
                                kBackRightSteerEncoderId,
                                kBackRightAbsoluteEncoderOffset,
                                kCANivoreCANBus,
                            ),
                        ),
                    ),
                )
                self.vision = VisionSubsystem(
                    RobotState.addVisionMeasurement,
                    RobotState.addTurretedVisionMeasurement,
                    [
                        # pylint: disable-next=possibly-used-before-assignment
                        VisionSubsystemIOPhotonSim(
                            "TURRET_CAM",
                            kTurretToCameraTransform,
                            RobotState.getSimTurretPose,
                            True,
                        ),
                    ],
                )
                self.turret = TurretSubsystem(TurretSubsystemIOSim())
                self.indexer = IndexerSubsystem(IndexerSubsystemIOSIM())
                self.climber = ClimberSubsystem(ClimberSubsystemIOSim())
                self.intake = IntakeSubsystem(IntakeSubsystemIOSim())
                self.flywheel = FlywheelSubsystem(FlywheelSubsystemIOSim())
                self.hood = HoodSubsystem(HoodSubsystemIOSim())

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
                    [VisionSubsystemIO()],
                )
                self.turret = TurretSubsystem(TurretSubsystemIO())
                self.indexer = IndexerSubsystem(IndexerSubsystemIO())
                self.climber = ClimberSubsystem(ClimberSubsystemIO())
                self.intake = IntakeSubsystem(IntakeSubsystemIO())
                self.flywheel = FlywheelSubsystem(FlywheelSubsystemIO())
                self.hood = HoodSubsystem(HoodSubsystemIO())

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
        self.preflightAlert = wpilib.Alert(
            "preflight checking not complete", wpilib.Alert.AlertType.kError
        )

        # preflight checklist
        AlertLogger.registerGroup("preflight")
        self.preflight = PreflightChecklist()

        # Initialize as active at startup;
        # this initial value may be updated later based on the actual shift state

        # Autonomous routines

        self.nothingAuto = commands2.WaitCommand(kAutoDuration)

        # Chooser
        self.chooser: LoggedDashboardChooser[commands2.Command] = (
            LoggedDashboardChooser("Autonomous")
        )
        self.chooser.addOption("Turret SysID", self.turret.sysIdRoutine(self.turret))
        self.chooser.addOption("Climb SysID", self.climber.sysIdRoutine(self.climber))
        self.chooser.addOption("Intake SysID", self.intake.sysIdRoutine(self.intake))
        self.chooser.addOption(
            "Flywheel SysID", self.flywheel.sysIdRoutine(self.flywheel)
        )
        self.chooser.addOption("Hood SysID", self.hood.sysIdRoutine(self.hood))

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
                RobotState.setAutonomousStartingLocation(startingLocation)

        self.chooser.onChange(changeStart)

        # Put the chooser on the dashboard
        self.oi = OperatorInterface()
        self.configureButtonBindings()
        self.configureOverrides()

        self.turret.setDefaultCommand(TurretCommands.trackedTurret(self.turret))
        self.indexer.setDefaultCommand(IndexerCommands.holdIndexer(self.indexer))
        self.hood.setDefaultCommand(HoodCommands.moveToMin(self.hood))

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def disabledPeriodic(self) -> None:
        self.preflight.update()

    def robotPeriodic(self) -> None:
        RobotState.periodic(
            self.drive.getRawRotation(),
            wpilib.RobotController.getFPGATime() / 1e6,
            self.drive.getAngularVelocity(),
            self.drive.getFieldRelativeSpeeds(),
            self.drive.getModulePositions(),
            self.turret.position,
            self.intake.position,
        )
        LoggedTunableNumber.updateAll()
        AutoUpdateGroup.updateAll()
        self.updateAlerts()
        Logger.recordOutput(
            "Component Poses",
            RobotMechanism.getPoses(
                self.turret.position,
                self.intake.position,
                self.climber.position,
            ),
        )

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

        # Driver Controller (Xbox) Section
        self.oi.driverController.rightBumper().whileTrue(
            AngleAlignDrive(
                self.drive,
                lambda: self.oi.driverY() * kNormalSpeedMultiplier,
                lambda: self.oi.driverX() * kNormalSpeedMultiplier,
            ).repeatedly()
        )
        self.oi.driverController.rightTrigger().whileTrue(
            ShootingCommands.shootBalls(self.indexer, self.hood, self.flywheel)
        )

        self.oi.driverController.povDown().onTrue(
            ResetGyro(self.drive, Pose2d(0, 0, 0))
        ).onTrue(self.oi.rumbleControllersCommand().withTimeout(0.5))

        self.oi.driverController.x().whileTrue(DefenseState(self.drive))

        self.oi.driverController.leftBumper().onTrue(
            IntakeCommands.toggleIntakeDeployment(self.intake)
        )
        self.oi.driverController.leftTrigger().whileTrue(
            IntakeCommands.runIntakeRollers(self.intake)
        )

        # Operator Controller (Farm Box) Section
        # these buttons come from our strategy spreadsheet,
        # they are magic numbers but all buttons are labeled properly on the controller

        self.oi.operatorController.button(1).onTrue(
            ClimbCommands.deployClimber(self.climber)
        )
        self.oi.operatorController.button(6).onTrue(
            ClimbCommands.retractClimber(self.climber)
        )
        self.oi.operatorController.button(2).whileTrue(
            ClimbCommands.bumpUp(self.climber).repeatedly()
        )
        self.oi.operatorController.button(7).whileTrue(
            ClimbCommands.bumpDown(self.climber).repeatedly()
        )

        self.oi.operatorController.button(3).whileTrue(
            IntakeCommands.reverseIntake(self.intake)
        )
        self.oi.operatorController.button(4).whileTrue(
            IntakeCommands.depotIntake(self.intake)
        )
        self.oi.operatorController.button(15).onTrue(
            IntakeCommands.bumpIntakeUp(self.intake)
        )
        self.oi.operatorController.button(16).onTrue(
            IntakeCommands.bumpIntakeDown(self.intake)
        )

        # indexer related
        self.oi.operatorController.button(5).whileTrue(
            IndexerCommands.kickIndexer(self.indexer)
        )
        self.oi.operatorController.button(10).whileTrue(
            IndexerCommands.ejectIndexer(self.indexer)
        )
        self.oi.operatorController.button(21).whileTrue(
            ShootingCommands.shootBalls(self.indexer, self.hood, self.flywheel)
        )

        # objective related
        self.oi.operatorController.button(17).onTrue(
            ShootingCommands.setFeedObjective()
        )
        self.oi.operatorController.button(18).onTrue(
            ShootingCommands.setShootObjective()
        )

        RobotState.shiftTrigger().onTrue(
            Commands.runOnce(lambda: self.shiftActiveAlert.set(True))
        ).onFalse(Commands.runOnce(lambda: self.shiftActiveAlert.set(False)))

        RobotState.hubAboutToChangeTrigger().onTrue(
            commands2.cmd.sequence(
                self.oi.rumbleControllersCommand(0.5).withTimeout(0.3),
                commands2.cmd.waitSeconds(0.7),
                self.oi.rumbleControllersCommand(0.5).withTimeout(0.3),
                commands2.cmd.waitSeconds(0.7),
                self.oi.rumbleControllersCommand(0.5).withTimeout(0.3),
                commands2.cmd.waitSeconds(0.7),
                self.oi.rumbleControllersCommand(1.0).withTimeout(0.5),
            ).withName("Shift Change Rumble")
        )  # rumble 3 times when about to change

    def configureOverrides(self) -> None:
        """
        Configures commands that are for overrides and manual mode values
        """
        NetworkTableButton("Override/Flywheel").whileTrue(
            OverrideCommands.overrideFlywheel(self.flywheel)
        )
        NetworkTableButton("Override/Hood").whileTrue(
            OverrideCommands.overrideHood(self.hood)
        )

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
        self.preflightAlert.set(not self.preflight.is_complete())

    def getAutonomousCommand(self) -> commands2.Command:
        selected = self.chooser.getSelected()
        if selected is None:
            return self.nothingAuto
        assert isinstance(selected, commands2.Command)
        return selected
