import os
from commands2.waitcommand import WaitCommand
import wpilib
from wpimath.geometry import Pose2d
import commands2
from pathplannerlib.auto import PathPlannerAuto, NamedCommands
from commands.drive.absoluterelativedrive import AbsoluteRelativeDrive
from commands.drive.anglealign import AngleAlignDrive
from commands.resetdrive import ResetDrive
from commands.drivedistance import DriveDistance
from commands.defensestate import DefenseState

# from commands.drive.drivewaypoint import DriveWaypoint
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.loggingsubsystem import LoggingSubsystem
from subsystems.vision.visionsubsystem import VisionSubsystem

from operatorinterface import OperatorInterface

import constants


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.drive = DriveSubsystem()
        self.vision = VisionSubsystem(self.drive)
        self.log = LoggingSubsystem()

        # Robot demo subsystems
        # self.velocity = VelocityControl()

        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = commands2.SequentialCommandGroup(
            ResetDrive(self.drive),
            DriveDistance(
                -4 * constants.kWheelCircumference,
                0.2,
                DriveDistance.Axis.X,
                self.drive,
            ),
        )
        self.nothingAuto = commands2.WaitCommand(constants.kAutoDuration)

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        NamedCommands.registerCommand("exampleWait", WaitCommand(2))

        pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        for file in os.listdir(pathsPath):
            relevantName = file.split(".")[0]
            auton = PathPlannerAuto(relevantName)
            wpilib.SmartDashboard.putData(f"autos/{relevantName}", auton)
            self.chooser.addOption(relevantName, auton)

        self.chooser.addOption("Do Nothing Auto", self.nothingAuto)
        self.chooser.setDefaultOption("Simple Auto", self.simpleAuto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            AbsoluteRelativeDrive(
                self.drive,
                lambda: OperatorInterface.Drive.ChassisControls.Translation.y()
                * constants.kTurboSpeedMultiplier,
                lambda: OperatorInterface.Drive.ChassisControls.Translation.x()
                * constants.kTurboSpeedMultiplier,
                OperatorInterface.Drive.ChassisControls.Rotation.x,
                OperatorInterface.Drive.ChassisControls.Rotation.y,
            )
        )

        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        OperatorInterface.Drive.align_angle().whileTrue(
            AngleAlignDrive(
                self.drive,
                lambda: OperatorInterface.Drive.ChassisControls.Translation.y()
                * constants.kNormalSpeedMultiplier,
                lambda: OperatorInterface.Drive.ChassisControls.Translation.x()
                * constants.kNormalSpeedMultiplier,
            ).repeatedly()
        )

        OperatorInterface.Drive.reset_gyro().onTrue(
            ResetDrive(self.drive, Pose2d(0, 0, 0))
        )

        OperatorInterface.Drive.defense_state().whileTrue(DefenseState(self.drive))

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
