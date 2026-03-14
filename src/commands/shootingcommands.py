from commands2 import Command, cmd
from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from robotstate import RobotState
from subsystems.drive.drivesubsystem import DriveSubsystem
from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem
from subsystems.hood.hoodsubsystem import HoodSubsystem
from subsystems.indexer.indexersubsystem import IndexerSubsystem, IndexerSubsystemGoal
from subsystems.intake.intakesubsystem import IntakeSubsystem

import commands.indexercommands as IndexerCommands  # module, not class
import commands.flywheelcommands as FlywheelCommands
import commands.intakecommands as IntakeCommands
import commands.hoodcommands as HoodCommands

from constants.trajectory import kRotationPGain, kRotationIGain, kRotationDGain
from constants.hood import kHoodMaxAngle
from constants.shooting import kShootingMap, kHoodAngleMap
from constants.turret import kTurretLocation, kTurretTolerance

from subsystems.turret.turretsubsystem import TurretSubsystem
from util.angleoptimize import optimizeAngle
from util.controltype import AnalogInput
from util.convenientmath import pose3dFrom2d


def shootBalls(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to shoot balls when the flywheel is at speed and the hood is at angle
    """
    return cmd.parallel(
        FlywheelCommands.shootWithDistance(flywheel, RobotState.distanceToHub),
        HoodCommands.angleHoodWithDistance(hood, RobotState.distanceToHub),
        # wait until the flywheel is at speed and the hood is at angle before kicking the indexer
        # to shoot, then keep kicking regardless if we are out of tolerance
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            IndexerCommands.kickIndexer(indexer)
        ),
    )


def shootWithOscillation(
    indexer: IndexerSubsystem,
    hood: HoodSubsystem,
    flywheel: FlywheelSubsystem,
    intake: IntakeSubsystem,
) -> Command:
    return cmd.parallel(
        FlywheelCommands.shootWithDistance(flywheel, RobotState.distanceToHub),
        HoodCommands.angleHoodWithDistance(hood, RobotState.distanceToHub),
        # wait until the flywheel is at speed and the hood is at angle before kicking the indexer
        # to shoot, then keep kicking regardless if we are out of tolerance
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            cmd.parallel(
                IndexerCommands.kickIndexer(indexer),
                IntakeCommands.oscillateIntake(intake),
            )
        ),
    )


def feedBalls(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to feed balls into the flywheel when the flywheel is at a lower speed and the hood
    is at a more open angle, for use when feeding from the floor or a low station
    """
    return cmd.parallel(
        FlywheelCommands.feedWithDistance(flywheel, RobotState.distanceToObjective),
        HoodCommands.angleHood(hood, lambda: kHoodMaxAngle),
        # wait until the flywheel is at speed and the hood is at angle before kicking the indexer
        # to shoot, then keep kicking regardless if we are out of tolerance
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            IndexerCommands.kickIndexer(indexer)
        ),
    )


def shootBasedOnMode(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to shoot or feed balls based on the current objective
    """
    return cmd.either(
        shootBalls(indexer, hood, flywheel),
        feedBalls(indexer, hood, flywheel),
        lambda: RobotState.objective == RobotState.RobotMetaObjective.SHOOT,
    )


def shootBasedOnModeWithOscillation(
    indexer: IndexerSubsystem,
    hood: HoodSubsystem,
    flywheel: FlywheelSubsystem,
    intake: IntakeSubsystem,
) -> Command:
    """
    Command to shoot or feed balls based on the current objective
    """
    return cmd.either(
        shootWithOscillation(indexer, hood, flywheel, intake),
        feedBalls(indexer, hood, flywheel),
        lambda: RobotState.objective == RobotState.RobotMetaObjective.SHOOT,
    )


def setFeedObjective() -> Command:
    def _set():
        RobotState.objective = RobotState.RobotMetaObjective.FEED

    return cmd.runOnce(_set).withName("FeedObjective")


def setShootObjective() -> Command:
    def _set():
        RobotState.objective = RobotState.RobotMetaObjective.SHOOT

    return cmd.runOnce(_set).withName("ShootObjective")


class TurretFixedDriveShoot(Command):
    def __init__(
        self,
        flywheel: FlywheelSubsystem,
        hood: HoodSubsystem,
        turret: TurretSubsystem,
        indexer: IndexerSubsystem,
        drive: DriveSubsystem,
        forward: AnalogInput,
        sideways: AnalogInput,
    ):
        Command.__init__(self)
        self.setName(type(self).__name__)

        self.lockedAngle: Rotation2d = Rotation2d()

        self.indexer = indexer
        self.flywheel = flywheel
        self.hood = hood
        self.turret = turret
        self.drive = drive

        self.forward = forward
        self.sideways = sideways

        self.rotationPID = PIDController(kRotationPGain, kRotationIGain, kRotationDGain)
        self.rotationPID.setTolerance(kTurretTolerance.radians())

        self.addRequirements(
            self.indexer, self.flywheel, self.hood, self.turret, self.drive
        )

    def initialize(self):
        self.lockedAngle = RobotState.turretRotation

    def execute(self):
        self.turret.setTurretGoal(self.lockedAngle)
        objectiveDistance = RobotState.distanceToObjective()
        objectiveLocation = RobotState.objectiveLocation()
        robotLocation = (
            RobotState.getFieldPose()
            if RobotState.objective == RobotState.RobotMetaObjective.FEED
            else RobotState.getHubPose()
        )
        turretLocation = (pose3dFrom2d(robotLocation) + kTurretLocation).toPose2d()
        self.flywheel.setGoal(float(kShootingMap(objectiveDistance)))
        self.hood.setHoodGoal(kHoodAngleMap(objectiveDistance))

        objectiveRelativeToRobot = objectiveLocation - turretLocation.translation()

        fieldRelativeAngle = objectiveRelativeToRobot.angle()
        driveAngle = fieldRelativeAngle - self.lockedAngle

        optimizedDirection = optimizeAngle(
            RobotState.getRotation(), driveAngle
        ).radians()
        rotation = self.rotationPID.calculate(
            RobotState.getRotation().radians(), optimizedDirection
        )

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            # if we're on the other side, switch the controls around
            self.drive.arcadeDriveWithFactors(
                -self.forward(),
                -self.sideways(),
                rotation,
                DriveSubsystem.CoordinateMode.FieldRelative,
            )
        else:
            self.drive.arcadeDriveWithFactors(
                self.forward(),
                self.sideways(),
                rotation,
                DriveSubsystem.CoordinateMode.FieldRelative,
            )

        if (
            self.hood.atTarget
            and self.turret.atTarget
            and self.rotationPID.atSetpoint()
        ):
            self.indexer.setTarget(IndexerSubsystemGoal.KICK)
