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
from subsystems.turret.turretsubsystem import TurretSubsystem

import commands.indexercommands as IndexerCommands  # module, not class
import commands.flywheelcommands as FlywheelCommands
import commands.intakecommands as IntakeCommands
import commands.hoodcommands as HoodCommands

from constants.trajectory import kRotationPGain, kRotationIGain, kRotationDGain
from constants.hood import kHoodMaxAngle
from constants.shooting import kShootingMap, kHoodAngleMap, kFeedFlywheelMap
from constants.turret import kTurretLocation, kTurretTolerance

from util.angleoptimize import optimizeAngle
from util.controltype import AnalogInput
from util.convenientmath import pose3dFrom2d


def feedIfTurretAligned(indexer: IndexerSubsystem) -> Command:
    """
    Command to feed balls into the flywheel when the turret is aligned, for use when feeding from the floor or a low station
    """

    return cmd.either(
        IndexerCommands.kickIndexer(indexer),
        cmd.none(),
        lambda: RobotState.turretAtAngle,
    ).repeatedly()


def shootBallsStatic(
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
        cmd.waitUntil(RobotState.readyToShoot).andThen(feedIfTurretAligned(indexer)),
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
                feedIfTurretAligned(indexer),
                IntakeCommands.oscillateIntake(intake),
            )
        ),
    )


def feedBallsStatic(
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
        cmd.waitUntil(RobotState.readyToShoot).andThen(feedIfTurretAligned(indexer)),
    )


def shootBasedOnModeStatic(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to shoot or feed balls based on the current objective
    """
    return cmd.either(
        shootBallsStatic(indexer, hood, flywheel),
        feedBallsStatic(indexer, hood, flywheel),
        lambda: RobotState.objective == RobotState.RobotMetaObjective.SHOOT,
    )


def shootBallsMoving(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to shoot balls while moving, with compensation for robot velocity
    """
    return cmd.parallel(
        FlywheelCommands.shootWithDistance(
            flywheel, lambda: RobotState.effectiveObjectiveDistance
        ),
        HoodCommands.angleHoodWithDistance(
            hood, lambda: RobotState.effectiveObjectiveDistance
        ),
        cmd.waitUntil(RobotState.readyToShoot).andThen(feedIfTurretAligned(indexer)),
    )


def shootBallsMovingWithOscillation(
    indexer: IndexerSubsystem,
    hood: HoodSubsystem,
    flywheel: FlywheelSubsystem,
    intake: IntakeSubsystem,
) -> Command:
    """
    Command to shoot balls while moving with oscillation, with compensation for robot velocity
    """
    return cmd.parallel(
        FlywheelCommands.shootWithDistance(
            flywheel, lambda: RobotState.effectiveObjectiveDistance
        ),
        HoodCommands.angleHoodWithDistance(
            hood, lambda: RobotState.effectiveObjectiveDistance
        ),
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            cmd.parallel(
                feedIfTurretAligned(indexer),
                IntakeCommands.oscillateIntake(intake),
            )
        ),
    )


def feedBallsMoving(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to feed balls while moving, with compensation for robot velocity
    """
    return cmd.parallel(
        FlywheelCommands.feedWithDistance(
            flywheel, lambda: RobotState.effectiveObjectiveDistance
        ),
        HoodCommands.angleHood(hood, lambda: kHoodMaxAngle),
        cmd.waitUntil(RobotState.readyToShoot).andThen(feedIfTurretAligned(indexer)),
    )


def feedBallsMovingWithOscillation(
    indexer: IndexerSubsystem,
    hood: HoodSubsystem,
    flywheel: FlywheelSubsystem,
    intake: IntakeSubsystem,
) -> Command:
    """
    Command to feed balls while moving with oscillation, with compensation for robot velocity
    """
    return cmd.parallel(
        FlywheelCommands.feedWithDistance(
            flywheel, lambda: RobotState.effectiveObjectiveDistance
        ),
        HoodCommands.angleHood(hood, lambda: kHoodMaxAngle),
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            cmd.parallel(
                feedIfTurretAligned(indexer),
                IntakeCommands.oscillateIntake(intake),
            )
        ),
    )


def shootBasedOnModeMoving(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to shoot or feed balls while moving based on the current objective,
    with compensation for robot velocity
    """
    return cmd.either(
        shootBallsMoving(indexer, hood, flywheel),
        feedBallsMoving(indexer, hood, flywheel),
        lambda: RobotState.objective == RobotState.RobotMetaObjective.SHOOT,
    )


def shootBasedOnModeMovingWithOscillation(
    indexer: IndexerSubsystem,
    hood: HoodSubsystem,
    flywheel: FlywheelSubsystem,
    intake: IntakeSubsystem,
) -> Command:
    """
    Command to shoot or feed balls while moving with oscillation based on the current objective,
    with compensation for robot velocity
    """
    return cmd.either(
        shootBallsMovingWithOscillation(indexer, hood, flywheel, intake),
        feedBallsMovingWithOscillation(indexer, hood, flywheel, intake),
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
        feedBallsStatic(indexer, hood, flywheel),
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
        # intake: IntakeSubsystem,
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
        # self.intake = intake

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
            RobotState.getHubPose()
            if RobotState.objective == RobotState.RobotMetaObjective.SHOOT
            else RobotState.getFieldPose()
        )
        turretLocation = (pose3dFrom2d(robotLocation) + kTurretLocation).toPose2d()
        if RobotState.objective == RobotState.RobotMetaObjective.SHOOT:
            self.flywheel.setGoal(float(kShootingMap(objectiveDistance)))
            self.hood.setHoodGoal(kHoodAngleMap(objectiveDistance))
        else:
            self.flywheel.setGoal(float(kFeedFlywheelMap(objectiveDistance)))
            self.hood.setHoodGoal(kHoodMaxAngle)

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
            # self.intake.setPivotGoal(PivotGoal.OSCILLATE)
            # self.intake.setRollerGoal(RollerGoal.FORWARD)

    def end(self, _interrupted: bool):
        self.flywheel.flywheelIdle()
        # self.intake.setPivotGoal(PivotGoal.DEPLOYED)
        # self.intake.setRollerGoal(RollerGoal.NEUTRAL)


def shootBasedOnOverride(
    flywheel: FlywheelSubsystem,
    hood: HoodSubsystem,
    indexer: IndexerSubsystem,
    turret: TurretSubsystem,
    drive: DriveSubsystem,
    forward: AnalogInput,
    sideways: AnalogInput,
) -> Command:
    return cmd.either(
        TurretFixedDriveShoot(
            flywheel, hood, turret, indexer, drive, forward, sideways
        ),
        cmd.parallel(
            shootBasedOnModeMoving(indexer, hood, flywheel),
            turret.getDefaultCommand(),
            drive.getDefaultCommand(),
        ),
        lambda: RobotState.turretOverriden,
    )
