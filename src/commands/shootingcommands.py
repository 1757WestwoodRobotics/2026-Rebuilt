from commands2 import Command, cmd
from robotstate import RobotState
from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem
from subsystems.hood.hoodsubsystem import HoodSubsystem
from subsystems.indexer.indexersubsystem import IndexerSubsystem
from subsystems.intake.intakesubsystem import IntakeSubsystem

import commands.indexercommands as IndexerCommands  # module, not class
import commands.flywheelcommands as FlywheelCommands
import commands.intakecommands as IntakeCommands
import commands.hoodcommands as HoodCommands

from constants.hood import kHoodMaxAngle


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
