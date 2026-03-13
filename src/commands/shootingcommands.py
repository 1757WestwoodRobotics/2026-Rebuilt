from commands2 import Command, cmd
from robotstate import RobotState
from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem
from subsystems.hood.hoodsubsystem import HoodSubsystem
from subsystems.indexer.indexersubsystem import IndexerSubsystem

import commands.indexercommands as IndexerCommands  # module, not class
import commands.flywheelcommands as FlywheelCommands
import commands.hoodcommands as HoodCommands

from constants.hood import kHoodMaxAngle


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
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            IndexerCommands.kickIndexer(indexer)
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
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            IndexerCommands.kickIndexer(indexer)
        ),
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
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            IndexerCommands.kickIndexer(indexer)
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
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            IndexerCommands.kickIndexer(indexer)
        ),
    )


def shootBasedOnModeMoving(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to shoot or feed balls while moving based on the current objective, with compensation for robot velocity
    """
    return cmd.either(
        shootBallsMoving(indexer, hood, flywheel),
        feedBallsMoving(indexer, hood, flywheel),
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
