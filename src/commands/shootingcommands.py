from commands2 import Command, cmd
from robotstate import RobotState
from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem
from subsystems.hood.hoodsubsystem import HoodSubsystem
from subsystems.indexer.indexersubsystem import IndexerSubsystem

import commands.indexercommands as IndexerCommands  # module, not class
import commands.flywheelcommands as FlywheelCommands
import commands.hoodcommands as HoodCommands

from constants.hood import kHoodMaxAngle
from constants.shooting import kFeedShootingPower


def shootBalls(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to shoot balls when the flywheel is at speed and the hood is at angle
    """
    return cmd.parallel(
        FlywheelCommands.shootWithDistance(flywheel, RobotState.distanceToHub),
        HoodCommands.angleHoodWithDistance(hood, RobotState.distanceToHub),
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            IndexerCommands.kickIndexer(indexer)
        ),
    )


def feedBalls(
    indexer: IndexerSubsystem, hood: HoodSubsystem, flywheel: FlywheelSubsystem
) -> Command:
    """
    Command to feed balls into the flywheel without waiting for it to be at speed or the hood to be at angle
    """
    return cmd.parallel(
        FlywheelCommands.feedWithDistance(flywheel, RobotState.distanceToObjective),
        HoodCommands.angleHood(hood, lambda: kHoodMaxAngle),
        cmd.waitUntil(RobotState.readyToShoot).andThen(
            IndexerCommands.kickIndexer(indexer)
        ),
    )


def setFeedObjective() -> Command:
    def _set():
        RobotState.objective = RobotState.RobotMetaObjective.FEED

    return cmd.runOnce(_set).withName("FeedObjective")


def setShootObjective() -> Command:
    def _set():
        RobotState.objective = RobotState.RobotMetaObjective.SHOOT

    return cmd.runOnce(_set).withName("ShootObjective")
