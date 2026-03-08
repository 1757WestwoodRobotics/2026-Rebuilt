from commands2 import Command, cmd
from robotstate import RobotState
from subsystems.flywheel.flywheelsubsystem import FlywheelSubsystem
from subsystems.hood.hoodsubsystem import HoodSubsystem
from subsystems.indexer.indexersubsystem import IndexerSubsystem

import commands.indexercommands as IndexerCommands  # module, not class
import commands.flywheelcommands as FlywheelCommands
import commands.hoodcommands as HoodCommands


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
