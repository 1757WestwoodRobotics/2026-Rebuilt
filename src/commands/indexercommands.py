from functools import partial
from commands2 import Command, cmd
from subsystems.indexer.indexersubsystem import IndexerSubsystem, IndexerSubsystemGoal


def holdIndexer(indexer: IndexerSubsystem) -> Command:
    return cmd.run(
        partial(indexer.setTarget, IndexerSubsystemGoal.HOLD), indexer
    ).withName("HoldIndexer")


def kickIndexer(indexer: IndexerSubsystem) -> Command:
    return cmd.run(
        partial(indexer.setTarget, IndexerSubsystemGoal.KICK), indexer
    ).withName("KickIndexer")


def ejectIndexer(indexer: IndexerSubsystem) -> Command:
    return cmd.run(
        partial(indexer.setTarget, IndexerSubsystemGoal.EJECT), indexer
    ).withName("EjectIndexer")
