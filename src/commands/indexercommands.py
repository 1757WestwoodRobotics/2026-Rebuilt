from functools import partial
from commands2 import Command, cmd
from subsystems.indexer.indexersubsystem import IndexerSubsystem, IndexerSubsystemTarget


def holdIndexer(indexer: IndexerSubsystem) -> Command:
    return cmd.run(
        partial(indexer.setTarget, IndexerSubsystemTarget.HOLD), indexer
    ).withName("HoldIndexer")


def kickIndexer(indexer: IndexerSubsystem) -> Command:
    return cmd.run(
        partial(indexer.setTarget, IndexerSubsystemTarget.KICK), indexer
    ).withName("KickIndexer")


def ejectIndexer(indexer: IndexerSubsystem) -> Command:
    return cmd.run(
        partial(indexer.setTarget, IndexerSubsystemTarget.EJECT), indexer
    ).withName("EjectIndexer")
