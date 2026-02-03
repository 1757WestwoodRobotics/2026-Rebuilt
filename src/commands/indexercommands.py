from functools import partial
from commands2 import Command, cmd
from subsystems.indexer.indexersubsystem import IndexerSubsystem, IndexerSubsystemTarget


def holdIndexer(indexer: IndexerSubsystem) -> Command:
    return cmd.run(
        partial(indexer.setTarget, IndexerSubsystemTarget.HOLDING), indexer
    ).withName("HoldIndexer")


def feedIndexer(indexer: IndexerSubsystem) -> Command:
    return cmd.run(
        partial(indexer.setTarget, IndexerSubsystemTarget.SHOOT), indexer
    ).withName("FeedIndexer")


def reverseIndexer(indexer: IndexerSubsystem) -> Command:
    return cmd.run(
        partial(indexer.setTarget, IndexerSubsystemTarget.EJECT), indexer
    ).withName("ReverseIndexer")
