from functools import partial
from commands2 import Command
from commands2 import cmd as Commands
from subsystems.indexer.indexersubsystem import IndexerSubsystem, IndexerSubsystemTarget


class IndexerCommands:
    @staticmethod
    def holdIndexer(indexer: IndexerSubsystem) -> Command:
        return Commands.run(
            partial(indexer.setTarget, IndexerSubsystemTarget.HOLDING), indexer
        ).withName("HoldIndexer")

    @staticmethod
    def feedIndexer(indexer: IndexerSubsystem) -> Command:
        return Commands.run(
            partial(indexer.setTarget, IndexerSubsystemTarget.SHOOT), indexer
        ).withName("FeedIndexer")

    @staticmethod
    def reverseIndexer(indexer: IndexerSubsystem) -> Command:
        return Commands.run(
            partial(indexer.setTarget, IndexerSubsystemTarget.EJECT), indexer
        ).withName("ReverseIndexer")
