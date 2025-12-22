from typing import Callable
from phoenix6 import BaseStatusSignal, StatusSignal

from phoenix6.status_code import StatusCode


def tryUntilOk(attempts: int, command: Callable[[], StatusCode]):
    for _ in range(attempts):
        code = command()
        if code.is_ok():
            break


class PhoenixUtil:
    registered_signals: dict[str, list[StatusSignal]] = {}

    @classmethod
    def registerSignal(cls, canbus: str, signal: StatusSignal):
        if canbus not in cls.registered_signals:
            cls.registered_signals[canbus] = []
        cls.registered_signals[canbus].append(signal)

    @classmethod
    def registerSignals(cls, canbus: str, *signals: StatusSignal):
        for signal in signals:
            cls.registerSignal(canbus, signal)

    @classmethod
    def updateSignals(cls):
        for signals in cls.registered_signals.values():
            BaseStatusSignal.refresh_all(signals)
