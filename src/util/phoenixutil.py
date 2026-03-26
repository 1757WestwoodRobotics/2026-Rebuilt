import time
from typing import Callable
from phoenix6 import BaseStatusSignal, CANBus, StatusSignal

from phoenix6.status_code import StatusCode


def tryUntilOk(attempts: int, command: Callable[[], StatusCode], label: str = ""):
    if attempts <= 0:
        raise ValueError("attempts must be greater than 0")

    start = time.monotonic()
    for attempt in range(attempts):
        code = command()
        if code.is_ok():
            elapsed_ms = (time.monotonic() - start) * 1000
            if attempt > 0:
                print(
                    f"[CAN Config] {label}: OK after {attempt + 1} attempts ({elapsed_ms:.0f}ms)"
                )
            return
    elapsed_ms = (time.monotonic() - start) * 1000
    print(
        f"[CAN Config] WARNING: {label}: FAILED all {attempts} attempts ({elapsed_ms:.0f}ms) - last status: {code}"
    )


class PhoenixUtil:
    registered_signals: dict[CANBus, list[StatusSignal]] = {}

    @classmethod
    def registerSignal(cls, canbus: CANBus, signal: StatusSignal):
        if canbus not in cls.registered_signals:
            cls.registered_signals[canbus] = []
        cls.registered_signals[canbus].append(signal)

    @classmethod
    def registerSignals(cls, canbus: CANBus, *signals: StatusSignal):
        for signal in signals:
            cls.registerSignal(canbus, signal)

    @classmethod
    def updateSignals(cls):
        for signals in cls.registered_signals.values():
            BaseStatusSignal.refresh_all(signals)
