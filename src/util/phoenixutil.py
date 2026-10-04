import time
from typing import Callable
from phoenix6 import BaseStatusSignal, CANBus, StatusSignal
from phoenix6.status_code import StatusCode
from wpilib import RobotBase


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
    registered_control_signals: dict[CANBus, list[StatusSignal]] = {}
    registered_diag_signals: dict[CANBus, list[StatusSignal]] = {}
    _diag_counter: int = 0
    _diag_subsample_ratio: int = (
        5  # refresh diag signals every 5 cycles (10 Hz at 50 Hz main loop)
    )

    @classmethod
    def registerSignal(
        cls, canbus: CANBus, signal: StatusSignal, is_diagnostic: bool = False
    ):
        target_dict = (
            cls.registered_diag_signals
            if is_diagnostic
            else cls.registered_control_signals
        )
        if canbus not in target_dict:
            target_dict[canbus] = []
        target_dict[canbus].append(signal)

    @classmethod
    def registerSignals(
        cls, canbus: CANBus, *signals: StatusSignal, is_diagnostic: bool = False
    ):
        for signal in signals:
            cls.registerSignal(canbus, signal, is_diagnostic=is_diagnostic)

    @classmethod
    def updateSignals(cls):
        if not RobotBase.isReal():
            return
        # Always refresh control-critical signals
        for signals in cls.registered_control_signals.values():
            BaseStatusSignal.refresh_all(signals)

        # Sub-sample diagnostic signals to reduce CAN bus IPC overhead
        cls._diag_counter += 1
        if cls._diag_counter >= cls._diag_subsample_ratio:
            cls._diag_counter = 0
            for signals in cls.registered_diag_signals.values():
                BaseStatusSignal.refresh_all(signals)
