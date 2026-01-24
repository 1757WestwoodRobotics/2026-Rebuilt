from typing import Callable
from pykit.networktables.loggednetworknumber import LoggedNetworkNumber

from constants import kTuningMode


class LoggedTunableNumber:
    _tableKey: str = "/Tuning"
    _lastHasChangedValues: dict[str, float] = {}
    _tunableNumbers: list["LoggedTunableNumber"] = []

    _key: str
    _default: float
    _dashboardNumber: LoggedNetworkNumber

    _callbacks: list[Callable[[float], None]] = []

    def __init__(self, key: str, default: float = 0.0) -> None:
        self._key = LoggedTunableNumber._tableKey + "/" + key
        self._default = default
        if kTuningMode:
            self._dashboardNumber = LoggedNetworkNumber(self._key, default)

        LoggedTunableNumber._tunableNumbers.append(self)

    def get(self) -> float:
        return self._dashboardNumber.value if kTuningMode else self._default

    def hasChanged(self) -> bool:
        currentValue = self.get()
        if self._key not in self._lastHasChangedValues:
            self._lastHasChangedValues[self._key] = currentValue
            return True
        if self._lastHasChangedValues[self._key] != currentValue:
            self._lastHasChangedValues[self._key] = currentValue
            return True
        return False

    @staticmethod
    def ifChanged(
        func: Callable[[list[float]], None], tunableNumbers: "list[LoggedTunableNumber]"
    ) -> None:
        anyChanged = False
        for tunableNumber in tunableNumbers:
            if tunableNumber.hasChanged():
                anyChanged = True
                break
        if anyChanged:
            func([tunableNumber.get() for tunableNumber in tunableNumbers])

    def onChange(self, func: Callable[[float], None]) -> None:
        self._callbacks.append(func)

    def periodic(self) -> None:
        if self.hasChanged():
            currentValue = self.get()
            for callback in self._callbacks:
                callback(currentValue)

    @staticmethod
    def updateAll() -> None:
        for tunableNumber in LoggedTunableNumber._tunableNumbers:
            tunableNumber.periodic()


class AutoUpdateGroup:
    _callback: Callable[..., None]
    _tunableNumbers: list[LoggedTunableNumber]
    _lastValues: list[float]

    _updateGroups: list["AutoUpdateGroup"] = []

    def __init__(
        self,
        callback: Callable[..., None],
        tunableNumbers: list[LoggedTunableNumber],
    ) -> None:
        self._callback = callback
        self._tunableNumbers = tunableNumbers
        self._lastValues = [tunableNumber.get() for tunableNumber in tunableNumbers]

    def periodic(self) -> None:
        currentValues = [tunableNumber.get() for tunableNumber in self._tunableNumbers]
        if currentValues != self._lastValues:
            self._lastValues = currentValues
            self._callback(*currentValues)

    @staticmethod
    def updateAll() -> None:
        for updateGroup in AutoUpdateGroup._updateGroups:
            updateGroup.periodic()
