from typing import Callable
from pykit.networktables.loggednetworknumber import LoggedNetworkNumber

from constants import kTuningMode


class LoggedTunableNumber:
    """
    A number that can be tuned via a dashboard when in tuning mode.
    This is linked to a NetworkTables entry at /Tuning/<key>.
    See https://docs.advantagescope.org/overview/live-sources/tuning-mode/ for more information about tuning mode.
    """

    _tableKey: str = "/Tuning"
    _lastHasChangedValues: dict[str, float] = {}
    _tunableNumbers: list["LoggedTunableNumber"] = []

    _key: str
    _default: float
    _dashboardNumber: LoggedNetworkNumber

    _callbacks: list[Callable[[float], None]]

    def __init__(self, key: str, default: float = 0.0) -> None:
        self._key = LoggedTunableNumber._tableKey + "/" + key
        self._default = default
        self._callbacks = []
        if kTuningMode:
            self._dashboardNumber = LoggedNetworkNumber(self._key, default)

        LoggedTunableNumber._tunableNumbers.append(self)

    def get(self) -> float:
        """returns the current value of the tunable number, if in tuning mode, otherwise returns the default value"""
        if kTuningMode:
            assert hasattr(self, "_dashboardNumber")
            return self._dashboardNumber.value
        return self._default

    def hasChanged(self) -> bool:
        """
        returns True if the value has changed since the last time this method was called, False otherwise
        if the value has never been checked before, it will return True in tuning mode, and False otherwise
        """
        currentValue = self.get()
        if self._key not in self._lastHasChangedValues:
            self._lastHasChangedValues[self._key] = currentValue
            return kTuningMode
        if self._lastHasChangedValues[self._key] != currentValue:
            self._lastHasChangedValues[self._key] = currentValue
            return True
        return False

    @staticmethod
    def ifChanged(
        func: Callable[[list[float]], None], tunableNumbers: "list[LoggedTunableNumber]"
    ) -> None:
        """calls a callback with the current values of tunableNumbers if any of them have changed since the last time this method was called"""
        anyChanged = False
        for tunableNumber in tunableNumbers:
            if tunableNumber.hasChanged():
                anyChanged = True
                break
        if anyChanged:
            func([tunableNumber.get() for tunableNumber in tunableNumbers])

    def onChange(self, func: Callable[[float], None]) -> None:
        """registers a callback to be called when the value changes"""
        self._callbacks.append(func)

    def periodic(self) -> None:
        """calls the registered callbacks if the value has changed"""
        if self.hasChanged():
            currentValue = self.get()
            for callback in self._callbacks:
                callback(currentValue)

    @staticmethod
    def updateAll() -> None:
        """calls periodic on all LoggedTunableNumbers to check for changes and call callbacks"""
        for tunableNumber in LoggedTunableNumber._tunableNumbers:
            tunableNumber.periodic()


class AutoUpdateGroup:
    """
    A group of LoggedTunableNumbers that will automatically call a callback when any of them change.
    """

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

        AutoUpdateGroup._updateGroups.append(self)

    def periodic(self) -> None:
        """calls the callback if any of the tunable numbers have changed"""
        currentValues = [tunableNumber.get() for tunableNumber in self._tunableNumbers]
        if currentValues != self._lastValues:
            self._lastValues = currentValues
            self._callback(*currentValues)

    @staticmethod
    def updateAll() -> None:
        """calls periodic on all AutoUpdateGroups to check for changes and call callbacks"""
        for updateGroup in AutoUpdateGroup._updateGroups:
            updateGroup.periodic()
