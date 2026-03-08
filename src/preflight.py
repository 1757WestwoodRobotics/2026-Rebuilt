from pykit.networktables.loggednetworkboolean import LoggedNetworkBoolean
from wpilib import Alert


class PreflightChecklist:
    class PreflightCheck:
        def __init__(self, name: str, key: str, expected: bool):
            self._key = key
            self._name = name
            self.value = LoggedNetworkBoolean(key, not expected)
            self.alert = Alert(
                "preflight", "Check Failed: " + name, Alert.AlertType.kError
            )

        def update(self):
            """
            Sets the proper alert for this check based on the value, which should be set by the 
            user to indicate whether the check has been completed or not
            """
            self.alert.set(not self.value())

        @property
        def name(self) -> str:
            return self._name

    def __init__(self):
        self.checks: list[PreflightChecklist.PreflightCheck] = [
            PreflightChecklist.PreflightCheck(
                "Robot has Power", "Preflight/RobotPower", True
            ),
            PreflightChecklist.PreflightCheck(
                "Robot was powered on in starting config", "Preflight/RobotStart", True
            ),
            PreflightChecklist.PreflightCheck(
                "E-Stop is Disengaged", "Preflight/E-Stop", True
            ),
            PreflightChecklist.PreflightCheck(
                "Xbox is Connected", "Preflight/Xbox", True
            ),
            PreflightChecklist.PreflightCheck(
                "Farm is Connected", "Preflight/Farm", True
            ),
            PreflightChecklist.PreflightCheck(
                "Autonomous is Selected", "Preflight/Autonomous", True
            ),
            PreflightChecklist.PreflightCheck(
                "Robot in correct autonomous position", "Preflight/AutoLoc", True
            ),
            PreflightChecklist.PreflightCheck(
                "Vision returns Results", "Preflight/Vision", True
            ),
        ]

    def update(self):
        for check in reversed(self.checks):
            check.update()

    def is_complete(self) -> bool:
        return all(check.value for check in self.checks)

    def missing(self) -> list[str]:
        return [check._key for check in self.checks if not check.value]
