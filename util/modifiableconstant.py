from wpilib import Preferences


class ModifiableConstant:
    def __init__(self, name: str, default: float):
        self.default = default
        self.name = name
        self.lookup = f"constants/{self.name}"

        Preferences.setDouble(self.lookup, self.default)

    @property
    def value(self) -> float:
        return Preferences.getDouble(self.lookup, self.default)

    @value.setter
    def set_value(self, value: float):
        Preferences.setDouble(self.lookup, value)
