from enum import Enum, auto

from math import copysign
from typing import Callable, Tuple
from commands2.button import Trigger
from pykit.networktables.loggednetworkboolean import LoggedNetworkBoolean
from wpilib import Joystick

from util.controltype import AnalogInput
from util.convenientmath import number, map_range


Axis = Callable[[], float]


class AxisButton(Trigger):
    """a trigger that can be fired by an axis hitting a certain limit"""

    def __init__(self, axis: Axis, threshold: float = 0.5) -> None:
        self.axis = axis
        self.threshold = threshold
        super().__init__(lambda: self.axis() > self.threshold)


class NetworkTableButton(Trigger):
    def __init__(self, key: str) -> None:
        self.logBool = LoggedNetworkBoolean(key, False)
        self.logBool.value = False
        super().__init__(lambda: self.logBool.value)


class ModifiableJoystickButton(Trigger):
    def _active(self):
        joystick, button = self.call()
        return joystick.getRawButton(button)

    def __init__(self, call: Callable[[], Tuple[Joystick, int]]):
        self.call = call
        super().__init__(self._active)


class DPadButton(Trigger):
    """a trigger that can be fired by a d pad button"""

    class DPad(Enum):
        Up = auto()
        Down = auto()
        Left = auto()
        Right = auto()

        def isNum(self, direction: int):
            if self == self.Up:
                return direction in [315, 0, 45]
            elif self == self.Down:
                return 135 <= direction <= 225
            elif self == self.Left:
                return 225 <= direction <= 315
            elif self == self.Right:
                return 45 <= direction <= 135
            else:
                return False

    def __init__(self, controller: Joystick, POVNumber: int, button: DPad):
        super().__init__(lambda: button.isNum(controller.getPOV(POVNumber)))


def Deadband(inputFn: AnalogInput, deadband: float) -> AnalogInput:
    def withDeadband() -> float:
        value = inputFn()
        if abs(value) <= deadband:
            return 0
        else:
            return value

    return withDeadband


def Invert(inputFn: AnalogInput) -> AnalogInput:
    def invert() -> float:
        return -1 * inputFn()

    return invert


def SignSquare(inputFn: AnalogInput) -> AnalogInput:
    def square() -> float:
        val = inputFn()
        return copysign(val * val, val)

    return square


def MapRange(
    inputFn: AnalogInput,
    inputMin: number,
    inputMax: number,
    outputMin: number,
    outputMax: number,
) -> AnalogInput:
    return lambda: map_range(inputFn(), inputMin, inputMax, outputMin, outputMax)


def Multiply(a: AnalogInput, b: AnalogInput) -> AnalogInput:
    return lambda: a() * b()
