from typing import Callable, Union
from commands2.button import Trigger
from wpilib import Joystick

AnalogInput = Callable[[], float]


class ControlBase:
    controller_id: int
    joysticks: dict[int, Joystick] = {}

    def __init__(self, joy_id: int) -> None:
        if joy_id not in ControlBase.joysticks:
            ControlBase.joysticks[joy_id] = Joystick(joy_id)
        self.controller_id = joy_id

    def __call__(self) -> Union[AnalogInput, Trigger]:
        raise NotImplementedError("Should be implemented by a subtype")


class ControlButton(ControlBase):
    button_idx: int

    def __init__(self, joy_id: int, button_idx: int) -> None:
        super().__init__(joy_id)
        self.button_idx = button_idx

    def __call__(self) -> Trigger:
        def condition() -> bool:
            return ControlBase.joysticks[self.controller_id].getRawButton(
                self.button_idx
            )

        return Trigger(condition)


class ControlAxis(ControlBase):
    axis_idx: int

    def __init__(self, joy_id: int, axis_idx: int) -> None:
        super().__init__(joy_id)
        self.axis_idx = axis_idx

    def __call__(self) -> AnalogInput:
        return lambda: ControlBase.joysticks[self.controller_id].getRawAxis(
            self.axis_idx
        )


class ControlPOV(ControlBase):
    pov_idx: int
    pov_angle: int

    def __init__(self, joy_id: int, pov_idx: int, pov_angle: int) -> None:
        super().__init__(joy_id)
        self.pov_idx = pov_idx
        self.pov_angle = pov_angle

    def __call__(self) -> AnalogInput:
        return (
            lambda: ControlBase.joysticks[self.controller_id].getPOV(self.pov_idx)
            == self.pov_angle
        )
