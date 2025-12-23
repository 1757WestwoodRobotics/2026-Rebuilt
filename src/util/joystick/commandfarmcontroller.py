from typing import Optional
from commands2.button import CommandJoystick
from wpilib.event import EventLoop


class CommandFarmController(CommandJoystick):
    def __init__(self, port: int):
        super().__init__(port)
        self.setXChannel(0)
        self.setYChannel(1)
        self.setTwistChannel(2)

    def bigButton(self, loop: Optional[EventLoop] = None):
        return self.button(21, loop)
