from typing import Optional
from commands2.button import CommandJoystick
from wpilib.event import EventLoop


class CommandTWCSThrottle(CommandJoystick):
    def __init__(self, port: int):
        super().__init__(port)
        self.setXChannel(0)  # these are GUESSES!!! They may not be accuragte
        self.setYChannel(1)
        self.setZChannel(2)
        self.setThrottleChannel(3)

    def thumb(self, loop: Optional[EventLoop] = None):
        return self.button(1, loop)

    def pinky(self, loop: Optional[EventLoop] = None):
        return self.button(2, loop)

    def ring(self, loop: Optional[EventLoop] = None):
        return self.button(3, loop)

    def rockerUp(self, loop: Optional[EventLoop] = None):
        return self.button(4, loop)

    def rockerDown(self, loop: Optional[EventLoop] = None):
        return self.button(5, loop)

    def joystickButton(self, loop: Optional[EventLoop] = None):
        return self.button(6, loop)

    def midClusterUp(self, loop: Optional[EventLoop] = None):
        return self.button(7, loop)

    def midClusterRight(self, loop: Optional[EventLoop] = None):
        return self.button(8, loop)

    def midClusterDown(self, loop: Optional[EventLoop] = None):
        return self.button(9, loop)

    def midClusterLeft(self, loop: Optional[EventLoop] = None):
        return self.button(10, loop)

    def lowerClusterUp(self, loop: Optional[EventLoop] = None):
        return self.button(11, loop)

    def lowerClusterRight(self, loop: Optional[EventLoop] = None):
        return self.button(12, loop)

    def lowerClusterDown(self, loop: Optional[EventLoop] = None):
        return self.button(13, loop)

    def lowerClusterLeft(self, loop: Optional[EventLoop] = None):
        return self.button(14, loop)
