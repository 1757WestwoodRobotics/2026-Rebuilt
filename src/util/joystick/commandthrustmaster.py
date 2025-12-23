from typing import Optional
from commands2.button import CommandJoystick
from wpilib.event import EventLoop


class CommandThrustmasterT16000M(CommandJoystick):
    def __init__(self, port: int):
        super().__init__(port)
        self.setXChannel(0)
        self.setYChannel(1)
        self.setTwistChannel(2)
        self.setThrottleChannel(3)

    def triggerButton(self, loop: Optional[EventLoop] = None):
        return self.button(1, loop)

    def topClusterBottom(self, loop: Optional[EventLoop] = None):
        return self.button(2, loop)

    def topClusterLeft(self, loop: Optional[EventLoop] = None):
        return self.button(3, loop)

    def topClusterRight(self, loop: Optional[EventLoop] = None):
        return self.button(4, loop)

    def rightClusterRightTop(self, loop: Optional[EventLoop] = None):
        return self.button(5, loop)

    def rightClusterMiddleTop(self, loop: Optional[EventLoop] = None):
        return self.button(6, loop)

    def rightClusterLeftTop(self, loop: Optional[EventLoop] = None):
        return self.button(7, loop)

    def rightClusterLeftBottom(self, loop: Optional[EventLoop] = None):
        return self.button(8, loop)

    def rightClusterMiddleBottom(self, loop: Optional[EventLoop] = None):
        return self.button(9, loop)

    def rightClusterRightBottom(self, loop: Optional[EventLoop] = None):
        return self.button(10, loop)

    def leftClusterLeftTop(self, loop: Optional[EventLoop] = None):
        return self.button(11, loop)

    def leftClusterMiddleTop(self, loop: Optional[EventLoop] = None):
        return self.button(12, loop)

    def leftClusterRightTop(self, loop: Optional[EventLoop] = None):
        return self.button(13, loop)

    def leftClusterRightBottom(self, loop: Optional[EventLoop] = None):
        return self.button(14, loop)

    def leftClusterMiddleBottom(self, loop: Optional[EventLoop] = None):
        return self.button(15, loop)

    def leftClusterLeftBottom(self, loop: Optional[EventLoop] = None):
        return self.button(16, loop)
