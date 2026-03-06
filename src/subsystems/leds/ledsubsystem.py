from commands2 import Subsystem
from phoenix6.hardware.candle import CANdle
from wpilib import DriverStation

from constants.led import (
    kCANdleCANId,
    kDisabledAnim,
    kEstopAnim,
    kPrepFlashAnim,
    kPrepAnim,
    kShootingFlashAnim,
    kShootingAnim,
)
from constants.drive import kCANivoreCANBus
from robotstate import RobotState
from util.logtracer import LogTracer


class LEDSubsystem(Subsystem):
    """
    LED subsystem has no real simulated instance, so we are initializing it no matter what.
    If we wanted to be more correct we could make a simulated CANdle that just logs the set control,
    but it doesn't seem worth the effort at this time.
    """

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.candle = CANdle(kCANdleCANId, kCANivoreCANBus)

        self._last_control = None

    def periodic(self) -> None:
        LogTracer.resetOuter("LEDSubsystem.periodic")
        if DriverStation.isEStopped():
            desired_control = kEstopAnim
        elif DriverStation.isDisabled():
            if RobotState.brownoutFlag:
                desired_control = kEstopAnim
            else:
                desired_control = kDisabledAnim
        else:
            if RobotState.hubAboutToChange():
                if RobotState.readyToShoot():
                    desired_control = kShootingFlashAnim
                else:
                    desired_control = kPrepFlashAnim
            else:
                if RobotState.readyToShoot():
                    desired_control = kShootingAnim
                else:
                    desired_control = kPrepAnim
        # Only send a new control if it has changed to reduce CAN bus usage
        if desired_control is not self._last_control:
            self.candle.set_control(desired_control)
            self._last_control = desired_control
        LogTracer.recordTotal()
