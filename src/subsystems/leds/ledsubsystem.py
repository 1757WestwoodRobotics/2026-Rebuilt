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

    def periodic(self) -> None:
        LogTracer.resetOuter("LEDSubsystem.periodic")
        if DriverStation.isEStopped():
            self.candle.set_control(kEstopAnim)
        elif DriverStation.isDisabled():
            if RobotState.brownoutFlag:
                self.candle.set_control(kEstopAnim)
            else:
                self.candle.set_control(kDisabledAnim)
        else:
            if RobotState.hubAboutToChange():
                if RobotState.readyToShoot():
                    self.candle.set_control(kShootingFlashAnim)
                else:
                    self.candle.set_control(kPrepFlashAnim)
            else:
                if RobotState.readyToShoot():
                    self.candle.set_control(kShootingAnim)
                else:
                    self.candle.set_control(kPrepAnim)
        LogTracer.recordTotal()
