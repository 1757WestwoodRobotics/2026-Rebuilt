from commands2 import Subsystem
from phoenix6.hardware.candle import CANdle
from wpilib import DriverStation

from constants.led import (
    kCANdleCANId,
    kDisabledAnim,
    kEstopAnim,
    kPrepFlashAnim,
    kPrepAnim,
)
from constants.drive import kCANivoreName
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

        self.candle = CANdle(kCANdleCANId, kCANivoreName)

    def periodic(self) -> None:
        LogTracer.resetOuter("LEDSubsystem.periodic")
        if DriverStation.isEStopped():
            self.candle.set_control(kEstopAnim)
        elif DriverStation.isDisabled():
            self.candle.set_control(kDisabledAnim)
        else:
            # this section is missing the checks for what the total shooter combo-system is doing
            if RobotState.hubAboutToChange():
                self.candle.set_control(kPrepFlashAnim)
            else:
                self.candle.set_control(kPrepAnim)
        LogTracer.recordTotal()
