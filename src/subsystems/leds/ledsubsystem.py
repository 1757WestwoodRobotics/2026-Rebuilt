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


class LEDSubsystem(Subsystem):
    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.candle = CANdle(kCANdleCANId, kCANivoreName)

    def periodic(self) -> None:
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
