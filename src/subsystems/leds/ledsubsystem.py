from typing import Optional, Union
from commands2 import Subsystem
from phoenix6.controls.empty_animation import EmptyAnimation
from phoenix6.controls.solid_color import RGBWColor, SolidColor
from phoenix6.hardware.candle import CANdle
from pykit.logger import Logger
from wpilib import DriverStation, Timer

from constants.led import (
    kCANdleCANId,
    kDisabledAnim,
    kEstopAnim,
    kPrepFlashAnim,
    kPrepAnim,
    kShootingFlashAnim,
    kShootingAnim,
    kBrownoutAnim,
    kAutoMaxFadeTime,
    kAutoOutColor,
    kCANdleTotalLedCount,
    kCANdleOnboardLedCount,
    kCANdleExternalLedCount,
    kEmptyZero,
    kEmptyOne,
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
        self._last_control_2: Optional[Union[EmptyAnimation, SolidColor]] = None
        self.lastEnabledAuto = False
        self.lastEnabledTime = 0.0

    def periodic(self) -> None:
        LogTracer.resetOuter("LEDSubsystem.periodic")
        if DriverStation.isEnabled():
            self.lastEnabledAuto = DriverStation.isAutonomous()
            self.lastEnabledTime = Timer.getFPGATimestamp()

        if DriverStation.isEStopped():
            self.candle.set_control(kEstopAnim)
            self.candle.set_control(kEmptyOne)
        elif DriverStation.isDisabled():
            if (
                self.lastEnabledAuto
                and Timer.getFPGATimestamp() - self.lastEnabledTime < kAutoMaxFadeTime
            ):
                percent = (
                    1
                    - (Timer.getFPGATimestamp() - self.lastEnabledTime)
                    / kAutoMaxFadeTime
                )
                self.candle.set_control(kEmptyZero)
                self.candle.set_control(kEmptyOne)
                self.candle.set_control(
                    SolidColor(
                        kCANdleOnboardLedCount,
                        int(percent * kCANdleExternalLedCount),
                        kAutoOutColor,
                    )
                )
                self.candle.set_control(
                    SolidColor(
                        kCANdleOnboardLedCount + int(percent * kCANdleExternalLedCount),
                        kCANdleTotalLedCount,
                        RGBWColor(0, 0, 0),
                    )
                )
            else:
                if RobotState.brownoutFlag:
                    self.candle.set_control(kBrownoutAnim)
                    self.candle.set_control(kEmptyOne)
                else:
                    self.candle.set_control(kDisabledAnim)
                    self.candle.set_control(kEmptyOne)
        else:
            if RobotState.hubAboutToChange():
                if RobotState.readyToShoot():
                    self.candle.set_control(kShootingFlashAnim)
                    self.candle.set_control(kEmptyOne)
                else:
                    self.candle.set_control(kPrepFlashAnim)
                    self.candle.set_control(kEmptyOne)
            else:
                if RobotState.readyToShoot():
                    self.candle.set_control(kShootingAnim)
                    self.candle.set_control(kEmptyOne)
                else:
                    self.candle.set_control(kPrepAnim)
                    self.candle.set_control(kEmptyOne)

        Logger.recordOutput("LED/lastEnabledAuto", self.lastEnabledAuto)
        Logger.recordOutput("LED/lastEnabledTime", self.lastEnabledTime)

        LogTracer.recordTotal()
