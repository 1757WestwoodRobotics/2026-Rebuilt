from enum import Enum
from commands2 import Subsystem
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

    # LED state identifiers for caching — avoids sending redundant CAN frames
    class LEDStates(Enum):
        ESTOP = (kEstopAnim, kEmptyOne)
        BROWNOUT = (kBrownoutAnim, kEmptyOne)
        DISABLED = (kDisabledAnim, kEmptyOne)
        AUTO_FADE = (kEmptyZero, kEmptyOne)
        PREP = (kPrepAnim, kEmptyOne, kEmptyZero)
        PREP_FLASH = (kPrepFlashAnim, kEmptyOne)
        SHOOTING = (kShootingAnim, kEmptyOne, kEmptyZero)
        SHOOTING_FLASH = (kShootingFlashAnim, kEmptyOne)
        NEUTRAL = ()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.candle = CANdle(kCANdleCANId, kCANivoreCANBus)

        self._lastFadePercent: int = -1  # track fade level for auto-fade animation
        self._lastState = self.LEDStates.NEUTRAL
        self.lastEnabledAuto = False
        self.lastEnabledTime = 0.0

    def setStateAnim(self, state: LEDStates) -> None:
        """Sets the last state and executes animations."""
        if self._lastState == state:
            return
        else:
            self._lastState = state
            for anim in self._lastState.value:
                self.candle.set_control(anim)

    def periodic(self) -> None:
        LogTracer.resetOuter("LEDSubsystem.periodic")
        if DriverStation.isEnabled():
            self.lastEnabledAuto = DriverStation.isAutonomous()
            self.lastEnabledTime = Timer.getFPGATimestamp()

        if DriverStation.isEStopped():
            self.setStateAnim(self.LEDStates.ESTOP)
        elif DriverStation.isDisabled():
            if (
                self.lastEnabledAuto
                and Timer.getFPGATimestamp() - self.lastEnabledTime < kAutoMaxFadeTime
            ):
                # Auto-fade: only send CAN frames when the integer fade level changes
                percent = (
                    1
                    - (Timer.getFPGATimestamp() - self.lastEnabledTime)
                    / kAutoMaxFadeTime
                )
                fadeLevel = int(percent * kCANdleExternalLedCount)
                if fadeLevel != self._lastFadePercent:
                    self._lastFadePercent = fadeLevel
                    self.setStateAnim(self.LEDStates.AUTO_FADE)
                    self.candle.set_control(
                        SolidColor(
                            kCANdleOnboardLedCount,
                            fadeLevel,
                            kAutoOutColor,
                        )
                    )
                    self.candle.set_control(
                        SolidColor(
                            kCANdleOnboardLedCount + fadeLevel,
                            kCANdleTotalLedCount,
                            RGBWColor(0, 0, 0),
                        )
                    )
            else:
                if RobotState.brownoutFlag:
                    self.setStateAnim(self.LEDStates.BROWNOUT)
                else:
                    self.setStateAnim(self.LEDStates.DISABLED)
        else:
            if RobotState.hubAboutToChange():
                if RobotState.readyToShoot():
                    self.setStateAnim(self.LEDStates.SHOOTING_FLASH)
                else:
                    self.setStateAnim(self.LEDStates.PREP_FLASH)
            else:
                if RobotState.readyToShoot():
                    self.setStateAnim(self.LEDStates.SHOOTING)
                else:
                    self.setStateAnim(self.LEDStates.PREP)

        Logger.recordOutput("LED/lastEnabledAuto", self.lastEnabledAuto)
        Logger.recordOutput("LED/lastEnabledTime", self.lastEnabledTime)

        LogTracer.recordTotal()
