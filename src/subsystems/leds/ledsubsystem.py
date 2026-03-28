from enum import Enum, auto
from typing import Optional
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
        STATE_ESTOP = (kEstopAnim, kEmptyOne)
        STATE_BROWNOUT = (kBrownoutAnim, kEmptyOne)
        STATE_DISABLED = (kDisabledAnim, kEmptyOne)
        STATE_AUTO_FADE = (kEmptyZero, kEmptyOne)
        STATE_PREP = (kShootingAnim, kEmptyOne, kEmptyZero)
        STATE_PREP_FLASH = (kPrepFlashAnim, kEmptyOne)
        STATE_SHOOTING = (kShootingAnim, kEmptyOne, kEmptyZero)
        STATE_SHOOTING_FLASH = (kShootingFlashAnim, kEmptyOne)
        STATE_NEUTRAL = ()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.candle = CANdle(kCANdleCANId, kCANivoreCANBus)

        self._lastFadePercent: int = -1  # track fade level for auto-fade animation
        self._lastState = self.LEDStates.STATE_NEUTRAL
        self.lastEnabledAuto = False
        self.lastEnabledTime = 0.0
        # self.estopState = "estop"
        # self.brownoutState = "brownout"
        # self.disabledState = "disabled"
        # self.autoFadeState = "auto_fade"
        # self.prepState = "prep"
        # self.prepFlashState = "prep_flash"
        # self.shootingState = "shooting"
        # self.shootingFlashState = "shooting_flash"

    # def _setState(self, state: str) -> bool:
    #     """Returns True if the state changed and CAN frames should be sent."""
    #     if self._lastState == state:
    #         return False
    #     self._lastState = state
    #     return True

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
            self.setStateAnim(self.LEDStates.STATE_ESTOP)
            # if self._setState(self.estopState):
            #     self.candle.set_control(kEstopAnim)
            #     self.candle.set_control(kEmptyOne)
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
                    self.setStateAnim(self.LEDStates.STATE_AUTO_FADE)
                    # self._lastState = self.LEDStates.STATE_AUTO_FADE

                    # self.candle.set_control(kEmptyZero)
                    # self.candle.set_control(kEmptyOne)
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
                    self.setStateAnim(self.LEDStates.STATE_BROWNOUT)
                    # if self._setState(self.brownoutState):
                    #     self.candle.set_control(kBrownoutAnim)
                    #     self.candle.set_control(kEmptyOne)
                else:
                    self.setStateAnim(self.LEDStates.STATE_DISABLED)

                    # if self._setState(self.disabledState):

                    #     self.candle.set_control(kDisabledAnim)
                    #     self.candle.set_control(kEmptyOne)
        else:
            if RobotState.hubAboutToChange():
                if RobotState.readyToShoot():
                    self.setStateAnim(self.LEDStates.STATE_SHOOTING_FLASH)

                    # if self._setState(self.shootingFlashState):
                    #     self.candle.set_control(kShootingFlashAnim)
                    #     self.candle.set_control(kEmptyOne)
                else:
                    self.setStateAnim(self.LEDStates.STATE_PREP_FLASH)

                    # if self._setState(self.prepFlashState):
                    #     self.candle.set_control(kPrepFlashAnim)
                    #     self.candle.set_control(kEmptyOne)
            else:
                if RobotState.readyToShoot():
                    self.setStateAnim(self.LEDStates.STATE_SHOOTING)

                    # if self._setState(self.shootingState):
                    #     self.candle.set_control(kShootingAnim)
                    #     self.candle.set_control(kEmptyOne)
                    #     self.candle.set_control(kEmptyZero)
                else:
                    self.setStateAnim(self.LEDStates.STATE_PREP)

                    # if self._setState(self.prepState):
                    #     self.candle.set_control(kPrepAnim)
                    #     self.candle.set_control(kEmptyOne)
                    #     self.candle.set_control(kEmptyZero)

        Logger.recordOutput("LED/lastEnabledAuto", self.lastEnabledAuto)
        Logger.recordOutput("LED/lastEnabledTime", self.lastEnabledTime)

        LogTracer.recordTotal()
