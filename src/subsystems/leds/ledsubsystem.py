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
        STATE_ESTOP = auto()
        STATE_BROWNOUT = auto()
        STATE_DISABLED = auto()
        STATE_AUTO_FADE = auto()
        STATE_PREP = auto()
        STATE_PREP_FLASH = auto()
        STATE_SHOOTING = auto()
        STATE_SHOOTING_FLASH = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)

        self.candle = CANdle(kCANdleCANId, kCANivoreCANBus)

        self._lastState: Optional[str] = None
        self._lastFadePercent: int = -1  # track fade level for auto-fade animation
        self.lastEnabledAuto = False
        self.lastEnabledTime = 0.0
        self.estopState = "estop"
        self.brownoutState = "brownout"
        self.disabledState = "disabled"
        self.autoFadeState = "auto_fade"
        self.prepState = "prep"
        self.prepFlashState = "prep_flash"
        self.shootingState = "shooting"
        self.shootingFlashState = "shooting_flash"

    def _setState(self, state: str) -> bool:
        """Returns True if the state changed and CAN frames should be sent."""
        if self._lastState == state:
            return False
        self._lastState = state
        return True

    def periodic(self) -> None:
        LogTracer.resetOuter("LEDSubsystem.periodic")
        if DriverStation.isEnabled():
            self.lastEnabledAuto = DriverStation.isAutonomous()
            self.lastEnabledTime = Timer.getFPGATimestamp()

        if DriverStation.isEStopped():
            if self._setState(self.estopState):
                self.candle.set_control(kEstopAnim)
                self.candle.set_control(kEmptyOne)
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
                    self._lastState = self.autoFadeState
                    self._lastFadePercent = fadeLevel
                    self.candle.set_control(kEmptyZero)
                    self.candle.set_control(kEmptyOne)
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
                    if self._setState(self.brownoutState):
                        self.candle.set_control(kBrownoutAnim)
                        self.candle.set_control(kEmptyOne)
                else:
                    if self._setState(self.disabledState):
                        self.candle.set_control(kDisabledAnim)
                        self.candle.set_control(kEmptyOne)
        else:
            if RobotState.hubAboutToChange():
                if RobotState.readyToShoot():
                    if self._setState(self.shootingFlashState):
                        self.candle.set_control(kShootingFlashAnim)
                        self.candle.set_control(kEmptyOne)
                else:
                    if self._setState(self.prepFlashState):
                        self.candle.set_control(kPrepFlashAnim)
                        self.candle.set_control(kEmptyOne)
            else:
                if RobotState.readyToShoot():
                    if self._setState(self.shootingState):
                        self.candle.set_control(kShootingAnim)
                        self.candle.set_control(kEmptyOne)
                        self.candle.set_control(kEmptyZero)
                else:
                    if self._setState(self.prepState):
                        self.candle.set_control(kPrepAnim)
                        self.candle.set_control(kEmptyOne)
                        self.candle.set_control(kEmptyZero)

        Logger.recordOutput("LED/lastEnabledAuto", self.lastEnabledAuto)
        Logger.recordOutput("LED/lastEnabledTime", self.lastEnabledTime)

        LogTracer.recordTotal()
