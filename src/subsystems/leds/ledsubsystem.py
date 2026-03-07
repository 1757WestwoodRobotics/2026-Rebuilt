from typing import Optional, Union
from commands2 import Subsystem
from phoenix6.controls.empty_animation import EmptyAnimation
from phoenix6.controls.solid_color import RGBWColor, SolidColor
from phoenix6.hardware.candle import CANdle
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

        desired_control_2: Union[EmptyAnimation, SolidColor] = (
            kEmptyOne  # prep a slot 1 empty animation
        )
        if DriverStation.isEStopped():
            desired_control = kEstopAnim
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
                desired_control = SolidColor(
                    0, int(percent * kCANdleTotalLedCount), kAutoOutColor
                )
                desired_control_2 = SolidColor(
                    int(percent * kCANdleTotalLedCount),
                    kCANdleTotalLedCount,
                    RGBWColor(0, 0, 0),
                )
            else:
                if RobotState.brownoutFlag:
                    desired_control = kBrownoutAnim
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

        if desired_control_2 is not self._last_control_2:  # slot 1
            self.candle.set_control(desired_control_2)
            self._last_control_2 = desired_control_2
        LogTracer.recordTotal()
