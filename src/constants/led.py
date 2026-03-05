from phoenix6.controls import (
    EmptyAnimation,
    RainbowAnimation,
    SolidColor,
    StrobeAnimation,
)
from phoenix6.signals.rgbw_color import RGBWColor

kCANdleCANId = 21
kCANdleOnboardLedCount = 8
kCANdleExternalLedCount = 64

kEmpty0Anim = EmptyAnimation(0)
kEmpty1Anim = EmptyAnimation(1)

kDisabledAnim = RainbowAnimation(0, kCANdleOnboardLedCount + kCANdleExternalLedCount, 0)

# red
kEstopAnim = StrobeAnimation(
    0, kCANdleOnboardLedCount + kCANdleExternalLedCount, 0, RGBWColor(255, 0, 0), 10
)

# yellow
kPrepColor = RGBWColor(255, 208, 0)
kPrepAnim = SolidColor(0, kCANdleOnboardLedCount + kCANdleExternalLedCount, kPrepColor)
kPrepFlashAnim = StrobeAnimation(
    0, kCANdleOnboardLedCount + kCANdleExternalLedCount, 0, kPrepColor, 10
)

# green
kShootingColor = RGBWColor(0, 255, 0)
kShootingAnim = SolidColor(
    0, kCANdleOnboardLedCount + kCANdleExternalLedCount, kShootingColor
)
kShootingFlashAnim = StrobeAnimation(
    0, kCANdleOnboardLedCount + kCANdleExternalLedCount, 0, kShootingColor, 10
)
