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

kAutoMaxFadeTime = 5.0
kAutoFadeDuration = 2.5  # nominally 3.0, reality is closer to 2.5 seconds in between

kEmptyOne = EmptyAnimation(1)

kCANdleTotalLedCount = kCANdleOnboardLedCount + kCANdleExternalLedCount

kDisabledAnim = RainbowAnimation(0, kCANdleTotalLedCount, 0)

# blue
kAutoOutColor = RGBWColor(0, 0, 255)

# red
kEstopAnim = StrobeAnimation(0, kCANdleTotalLedCount, 0, RGBWColor(255, 0, 0), 10)

# orange
kBrownoutColor = RGBWColor(255, 64, 0)
kBrownoutAnim = StrobeAnimation(0, kCANdleTotalLedCount, 0, kBrownoutColor, 10)

# yellow
kPrepColor = RGBWColor(255, 208, 0)
kPrepAnim = SolidColor(0, kCANdleTotalLedCount, kPrepColor)
kPrepFlashAnim = StrobeAnimation(0, kCANdleTotalLedCount, 0, kPrepColor, 10)

# green
kShootingColor = RGBWColor(0, 255, 0)
kShootingAnim = SolidColor(0, kCANdleTotalLedCount, kShootingColor)
kShootingFlashAnim = StrobeAnimation(0, kCANdleTotalLedCount, 0, kShootingColor, 10)
