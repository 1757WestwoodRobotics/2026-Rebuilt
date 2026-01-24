from wpimath.geometry import Translation2d
from .math import kMetersPerInch, kMetersPerFoot

# Field physical parameters
kFieldLength = 54 * kMetersPerFoot + 3.2 * kMetersPerInch
"""meters"""

kFieldWidth = 26 * kMetersPerFoot + 5.7 * kMetersPerInch
"""meters"""

kBlueTargetLocation = Translation2d(
    kFieldWidth / 3.5761, kFieldLength / 2
)  # Location of Blue target (flipped when necessary)
kAutoDuration = 20
"""seconds"""

kEndgameDuration = 30
"""seconds"""

kTeleopDuration = 140
"""seconds"""

kMatchDuration = kAutoDuration + kTeleopDuration
"""seconds"""

kTransitionShiftDuration = 10
"""seconds"""

kShiftDuration = 25
"""seconds"""
