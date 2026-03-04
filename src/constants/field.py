from wpimath.geometry import Translation2d
from .math import kMetersPerInch, kMetersPerFoot

# Field physical parameters
kFieldLength = 54 * kMetersPerFoot + 3.2 * kMetersPerInch
"""meters"""

kFieldWidth = 26 * kMetersPerFoot + 5.7 * kMetersPerInch
"""meters"""

kHubHeight = 1.8288
"""meters"""

kCloseHubLocation = Translation2d(
    kFieldLength / 3.5761, kFieldWidth / 2
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

kFarHubLocation = Translation2d(kFieldLength, kFieldWidth) - kCloseHubLocation


"""
Trench related parameters
"""

kTrenchWidth = 50.65 * kMetersPerInch
"""meters"""

kTrenchDepth = 47.0 * kMetersPerInch
"""meters"""

kTrenchHeight = 22.25 * kMetersPerInch
"""meters"""

kLeftCloseTrenchOpeningLeft = Translation2d(kCloseHubLocation.x, kFieldWidth)

kLeftCloseTrenchOpeningRight = Translation2d(
    kCloseHubLocation.x, kFieldWidth - kTrenchWidth
)
kLeftCloseTrenchCenter = Translation2d(
    kCloseHubLocation.x, kFieldWidth - kTrenchWidth / 2
)

kRightCloseTrenchOpeningLeft = Translation2d(kCloseHubLocation.x, kTrenchWidth)
kRightCloseTrenchOpeningRight = Translation2d(kCloseHubLocation.x, 0)

kRightCloseTrenchCenter = Translation2d(kCloseHubLocation.x, kTrenchWidth / 2)

kLeftFarTrenchOpeningLeft = Translation2d(
    kFarHubLocation.x, kFieldWidth
)  # Far trench is mirrored across field center
kLeftFarTrenchOpeningRight = Translation2d(
    kFarHubLocation.x, kFieldWidth - kTrenchWidth
)  # Far trench is mirrored across field center
kLeftFarTrenchCenter = Translation2d(
    kFarHubLocation.x, kFieldWidth - kTrenchWidth / 2
)  # Far trench is mirrored across field center
kRightFarTrenchOpeningLeft = Translation2d(kFarHubLocation.x, kTrenchWidth)
kRightFarTrenchOpeningRight = Translation2d(kFarHubLocation.x, 0)
kRightFarTrenchCenter = Translation2d(kFarHubLocation.x, kTrenchWidth / 2)

kTrenchCenters = [
    kLeftCloseTrenchCenter,
    kRightCloseTrenchCenter,
    kLeftFarTrenchCenter,
    kRightFarTrenchCenter,
]
