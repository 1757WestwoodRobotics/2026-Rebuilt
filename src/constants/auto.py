from wpimath.geometry import Translation2d
from .drive import (
    kWheelCircumference,
)
from .math import kMetersPerInch

# Autonomous
kAutoDriveDistance = -8 * kWheelCircumference
"""meters"""

kAutoFrontwaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoSidewaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoDistanceThreshold = 6 * kMetersPerInch
"""meters"""

kAutoDriveSpeedFactor = 0.5
"""dimensionless"""

kAutoWaitDuration = 1
"""seconds"""

kAutoTargetOffset = Translation2d(2, 0)
"""[meters, meters]"""

kAutoDistanceTolerance = 0.1
"""meters"""

kAutoRotationTolerance = 0.1
"""radians"""
