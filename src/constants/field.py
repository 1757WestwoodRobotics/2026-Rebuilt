from wpimath.geometry import Translation2d
from .math import kMetersPerInch, kMetersPerFoot

# Field physical parameters
kFieldLength = 54 * kMetersPerFoot + 3.2 * kMetersPerInch
"""meters"""

kFieldWidth = 26 * kMetersPerFoot + 5.7 * kMetersPerInch
"""meters"""

kBlueTargetLocation = Translation2d(
    kFieldWidth / 2, kFieldLength / 3.5761
)  # Must be fixed

kRedTargetLocation = Translation2d(
    kFieldWidth / 2, kFieldLength / 3.5761
)  # Must be fixed
"""Location of targets on field.  Best current guess for 2026"""
