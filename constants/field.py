from wpimath.geometry import Translation2d
from .math import kMetersPerInch, kMetersPerFoot

# Field Physical parameters
kFieldLength = 54 * kMetersPerFoot + 3.25 * kMetersPerInch
"""meters"""

kFieldWidth = 26 * kMetersPerFoot + 3.5 * kMetersPerInch
"""meters"""

kTargetLocation = Translation2d(kFieldWidth / 2, kFieldLength / 2)
