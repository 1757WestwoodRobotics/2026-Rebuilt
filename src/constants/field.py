from wpimath.geometry import Translation2d
from .math import kMetersPerInch, kMetersPerFoot

# Field physical parameters
kFieldLength = 54 * kMetersPerFoot + 3.25 * kMetersPerInch
"""meters"""

kFieldWidth = 26 * kMetersPerFoot + 3.5 * kMetersPerInch
"""meters"""

kTargetLocation = Translation2d(kFieldWidth / 2, kFieldLength / 2)
"""Sets target in center of field.  Presumbly, this needs to be adjusted for 2026."""
