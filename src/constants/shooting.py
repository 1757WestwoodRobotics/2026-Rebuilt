"""
This module is for all the functions related to higher order shooting operations
This should include any mapping functions for distance,
compensation factors for shooting on the move, and any other functions that are used to calculate
the parameters for shooting commands, such as flywheel speed or hood angle, based on various inputs
such as distance to target, robot velocity, etc.
"""

from functools import partial

from numpy import interp
from wpimath.geometry import Rotation2d

kShootingMap = partial(
    interp,
    xp=[1.39, 1.86, 2.08, 2.26, 2.86, 3.12, 3.45],
    fp=[200, 230, 230, 230, 245, 250, 260],
)
# Example mapping function for distance to flywheel speed,
# where xp is the distance in meters and fp is the
# corresponding flywheel speed in rad/s

kHoodAngleMap = lambda x: Rotation2d.fromDegrees(
    interp(
        x,
        xp=[1.39, 1.86, 2.08, 2.26, 2.86, 3.12, 3.45],
        fp=[12, 15, 17.5, 20, 21, 23, 24],
    )
)

kFeedFlywheelMap = partial(interp, xp=[0, 5, 10], fp=[5, 40, 80])  # rad / s
