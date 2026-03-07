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

kShootingMap = partial(interp, xp=[0, 5, 10, 15], fp=[0, 1000, 2000, 3000])
# Example mapping function for distance to flywheel speed,
# where xp is the distance in meters and fp is the
# corresponding flywheel speed in rad/s

kHoodAngleMap = lambda x: Rotation2d.fromDegrees(
    interp(x, xp=[0, 5, 10, 15], fp=[0, 15, 30, 45])
)
