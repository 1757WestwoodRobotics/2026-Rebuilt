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
    xp=[
        1.24,
        1.71,
        1.93,
        2.11,
        2.71,
        2.97,
        3.30,
    ],
    fp=[200, 230, 230, 230, 245, 250, 260],
)
# Example mapping function for distance to flywheel speed,
# where xp is the distance in meters and fp is the
# corresponding flywheel speed in rad/s

kHoodAngleMap = lambda x: Rotation2d.fromDegrees(
    interp(
        x,
        xp=[
            1.24,
            1.71,
            1.93,
            2.11,
            2.71,
            2.97,
            3.30,
        ],
        fp=[12, 15, 17.5, 20, 21, 23, 24],
    )
)

kFeedFlywheelMap = partial(interp, xp=[0, 5, 10], fp=[5, 40, 80])  # rad / s

kShotTimeMap = partial(
    interp,
    xp=[1.24, 1.71, 1.93, 2.11, 2.71, 2.97, 3.30],
    fp=[0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
)
# Example mapping function for distance to shot time, where xp is the distance in
# meters and fp is the corresponding shot time in seconds

kSOTMIterations = 5
