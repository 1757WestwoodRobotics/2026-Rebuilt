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

kSampleDistances = [
    1.24,
    1.71,
    1.93,
    2.11,
    2.71,
    2.97,
    3.30,
    3.8,
    5.0,
    6.0,
]  # in meters

kShootingMap = partial(
    interp,
    xp=kSampleDistances,
    fp=[200, 230, 230, 230, 245, 250, 260, 280, 300, 320],
)
# Example mapping function for distance to flywheel speed,
# where xp is the distance in meters and fp is the
# corresponding flywheel speed in rad/s

kHoodAngleMap = lambda x: Rotation2d.fromDegrees(
    interp(
        x,
        xp=kSampleDistances,
        fp=[12, 15, 17.5, 20, 21, 23, 24, 25, 30, 35],
    )
)

kFeedSampleDistances = [*kSampleDistances, 8.0, 10.0]

kFeedFlywheelMap = partial(
    interp,
    xp=kFeedSampleDistances,
    fp=[110, 135, 145, 150, 175, 185, 195, 220, 250, 274, 320, 360],
)  # rad / s

kFeedShotTimeMap = partial(
    interp,
    xp=kFeedSampleDistances,
    fp=[0.59, 0.66, 0.69, 0.73, 0.81, 0.84, 0.88, 0.95, 1.04, 1.14, 1.30, 1.45],
)

kShotTimeMap = partial(
    interp,
    xp=kSampleDistances,
    fp=[0.78, 0.787, 0.796, 0.792, 0.907, 0.911, 0.940, 1.02, 1.04, 1.05],
)
# Example mapping function for distance to shot time, where xp is the distance in
# meters and fp is the corresponding shot time in seconds

kSOTMIterations = 1
