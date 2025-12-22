from math import tau, floor, pi
from wpimath.geometry import Rotation2d


def optimizeAngle(currentAngle: Rotation2d, targetAngle: Rotation2d) -> Rotation2d:
    currentAngleRad = currentAngle.radians()
    targetAngleRad = targetAngle.radians()

    deltaAngle = targetAngleRad - currentAngleRad
    deltaAngle = deltaAngle - tau * floor((deltaAngle + pi) / tau)

    return Rotation2d(deltaAngle + currentAngleRad)
