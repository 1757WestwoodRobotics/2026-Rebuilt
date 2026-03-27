from math import pi

from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Transform3d, Translation3d
from constants.math import kRadiansPerDegree
from constants.turret import kTurretLocation
from util.convenientmath import map_range


class RobotMechanism:
    @staticmethod
    def getPoses(turretRotation: Rotation2d, intakePivot: Rotation2d) -> list[Pose3d]:
        # Order is turret, bin wall, pivot
        intakeRotation = -intakePivot.radians()
        return [
            Pose3d()
            + kTurretLocation
            + Transform3d(
                Translation3d(), Rotation3d(0, 0, turretRotation.radians() - pi / 2)
            ),
            Pose3d(
                map_range(intakeRotation, 0, -125.5 * kRadiansPerDegree, 0.298, 0),
                0,
                0,
                Rotation3d(),
            ),
            Pose3d(0.252, 0, 0.184, Rotation3d(0, intakeRotation, 0)),
        ]
