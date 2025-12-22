from math import pi
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from constants.field import kFieldWidth, kFieldLength


class FlipUtil:
    """
    Utility for flipping wpilib geometry classes based on current alliance
    """

    @staticmethod
    def shouldFlip() -> bool:
        """
        Determine whether to flip based on current alliance

        :return: True if should flip, False otherwise
        """
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    @staticmethod
    def fieldPose(pose: Pose2d) -> Pose2d:
        """
        Flip a Pose2d based on current alliance

        :param pose: The Pose2d to potentially flip
        :return: The flipped or original Pose2d
        """
        return Pose2d(
            FlipUtil.fieldTranslation(pose.translation()),
            FlipUtil.fieldRotation(pose.rotation()),
        )

    @staticmethod
    def fieldTranslation(translation: Translation2d) -> Translation2d:
        """
        Flip a Translation2d based on current alliance

        :param translation: The Translation2d to potentially flip
        :return: The flipped or original Translation2d
        """
        if FlipUtil.shouldFlip():
            return Translation2d(kFieldLength, kFieldWidth) - translation
        return translation

    @staticmethod
    def fieldRotation(rotation: Rotation2d) -> Rotation2d:
        """
        Flip a Rotation2d based on current alliance

        :param rotation: The Rotation2d to potentially flip
        :return: The flipped or original Rotation2d
        """
        if FlipUtil.shouldFlip():
            return rotation - Rotation2d(pi)
        return rotation
