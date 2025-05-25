from typing import Optional
from wpimath.geometry import Rotation2d, Transform3d

from subsystems.drivesubsystem import VisionObservation


class VisionSubsystemIO:
    def updateCameraPosition(self, transform: Transform3d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def getRobotFieldPose(self) -> Optional[VisionObservation]:
        raise NotImplementedError("Must be implemented by subclass")

    def updateRobotYaw(self, yaw: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def setLights(self, lightVal: bool) -> None:
        raise NotImplementedError("Must be implemented by subclass")
