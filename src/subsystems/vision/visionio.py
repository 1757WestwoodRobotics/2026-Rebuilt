from dataclasses import dataclass, field
from enum import Enum
from typing import List
from pykit.autolog import autolog
from wpimath.geometry import Pose3d, Transform3d
from wpiutil.wpistruct import make_wpistruct


class ObservationType(Enum):
    MEGATAG_1 = 0
    MEGATAG_2 = 1
    PHOTONVISION = 2


@make_wpistruct(name="visionobservation")
@autolog
@dataclass
class VisionSubsystemPoseObservation:
    timestamp: float = 0
    pose: Pose3d = field(default_factory=Pose3d)
    ambiguity: float = 0
    tagCount: int = 0
    averageTagDistance: float = 0
    observationType: int = ObservationType.PHOTONVISION.value


@make_wpistruct(name="turretedobservation")
@autolog
@dataclass
class VisionSubsystemTurretedPoseObservation:
    timestamp: float = 0
    fieldToTurret: Transform3d = field(default_factory=Transform3d)
    ambiguity: float = 0
    tagCount: int = 0
    averageTagDistance: float = 0
    observationType: int = ObservationType.PHOTONVISION.value


class VisionSubsystemIO:

    @autolog
    @dataclass
    class VisionSubsystemIOInputs:
        connected: bool = False
        poseObservations: list[VisionSubsystemPoseObservation] = field(
            default_factory=list
        )
        turretedObservations: list[VisionSubsystemTurretedPoseObservation] = field(
            default_factory=list
        )
        tagIds: List[int] = field(default_factory=list)

    def updateInputs(self, inputs: VisionSubsystemIOInputs):
        pass

    def updateCameraPosition(self, transform: Transform3d) -> None:
        raise NotImplementedError("Must be implemented by subclass")
