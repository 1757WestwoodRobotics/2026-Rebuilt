from collections.abc import Callable
from typing import Optional
from wpimath.geometry import Pose2d, Transform3d
from photonlibpy.simulation import SimCameraProperties, PhotonCameraSim, VisionSystemSim

from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniophoton import VisionSubsystemIOPhotonVision

from constants.vision import kApriltagFieldLayout


class VisionSubsystemIOPhotonSim(VisionSubsystemIOPhotonVision):
    visionSim: Optional[VisionSystemSim] = None

    def __init__(
        self, name: str, robotToCamera: Transform3d, poseSupplier: Callable[[], Pose2d]
    ) -> None:
        VisionSubsystemIOPhotonVision.__init__(self, name, robotToCamera)
        self.poseSupplier = poseSupplier

        if VisionSubsystemIOPhotonSim.visionSim is None:
            VisionSubsystemIOPhotonSim.visionSim = VisionSystemSim("main")
            VisionSubsystemIOPhotonSim.visionSim.addAprilTags(kApriltagFieldLayout)

        cameraProperties = SimCameraProperties.OV9281_1280_720()
        self.cameraSim = PhotonCameraSim(
            self.camera, cameraProperties, kApriltagFieldLayout
        )

        VisionSubsystemIOPhotonSim.visionSim.addCamera(self.cameraSim, robotToCamera)

    def updateInputs(self, inputs: VisionSubsystemIO.VisionSubsystemIOInputs):
        assert VisionSubsystemIOPhotonSim.visionSim is not None
        VisionSubsystemIOPhotonSim.visionSim.update(self.poseSupplier())
        super().updateInputs(inputs)
