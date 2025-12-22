from wpimath.geometry import Pose3d, Transform3d
from photonlibpy import PhotonCamera

from subsystems.vision.visionio import (
    ObservationType,
    VisionSubsystemIO,
    VisionSubsystemPoseObservation,
)

from constants.vision import kApriltagFieldLayout


class VisionSubsystemIOPhotonVision(VisionSubsystemIO):
    def __init__(self, name: str, robotToCamera: Transform3d) -> None:
        VisionSubsystemIO.__init__(self)
        self.camera = PhotonCamera(name)
        self.robotToCamera = robotToCamera

    def updateCameraPosition(self, transform: Transform3d) -> None:
        self.robotToCamera = transform

    def updateInputs(self, inputs: VisionSubsystemIO.VisionSubsystemIOInputs):
        inputs.connected = self.camera.isConnected()
        tagIds = []
        poseObservations = []

        for result in self.camera.getAllUnreadResults():
            if result.multitagResult is not None:
                fieldToCamera = result.multitagResult.estimatedPose.best
                fieldToRobot = fieldToCamera + self.robotToCamera.inverse()

                robotPose = Pose3d(fieldToRobot.translation(), fieldToRobot.rotation())

                totalTagDistance = 0.0
                for target in result.targets:
                    totalTagDistance += target.bestCameraToTarget.translation().norm()

                tagIds.extend(result.multitagResult.fiducialIDsUsed)

                poseObservations.append(
                    VisionSubsystemPoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        result.multitagResult.estimatedPose.ambiguity,
                        len(result.multitagResult.fiducialIDsUsed),
                        totalTagDistance / len(result.targets),
                        ObservationType.PHOTONVISION.value,
                    )
                )
            elif len(result.targets) > 0:
                target = result.targets[0]

                tagPose = kApriltagFieldLayout.getTagPose(target.fiducialId)
                if tagPose is not None:
                    fieldToTarget = Transform3d(
                        tagPose.translation(), tagPose.rotation()
                    )
                    cameraToTarget = target.bestCameraToTarget
                    fieldToCamera = fieldToTarget + cameraToTarget.inverse()
                    fieldToRobot = fieldToCamera + self.robotToCamera.inverse()
                    robotPose = Pose3d(
                        fieldToRobot.translation(), fieldToRobot.rotation()
                    )

                    tagIds.append(target.fiducialId)

                    poseObservations.append(
                        VisionSubsystemPoseObservation(
                            result.getTimestampSeconds(),
                            robotPose,
                            target.poseAmbiguity,
                            1,
                            cameraToTarget.translation().norm(),
                            ObservationType.PHOTONVISION.value,
                        )
                    )
        inputs.poseObservations = poseObservations
        inputs.tagIds = tagIds
