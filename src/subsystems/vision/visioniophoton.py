from wpimath.geometry import Pose3d, Transform3d
from photonlibpy import PhotonCamera

from subsystems.vision.visionio import (
    ObservationType,
    VisionSubsystemIO,
    VisionSubsystemPoseObservation,
    VisionSubsystemTurretedPoseObservation,
)

from constants.vision import kApriltagFieldLayout


class VisionSubsystemIOPhotonVision(VisionSubsystemIO):
    def __init__(
        self, name: str, robotToCamera: Transform3d, isTurreted: bool = False
    ) -> None:
        """
        Initializes the VisionSubsystemIOPhotonVision with a PhotonCamera.
        Args:
            name (str): The name of the PhotonCamera.
            robotToCamera (Transform3d): The transform from the robot to the camera. Alternatively, if the camera is turreted, this is the transform from the turret base to the camera.
            isTurreted (bool): Whether the camera is turreted.
        """
        VisionSubsystemIO.__init__(self)
        self.camera = PhotonCamera(name)
        self.robotToCamera = robotToCamera
        self.isTurreted = isTurreted

    def updateCameraPosition(self, transform: Transform3d) -> None:
        self.robotToCamera = transform

    def updateInputs(self, inputs: VisionSubsystemIO.VisionSubsystemIOInputs):
        inputs.connected = self.camera.isConnected()
        tagIds = []
        poseObservations = []
        turretedObservations = []

        for result in self.camera.getAllUnreadResults():
            if result.multitagResult is not None:
                fieldToCamera = result.multitagResult.estimatedPose.best
                fieldToBase = fieldToCamera + self.robotToCamera.inverse()

                robotPose = Pose3d(fieldToBase.translation(), fieldToBase.rotation())

                totalTagDistance = 0.0
                for target in result.targets:
                    totalTagDistance += target.bestCameraToTarget.translation().norm()

                tagIds.extend(result.multitagResult.fiducialIDsUsed)

                idsCondensed = 0
                # this is stored in a 32 bit unsigned integer, this is OK since only 32 tags exist on the field
                for tagId in result.multitagResult.fiducialIDsUsed:
                    idsCondensed |= (
                        1 << tagId - 1
                    )  # condense the tag IDs into a single integer using bitwise OR. This allows us to store up to 32 tag IDs in a single integer, which is more efficient than storing a list of integers.

                if not self.isTurreted:
                    poseObservations.append(
                        VisionSubsystemPoseObservation(
                            result.getTimestampSeconds(),
                            robotPose,
                            result.multitagResult.estimatedPose.ambiguity,
                            len(result.multitagResult.fiducialIDsUsed),
                            idsCondensed,
                            totalTagDistance / len(result.targets),
                            ObservationType.PHOTONVISION.value,
                        )
                    )
                else:
                    turretedObservations.append(
                        VisionSubsystemTurretedPoseObservation(
                            result.getTimestampSeconds(),
                            fieldToBase,  # this transform is from field to turret
                            result.multitagResult.estimatedPose.ambiguity,
                            len(result.multitagResult.fiducialIDsUsed),
                            idsCondensed,
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
                    fieldToBase = fieldToCamera + self.robotToCamera.inverse()
                    robotPose = Pose3d(
                        fieldToBase.translation(), fieldToBase.rotation()
                    )

                    tagIds.append(target.fiducialId)

                    if not self.isTurreted:
                        poseObservations.append(
                            VisionSubsystemPoseObservation(
                                result.getTimestampSeconds(),
                                robotPose,
                                target.poseAmbiguity,
                                1,
                                target.fiducialId,
                                cameraToTarget.translation().norm(),
                                ObservationType.PHOTONVISION.value,
                            )
                        )
                    else:
                        turretedObservations.append(
                            VisionSubsystemTurretedPoseObservation(
                                result.getTimestampSeconds(),
                                fieldToBase,
                                target.poseAmbiguity,
                                1,
                                target.fiducialId,
                                cameraToTarget.translation().norm(),
                                ObservationType.PHOTONVISION.value,
                            )
                        )

        inputs.poseObservations = poseObservations
        inputs.tagIds = tagIds
        inputs.turretedObservations = turretedObservations
