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

        # Cache tag poses at init to avoid repeated field layout lookups in the hot loop
        self._tagPoseCache: dict[int, Pose3d | None] = {
            i: kApriltagFieldLayout.getTagPose(i) for i in range(1, 23)
        }

    def updateCameraPosition(self, transform: Transform3d) -> None:
        self.robotToCamera = transform

    def updateInputs(self, inputs: VisionSubsystemIO.VisionSubsystemIOInputs):
        inputs.connected = self.camera.isConnected()

        # Skip getAllUnreadResults() when camera is disconnected.
        # This avoids the expensive _versionCheck() inside photonlibpy which
        # generates full Python stack traces via wpilib.reportWarning() every 5 seconds,
        # each costing significant CPU time on the roboRIO.
        if not inputs.connected:
            inputs.poseObservations = []
            inputs.tagIds = []
            inputs.turretedObservations = []
            return

        tagIds = []
        poseObservations = []
        turretedObservations = []

        # Limit to last 2 results per cycle instead of 10.
        # PhotonVision runs at 30-90 FPS while the robot loop is 50 Hz,
        # so results accumulate. Processing 10 results means 10x the
        # Transform3d math per cycle. The most recent 2 are sufficient
        # since older results have increasingly stale timestamps.
        allResults = self.camera.getAllUnreadResults()
        lastResults = allResults[-2:]
        for result in lastResults:
            if result.multitagResult is not None:
                fieldToCamera = result.multitagResult.estimatedPose.best
                fieldToBase = fieldToCamera + self.robotToCamera.inverse()

                robotPose = Pose3d(fieldToBase.translation(), fieldToBase.rotation())

                totalTagDistance = 0.0
                for target in result.targets:
                    totalTagDistance += target.bestCameraToTarget.translation().norm()

                tagIds.extend(result.multitagResult.fiducialIDsUsed)

                idsCondensed = 0
                for tagId in result.multitagResult.fiducialIDsUsed:
                    idsCondensed |= 1 << (tagId - 1)

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

                tagPose = self._tagPoseCache.get(target.fiducialId)
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

                    # Encode as bitmask: tag ID 1 -> bit 0, tag ID 2 -> bit 1, etc.
                    tagBitmask = 1 << (target.fiducialId - 1)

                    if not self.isTurreted:
                        poseObservations.append(
                            VisionSubsystemPoseObservation(
                                result.getTimestampSeconds(),
                                robotPose,
                                target.poseAmbiguity,
                                1,
                                tagBitmask,
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
                                tagBitmask,
                                cameraToTarget.translation().norm(),
                                ObservationType.PHOTONVISION.value,
                            )
                        )

        inputs.poseObservations = poseObservations
        inputs.tagIds = tagIds
        inputs.turretedObservations = turretedObservations
