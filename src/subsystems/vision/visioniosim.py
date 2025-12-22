from math import hypot, sin
from typing import Callable
from wpilib import RobotController, Timer
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Translation3d,
)
from subsystems.vision.visionio import (
    ObservationType,
    VisionSubsystemIO,
    VisionSubsystemPoseObservation,
)
from util.convenientmath import pose3dFrom2d
from constants.vision import (
    kCameraFOVVertical,
    kCameraFOVHorizontal,
    kSimulationVariation,
    kApriltagPositionDict,
)


class VisionSubsystemIOSim(VisionSubsystemIO):
    def __init__(
        self, name: str, robotToCamera: Transform3d, poseSupplier: Callable[[], Pose2d]
    ) -> None:
        VisionSubsystemIO.__init__(self)
        self.camera = SimCamera(
            name,
            robotToCamera,
            kCameraFOVHorizontal,
            kCameraFOVVertical,
            "ll",
        )
        self.poseSupplier = poseSupplier
        self.rng = RNG(kSimulationVariation)
        self.location = robotToCamera

    def updateInputs(self, inputs: VisionSubsystemIO.VisionSubsystemIOInputs):
        inputs.connected = True

        simPose = self.poseSupplier()
        simPose3d = pose3dFrom2d(simPose)

        botPose = Pose3d()
        tagPoses: list[Transform3d] = []
        tagIds: list[int] = []
        poseObservations: list[VisionSubsystemPoseObservation] = []

        for tagId, apriltag in kApriltagPositionDict.items():
            if self.camera.canSeeTarget(simPose3d, apriltag):
                botToTagPose = Pose3d() + Transform3d(simPose3d, apriltag)
                avgDist = botToTagPose.translation().norm()
                rngOffset = Transform3d(
                    Translation3d(
                        self.rng.getNormalRandom() * avgDist * avgDist,
                        self.rng.getNormalRandom() * avgDist * avgDist,
                        self.rng.getNormalRandom() * avgDist * avgDist,
                    ),
                    Rotation3d(),
                )
                botToTagPose = (
                    botToTagPose + rngOffset * botToTagPose.translation().norm()
                )
                tagPoses.append(Transform3d(simPose3d + self.camera.location, apriltag))
                tagIds.append(tagId)
                botPose = (
                    Pose3d(
                        simPose3d.X(),
                        simPose3d.Y(),
                        simPose3d.Z(),
                        simPose3d.rotation(),
                    )
                    + rngOffset
                )
                avgDist = botToTagPose.translation().norm()
                poseObservations.append(
                    VisionSubsystemPoseObservation(
                        RobotController.getFPGATime() / 1e6,
                        botPose,
                        0.1,
                        1,
                        avgDist,
                        ObservationType.PHOTONVISION.value,
                    )
                )

        inputs.tagIds = tagIds
        inputs.poseObservations = poseObservations

    def updateCameraPosition(self, transform: Transform3d) -> None:
        self.camera.location = transform


class SimCamera:
    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        name: str,
        location: Transform3d,
        horizFOV: float,
        vertFOV: float,
        key: str,
    ) -> None:
        self.name = name
        self.location = location
        self.horizFOV = horizFOV
        self.vertFOV = vertFOV
        self.key = key

    def canSeeTarget(self, botPose: Pose3d, targetPose: Pose3d):
        cameraPose = botPose + self.location
        rel = CameraTargetRelation(cameraPose, targetPose)
        return (
            abs(rel.camToTargYaw.degrees()) < self.horizFOV / 2
            and abs(rel.camToTargPitch.degrees()) < self.vertFOV / 2
            and abs(rel.targToCamAngle.degrees()) < 90
        )


class CameraTargetRelation:
    def __init__(self, cameraPose: Pose3d, targetPose: Pose3d) -> None:
        self.cameraPose = cameraPose
        self.camToTarg = Transform3d(cameraPose, targetPose)
        self.camToTargDist = self.camToTarg.translation().norm()
        self.camToTargDistXY = hypot(
            self.camToTarg.translation().X(), self.camToTarg.translation().Y()
        )
        self.camToTargYaw = Rotation2d(self.camToTarg.X(), self.camToTarg.Y())
        self.camToTargPitch = Rotation2d(self.camToTargDistXY, -self.camToTarg.Z())
        self.camToTargAngle = Rotation2d(
            hypot(self.camToTargYaw.radians(), self.camToTargPitch.radians())
        )

        self.targToCam = Transform3d(targetPose, cameraPose)
        self.targToCamYaw = Rotation2d(self.targToCam.X(), self.targToCam.Y())
        self.targToCamPitch = Rotation2d(self.camToTargDistXY, -self.targToCam.Z())
        self.targToCamAngle = Rotation2d(
            hypot(self.targToCamYaw.radians(), self.targToCamPitch.radians())
        )


class RNG:
    def __init__(self, stdDev: float) -> None:
        self.stdDev = stdDev
        # self.rng = np.random.normal(0, stdDev, number)
        # self.rngIdx = 0

    def getNormalRandom(self) -> float:
        return sin(1000000 * Timer.getFPGATimestamp()) * self.stdDev
