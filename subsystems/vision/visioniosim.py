from math import hypot, sin
from typing import Optional
from ntcore import NetworkTableInstance
from wpilib import Timer
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Translation3d,
)
import constants
from subsystems.drivesubsystem import VisionObservation
from subsystems.vision.visionio import VisionSubsystemIO
from util.convenientmath import pose3dFrom2d, clamp


class VisionSubsystemIOSim(VisionSubsystemIO):
    def __init__(self, name: str, location: Transform3d) -> None:
        VisionSubsystemIO.__init__(self)
        self.simBotPoseGetter = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kSimRobotPoseArrayKey, Pose2d)
            .subscribe(Pose2d())
        )
        self.camera = SimCamera(
            name,
            location,
            constants.kCameraFOVHorizontal,
            constants.kCameraFOVVertical,
            "ll",
        )
        self.rng = RNG(constants.kSimulationVariation)
        self.location = location

    def getRobotFieldPose(self) -> Optional[VisionObservation]:
        simPose = self.simBotPoseGetter.get()
        simPose3d = pose3dFrom2d(simPose)

        seeTag = False
        botPose = Pose3d()
        tagPoses: list[Transform3d] = []

        for _tagId, apriltag in constants.kApriltagPositionDict.items():
            if self.camera.canSeeTarget(simPose3d, apriltag):
                rngOffset = Transform3d(
                    Translation3d(
                        self.rng.getNormalRandom(),
                        self.rng.getNormalRandom(),
                        self.rng.getNormalRandom(),
                    ),
                    Rotation3d(),
                )
                botToTagPose = Pose3d() + Transform3d(simPose3d, apriltag)
                botToTagPose = (
                    botToTagPose + rngOffset * botToTagPose.translation().norm()
                )
                tagPoses.append(Transform3d(simPose3d + self.camera.location, apriltag))
                seeTag = True
                botPose = (
                    Pose3d(
                        simPose3d.X(),
                        simPose3d.Y(),
                        simPose3d.Z(),
                        simPose3d.rotation(),
                    )
                    + rngOffset
                )

        pose = Pose3d(
            clamp(botPose.X(), 0, constants.kFieldLength),
            clamp(botPose.Y(), 0, constants.kFieldWidth),
            botPose.Z(),
            botPose.rotation(),
        )
        return (
            VisionObservation(
                pose.toPose2d(), Timer.getFPGATimestamp(), [0.7, 0.7, 9999999]
            )
            if seeTag
            else None
        )

    def updateCameraPosition(self, transform: Transform3d) -> None:
        self.camera.location = transform

    def updateRobotYaw(self, _yaw: Rotation2d) -> None:
        pass  # doesn't matter

    def setLights(self, _lightVal: bool) -> None:
        pass  # doesn't matter


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
