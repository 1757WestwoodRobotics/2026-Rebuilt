from typing import Callable, List
from commands2 import Subsystem
from pykit.logger import Logger

from subsystems.vision.visionio import VisionSubsystemIO

from constants.vision import (
    kApriltagFieldLayout,
    kMaxVisionZError,
    kMaxVisionAmbiguity,
    kXyStdDevCoeff,
    kThetaStdDevCoeff,
)
from constants.turret import kTurretLocation
from util.convenientmath import pose3dFromTransform3d
from util.logtracer import LogTracer
from util.robotposeestimator import TurretedVisionObservation, VisionObservation


# Number of April tags on the field — limits bitmask iteration
_MAX_TAG_ID = 22


class VisionSubsystem(Subsystem):
    def __init__(
        self,
        visionConsumer: Callable[[VisionObservation], None],
        turretedVisionConsumer: Callable[[TurretedVisionObservation], None],
        io: List[VisionSubsystemIO],
    ) -> None:
        self.consumer = visionConsumer
        self.turretedConsumer = turretedVisionConsumer
        self.io = io

        self.inputs: list[VisionSubsystemIO.VisionSubsystemIOInputs] = []
        for _ in io:
            self.inputs.append(VisionSubsystemIO.VisionSubsystemIOInputs())

        # Cache tag poses at init to avoid repeated field layout lookups
        self._tagPoseCache: dict[int, object] = {}
        for i in range(1, _MAX_TAG_ID + 1):
            pose = kApriltagFieldLayout.getTagPose(i)
            if pose is not None:
                self._tagPoseCache[i] = pose

        # Cache field bounds to avoid method calls in the hot loop
        self._fieldLength = kApriltagFieldLayout.getFieldLength()
        self._fieldWidth = kApriltagFieldLayout.getFieldWidth()

    # pylint:disable-next=too-many-locals, too-many-statements, too-many-branches
    def periodic(self) -> None:
        LogTracer.resetOuter("VisionSubsystem")
        for idx, (i, inp) in enumerate(zip(self.io, self.inputs)):
            i.updateInputs(inp)
            Logger.processInputs(f"Vision/Camera{idx}", self.inputs[idx])
            LogTracer.record(f"Camera{idx} UpdateInputs")
        LogTracer.record("All Cameras UpdateInputs")

        allTagPoses = []
        allRobotPoses = []
        allRobotPosesAccepted = []
        allRobotPosesRejected = []

        allTurretedTransforms = []
        allTurretedTransformsRejected = []
        allTurretedTransformsAccepted = []

        LogTracer.reset()
        for idx, camera in enumerate(self.inputs):
            tagPoses = []
            robotPoses = []
            robotPosesAccepted = []
            robotPosesRejected = []

            turretedTransforms = []
            turretedTransformsAccepted = []
            turretedTransformsRejected = []

            for tagId in camera.tagIds:
                tagPose = self._tagPoseCache.get(tagId)
                if tagPose is not None:
                    tagPoses.append(tagPose)

            for observation in camera.poseObservations:
                rejectPose = (
                    observation.tagCount == 0
                    or (
                        observation.tagCount == 1
                        and observation.ambiguity > kMaxVisionAmbiguity
                    )
                    or abs(observation.pose.Z()) > kMaxVisionZError
                    or observation.pose.X() < 0.0
                    or observation.pose.X() > self._fieldLength
                    or observation.pose.Y() < 0.0
                    or observation.pose.Y() > self._fieldWidth
                )

                robotPoses.append(observation.pose)
                if rejectPose:
                    robotPosesRejected.append(observation.pose)
                else:
                    robotPosesAccepted.append(observation.pose)

                if rejectPose:
                    continue

                stdDevFactor = (
                    pow(observation.averageTagDistance, 2.0) / observation.tagCount
                )
                linearStdDev = kXyStdDevCoeff * stdDevFactor
                angularStdDev = kThetaStdDevCoeff * stdDevFactor

                # Extract tag IDs from bitmask — only iterate up to _MAX_TAG_ID
                observedTags = []
                tagsList = observation.tagsList
                for tagId in range(_MAX_TAG_ID):
                    if tagsList & (1 << tagId):
                        observedTags.append(
                            tagId + 1
                        )  # add 1 since tag IDs are 1-indexed but bitmask is 0-indexed

                self.consumer(
                    VisionObservation(
                        observation.pose.toPose2d(),
                        observation.timestamp,
                        [linearStdDev, linearStdDev, angularStdDev],
                        observedTags,
                    )
                )
            LogTracer.record(f"Camera{idx} ProcessObservations")
            for observation in camera.turretedObservations:
                rejectPose = (
                    observation.tagCount == 0
                    or (
                        observation.tagCount == 1
                        and observation.ambiguity > kMaxVisionAmbiguity
                    )
                    or abs(
                        (observation.fieldToTurret + kTurretLocation.inverse())
                        .translation()
                        .Z()
                    )
                    > kMaxVisionZError  # work backwards onto what the robot pose would be
                    or observation.fieldToTurret.X() < 0.0
                    or observation.fieldToTurret.X() > self._fieldLength
                    or observation.fieldToTurret.Y() < 0.0
                    or observation.fieldToTurret.Y() > self._fieldWidth
                )
                turretPose = pose3dFromTransform3d(observation.fieldToTurret)
                turretedTransforms.append(turretPose)
                if rejectPose:
                    turretedTransformsRejected.append(turretPose)
                else:
                    turretedTransformsAccepted.append(turretPose)

                if rejectPose:
                    continue

                stdDevFactor = (
                    pow(observation.averageTagDistance, 2.0) / observation.tagCount
                )
                linearStdDev = kXyStdDevCoeff * stdDevFactor
                angularStdDev = kThetaStdDevCoeff * stdDevFactor

                # Extract tag IDs from bitmask — only iterate up to _MAX_TAG_ID
                observedTags = []
                tagsList = observation.tagsList
                for tagId in range(_MAX_TAG_ID):
                    if tagsList & (1 << tagId):
                        observedTags.append(tagId + 1)

                self.turretedConsumer(
                    TurretedVisionObservation(
                        observation.fieldToTurret,
                        observation.timestamp,
                        [linearStdDev, linearStdDev, angularStdDev],
                        observedTags,
                    )
                )

            # Only log per-camera data when there are observations to report
            if len(tagPoses) > 0:
                Logger.recordOutput(f"Vision/Camera{idx}/TagPose", tagPoses)
            if len(robotPoses) > 0:
                Logger.recordOutput(f"Vision/Camera{idx}/RobotPoses", robotPoses)
                Logger.recordOutput(
                    f"Vision/Camera{idx}/RobotPosesRejected", robotPosesRejected
                )
                Logger.recordOutput(
                    f"Vision/Camera{idx}/RobotPosesAccepted", robotPosesAccepted
                )
            if len(turretedTransforms) > 0:
                Logger.recordOutput(
                    f"Vision/Camera{idx}/TurretedTransforms", turretedTransforms
                )
                Logger.recordOutput(
                    f"Vision/Camera{idx}/TurretedTransformsRejected",
                    turretedTransformsRejected,
                )
                Logger.recordOutput(
                    f"Vision/Camera{idx}/TurretedTransformsAccepted",
                    turretedTransformsAccepted,
                )
            allTagPoses.extend(tagPoses)
            allRobotPoses.extend(robotPoses)
            allRobotPosesAccepted.extend(robotPosesAccepted)
            allRobotPosesRejected.extend(robotPosesRejected)
            allTurretedTransforms.extend(turretedTransforms)
            allTurretedTransformsAccepted.extend(turretedTransformsAccepted)
            allTurretedTransformsRejected.extend(turretedTransformsRejected)
        LogTracer.record("All Cameras ProcessObservations")

        # Only log summary data when there are observations
        if len(allTagPoses) > 0:
            Logger.recordOutput("Vision/Summary/TagPose", allTagPoses)
        if len(allRobotPoses) > 0:
            Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses)
            Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected", allRobotPosesRejected
            )
            Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted
            )
        if len(allTurretedTransforms) > 0:
            Logger.recordOutput(
                "Vision/Summary/TurretedTransforms", allTurretedTransforms
            )
            Logger.recordOutput(
                "Vision/Summary/TurretedTransformsRejected",
                allTurretedTransformsRejected,
            )
            Logger.recordOutput(
                "Vision/Summary/TurretedTransformsAccepted",
                allTurretedTransformsAccepted,
            )
        LogTracer.recordTotal()
