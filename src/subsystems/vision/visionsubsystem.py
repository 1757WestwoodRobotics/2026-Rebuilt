from typing import Callable, List
from commands2 import Subsystem
from pykit.logger import Logger

from subsystems.drive.robotposeestimator import VisionObservation
from subsystems.vision.visionio import VisionSubsystemIO

from constants.vision import (
    kApriltagFieldLayout,
    kMaxVisionZError,
    kMaxVisionAmbiguity,
    kXyStdDevCoeff,
    kThetaStdDevCoeff,
)
from util.logtracer import LogTracer


class VisionSubsystem(Subsystem):
    def __init__(
        self,
        visionConsumer: Callable[[VisionObservation], None],
        io: List[VisionSubsystemIO],
    ) -> None:
        self.consumer = visionConsumer
        self.io = io

        self.inputs: list[VisionSubsystemIO.VisionSubsystemIOInputs] = []
        for _ in io:
            self.inputs.append(VisionSubsystemIO.VisionSubsystemIOInputs())

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

        LogTracer.reset()
        for idx, camera in enumerate(self.inputs):
            tagPoses = []
            robotPoses = []
            robotPosesAccepted = []
            robotPosesRejected = []

            for tagId in camera.tagIds:
                tagPose = kApriltagFieldLayout.getTagPose(tagId)
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
                    or observation.pose.X() > kApriltagFieldLayout.getFieldLength()
                    or observation.pose.Y() < 0.0
                    or observation.pose.Y() > kApriltagFieldLayout.getFieldWidth()
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

                # here you can also factor in per-camera weighting

                self.consumer(
                    VisionObservation(
                        observation.pose.toPose2d(),
                        observation.timestamp,
                        [linearStdDev, linearStdDev, angularStdDev],
                    )
                )
            LogTracer.record(f"Camera{idx} ProcessObservations")

            Logger.recordOutput(f"Vision/Camera{idx}/TagPose", tagPoses)
            Logger.recordOutput(f"Vision/Camera{idx}/RobotPoses", robotPoses)
            Logger.recordOutput(
                f"Vision/Camera{idx}/RobotPosesRejected", robotPosesRejected
            )
            Logger.recordOutput(
                f"Vision/Camera{idx}/RobotPosesAccepted", robotPosesAccepted
            )
            allTagPoses.extend(tagPoses)
            allRobotPoses.extend(robotPoses)
            allRobotPosesAccepted.extend(robotPosesAccepted)
            allRobotPosesRejected.extend(robotPosesRejected)

        Logger.recordOutput("Vision/Summary/TagPose", allTagPoses)
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses)
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected)
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted)
        LogTracer.recordTotal()
