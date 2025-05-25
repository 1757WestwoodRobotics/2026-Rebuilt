from typing import Optional
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Transform3d, Pose2d
from subsystems.drivesubsystem import VisionObservation
from subsystems.vision.visionio import VisionSubsystemIO

from util.convenientmath import clamp
import constants


class VisionSubsystemIOLimelight(VisionSubsystemIO):
    def __init__(self, name: str, transform: Transform3d) -> None:
        VisionSubsystemIO.__init__(self)
        self.location = transform
        self.cameraTable = NetworkTableInstance.getDefault().getTable(name)
        self.validTarget = self.cameraTable.getIntegerTopic("tv").subscribe(0)
        self.pipelineLatency = self.cameraTable.getIntegerTopic("tl").subscribe(0)
        self.captureLatency = self.cameraTable.getIntegerTopic("cl").subscribe(0)
        self.ledState = self.cameraTable.getDoubleTopic("ledMode").publish()
        self.ledState.set(1)
        self.botpose = self.cameraTable.getDoubleArrayTopic(
            "botpose_orb_wpiblue"
        ).subscribe([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        self.camPoseSetter = self.cameraTable.getDoubleArrayTopic(
            "camerapose_robotspace_set"
        ).publish()
        self.robotOrientationSetter = self.cameraTable.getDoubleArrayTopic(
            "robot_orientation_set"
        ).publish()
        self.robotPoseGetter = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kRobotPoseArrayKeys.valueKey, Pose2d)
            .subscribe(Pose2d())
        )

    def getRobotFieldPose(self) -> Optional[VisionObservation]:
        if self.validTarget.get() == 0:
            return None
        botPose = self.botpose.get()
        poseX, poseY, poseZ = botPose[0:3]
        rotation = self.robotPoseGetter.get().rotation().radians()
        pose = Pose3d(
            clamp(poseX, 0, constants.kFieldLength),
            clamp(poseY, 0, constants.kFieldWidth),
            poseZ,
            Rotation3d(0, 0, rotation),
        )

        # this section is taken from the secret limelight docs called the limelight lib source code
        # if you want the actual stuff look at https://github.com/LimelightVision/limelightlib-wpijava/blob/cbb84564b10316aaa64440c0f43db64d45539fe5/LimelightHelpers.java#L738
        tsValue = self.botpose.getAtomic()
        timestamp = tsValue.time

        adjustedTimestamp = (
            timestamp / 1e6 - (0 if len(botPose) < 7 else botPose[6]) / 1e4
        )

        return VisionObservation(
            pose.toPose2d(), adjustedTimestamp, [0.7, 0.7, 9999999]
        )

    def updateCameraPosition(self, transform: Transform3d) -> None:
        self.camPoseSetter.set(
            [
                transform.X(),
                -transform.Y(),
                transform.Z(),
                transform.rotation().X() / constants.kRadiansPerDegree,
                -transform.rotation().Y() / constants.kRadiansPerDegree,
                transform.rotation().Z() / constants.kRadiansPerDegree,
            ]
        )

    def updateRobotYaw(self, yaw: Rotation2d) -> None:
        self.robotOrientationSetter.set([yaw.degrees(), 0, 0, 0, 0, 0])

    def setLights(self, lightVal: bool) -> None:
        if lightVal:
            self.ledState.set(3)
        else:
            self.ledState.set(1)
