from typing import Optional

# import numpy as np

from commands2 import Subsystem
from ntcore import BooleanPublisher, NetworkTableInstance, StructPublisher
from wpilib import RobotBase
from wpimath.geometry import Pose2d, Pose3d

from subsystems.drive.drivesubsystem import (
    DriveSubsystem,
)
from subsystems.drive.robotposeestimator import RobotPoseEstimator
from subsystems.vision.visionio import VisionSubsystemIO
from subsystems.vision.visioniolimelight import VisionSubsystemIOLimelight
from subsystems.vision.visioniosim import VisionSubsystemIOSim
from util.convenientmath import pose3dFrom2d

from constants.drive import kRobotPoseArrayKeys
from constants.vision import (
    kRobotVisionPose1ArrayKeys,
    kRobotVisionPose2ArrayKeys,
    kCameraLocationPublisherKey,
    kRobotToCamera1Transform,
    kRobotToCamera2Transform,
)
from constants.trajectory import kWaypointAtTargetKey


class VisionSubsystem(Subsystem):
    camera1: VisionSubsystemIO
    camera2: VisionSubsystemIO

    def __init__(self, drive: DriveSubsystem) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.poseReceiver = (
            NetworkTableInstance.getDefault()
            .getStructTopic(kRobotPoseArrayKeys.valueKey, Pose2d)
            .subscribe(Pose2d())
        )

        self.vision1PosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(kRobotVisionPose1ArrayKeys.valueKey, Pose2d)
            .publish()
        )
        self.vision2PosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(kRobotVisionPose2ArrayKeys.valueKey, Pose2d)
            .publish()
        )
        self.visionPose1ValidPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(kRobotVisionPose2ArrayKeys.validKey)
            .publish()
        )
        self.visionPose2ValidPublisher = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(kRobotVisionPose2ArrayKeys.validKey)
            .publish()
        )

        self.cameraPosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic(kCameraLocationPublisherKey, Pose3d)
            .publish()
        )
        self.atPositionIndicator = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(kWaypointAtTargetKey)
            .subscribe(False)
        )

        if RobotBase.isReal():
            self.camera1 = VisionSubsystemIOLimelight(
                "limelight-br", kRobotToCamera1Transform
            )
            self.camera2 = VisionSubsystemIOLimelight(
                "limelight-fl", kRobotToCamera2Transform
            )
        else:
            self.camera1 = VisionSubsystemIOSim("limelight1", kRobotToCamera1Transform)
            self.camera2 = VisionSubsystemIOSim("limelight2", kRobotToCamera2Transform)

        self.camera1.updateCameraPosition(kRobotToCamera1Transform)
        self.camera2.updateCameraPosition(kRobotToCamera2Transform)

    def periodic(self) -> None:
        yaw = self.poseReceiver.get().rotation()
        self.camera1.updateRobotYaw(yaw)
        self.camera2.updateRobotYaw(yaw)

        atPosition = self.atPositionIndicator.get()

        self.camera1.setLights(atPosition)
        self.camera2.setLights(atPosition)

        cameraPoses = [
            VisionSubsystem.processCamera(
                self.camera1,
                self.vision1PosePublisher,
                self.visionPose1ValidPublisher,
                self.drive.estimator,
            ),
            VisionSubsystem.processCamera(
                self.camera2,
                self.vision2PosePublisher,
                self.visionPose2ValidPublisher,
                self.drive.estimator,
            ),
        ]

        self.cameraPosePublisher.set(list(filter(lambda x: x is not None, cameraPoses)))

    @staticmethod
    def processCamera(
        camera: VisionSubsystemIO,
        posePublisher: StructPublisher,
        poseValidPublisher: BooleanPublisher,
        driveEstimator: RobotPoseEstimator,
    ) -> Optional[Pose3d]:
        visionPose = camera.getRobotFieldPose()

        cameraPose = None

        if visionPose is not None:
            cameraPose = pose3dFrom2d(visionPose.visionPose) + camera.location
            posePublisher.set(visionPose.visionPose)
            poseValidPublisher.set(True)

            driveEstimator.addVisionMeasurement(visionPose)
        else:
            visionPose = None
            poseValidPublisher.set(False)

        return cameraPose
