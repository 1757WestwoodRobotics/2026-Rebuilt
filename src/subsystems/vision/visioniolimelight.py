from typing import Callable
from ntcore import NetworkTableInstance
from wpilib import RobotController
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Transform3d
from subsystems.vision.visionio import (
    ObservationType,
    VisionSubsystemIO,
    VisionSubsystemPoseObservation,
)


from constants.math import kRadiansPerDegree


class VisionSubsystemIOLimelight(VisionSubsystemIO):
    def __init__(
        self,
        name: str,
        transform: Transform3d,
        rotationSupplier: Callable[[], Rotation2d],
    ) -> None:
        VisionSubsystemIO.__init__(self)
        self.location = transform
        self.cameraTable = NetworkTableInstance.getDefault().getTable(name)
        self.rotationSupplier = rotationSupplier
        self.validTarget = self.cameraTable.getIntegerTopic("tv").subscribe(0)
        self.latencySubscriber = self.cameraTable.getIntegerTopic("tl").subscribe(0)
        self.ledState = self.cameraTable.getDoubleTopic("ledMode").publish()
        self.ledState.set(1)

        self.megatag1 = self.cameraTable.getDoubleArrayTopic(
            "botpose_wpiblue"
        ).subscribe([])

        self.megatag2 = self.cameraTable.getDoubleArrayTopic(
            "botpose_orb_wpiblue"
        ).subscribe([])

        self.camPoseSetter = self.cameraTable.getDoubleArrayTopic(
            "camerapose_robotspace_set"
        ).publish()
        self.orientationPublisher = self.cameraTable.getDoubleArrayTopic(
            "robot_orientation_set"
        ).publish()

        self.updateCameraPosition(self.location)

    def updateInputs(self, inputs: VisionSubsystemIO.VisionSubsystemIOInputs):
        inputs.connected = (
            (RobotController.getFPGATime() - self.latencySubscriber.getLastChange())
            / 1000
        ) < 250
        self.orientationPublisher.set(
            [self.rotationSupplier().degrees(), 0, 0, 0, 0, 0]
        )

        NetworkTableInstance.getDefault().flush()  # inefficient on usage but good for LL

        tagIds = []
        poseObservatios = []

        for sample in self.megatag1.readQueue():
            if len(sample.value) == 0:
                continue
            for i in range(11, len(sample.value), 7):
                tagIds.append(int(sample.value[i]))
                poseObservatios.append(
                    VisionSubsystemPoseObservation(
                        sample.time * 1e-6 - sample.value[6] * 1e-3,
                        VisionSubsystemIOLimelight.parsePose(sample.value),
                        sample.value[17] if len(sample.value) >= 18 else 0,
                        int(sample.value[7]),
                        sample.value[9],
                        ObservationType.MEGATAG_1.value,
                    )
                )

        for sample in self.megatag2.readQueue():
            if len(sample.value) == 0:
                continue
            for i in range(11, len(sample.value), 7):
                tagIds.append(int(sample.value[i]))
                poseObservatios.append(
                    VisionSubsystemPoseObservation(
                        sample.time * 1e-6 - sample.value[6] * 1e-3,
                        VisionSubsystemIOLimelight.parsePose(sample.value),
                        0.0,
                        int(sample.value[7]),
                        sample.value[9],
                        ObservationType.MEGATAG_2.value,
                    )
                )

        inputs.poseObservations = poseObservatios
        inputs.tagIds = tagIds

    @staticmethod
    def parsePose(rawLLArray: list[float]):
        return Pose3d(
            rawLLArray[0],
            rawLLArray[1],
            rawLLArray[2],
            Rotation3d(
                rawLLArray[3] / kRadiansPerDegree,
                rawLLArray[4] / kRadiansPerDegree,
                rawLLArray[5] / kRadiansPerDegree,
            ),
        )

    def updateCameraPosition(self, transform: Transform3d) -> None:
        self.camPoseSetter.set(
            [
                transform.X(),
                -transform.Y(),
                transform.Z(),
                transform.rotation().X() / kRadiansPerDegree,
                -transform.rotation().Y() / kRadiansPerDegree,
                transform.rotation().Z() / kRadiansPerDegree,
            ]
        )

    def setLights(self, lightVal: bool) -> None:
        if lightVal:
            self.ledState.set(3)
        else:
            self.ledState.set(1)
