from functools import reduce
from operator import add

from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import PowerDistribution, DriverStation, SmartDashboard

from wpimath.geometry import (
    Pose2d,
    Transform3d,
    Rotation3d,
    Pose3d,
)
from util.convenientmath import pose3dFrom2d
from operatorinterface import OperatorInterface

import constants


class LoggingSubsystem(Subsystem):
    def __init__(self, oi: OperatorInterface) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        SmartDashboard.putData("PDH", self.pdh)
        self.oi = oi
        self.dsTable = NetworkTableInstance.getDefault().getTable(
            constants.kJoystickKeyLogPrefix
        )

        self.robotPoseGetter = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kRobotPoseArrayKeys.valueKey, Pose2d)
            .subscribe(Pose2d())
        )
        self.elevatorPositionGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kElevatorPositionKey)
            .subscribe(0)
        )
        self.elevatorPosePublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic(constants.kElevatorPoseKey, Pose3d)
            .publish()
        )
        self.pivotAngleGetter = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(constants.kPivotAngleKey)
            .subscribe(0)
        )
        self.intakePosesPublisher = (
            NetworkTableInstance.getDefault()
            .getStructArrayTopic("intake/poses", Pose3d)
            .publish()
        )

    def updateBotPositions(self) -> None:
        botPose = pose3dFrom2d(self.robotPoseGetter.get(Pose2d()))
        elevatorHeight = self.elevatorPositionGetter.get(0)
        elevatorPosition = (
            botPose
            + constants.kRobotToElevatorTransform
            + Transform3d(0, 0, elevatorHeight, Rotation3d())
        )
        self.elevatorPosePublisher.set(elevatorPosition)
        armRotation = -self.pivotAngleGetter.get()
        armRootPosition = elevatorPosition + Transform3d(
            0,
            -constants.kPivotToArmRoot,
            0,
            Rotation3d(0, armRotation, 0),
        )
        armEndPosition = armRootPosition + constants.kArmRootToArmEndTransform
        self.intakePosesPublisher.set([armRootPosition, armEndPosition])

    def periodic(self) -> None:

        for controller in self.oi.controllers.values():
            self.dsTable.putNumber(
                f"Joystick{controller.getPort()}/ButtonCount",
                controller.getButtonCount(),
            )
            encodedButtonValue = reduce(
                add,
                [
                    controller.getRawButton(i) << i - 1
                    for i in range(controller.getButtonCount() + 1, 0, -1)
                ],
                0,
            )
            self.dsTable.putNumber(
                f"Joystick{controller.getPort()}/ButtonValues", encodedButtonValue
            )
            self.dsTable.putNumberArray(
                f"Joystick{controller.getPort()}/AxisValues",
                [controller.getRawAxis(i) for i in range(controller.getAxisCount())],
            )
            self.dsTable.putNumberArray(
                f"Joystick{controller.getPort()}/POVs",
                [controller.getPOV(i) for i in range(controller.getPOVCount())],
            )

        self.dsTable.putBoolean("Enabled", DriverStation.isEnabled())
        self.dsTable.putBoolean("auto", DriverStation.isAutonomous())
        alliance = DriverStation.getAlliance()
        allianceNumber = (
            0
            if alliance is None
            else (1 if alliance == DriverStation.Alliance.kRed else 2)
        )
        self.dsTable.putNumber("AllianceStation", allianceNumber)
        station = DriverStation.getLocation()
        self.dsTable.putNumber("location", 0.0 if station is None else station)

        self.updateBotPositions()
