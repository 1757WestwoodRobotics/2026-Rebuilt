import typing
from commands2 import Command
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController
from wpimath.kinematics import DifferentialDriveKinematics, DifferentialDriveWheelSpeeds

from subsystems.drive.drivesubsystem import DriveSubsystem

from constants.drive import (
    kDrivePGain,
    kDriveIGain,
    kDriveDGain,
    kMaxForwardLinearVelocity,
    kMaxForwardLinearAcceleration,
    kSwerveModuleCenterToRobotCenterWidth,
)


class ControlledMotor:
    """
    you shouldn't need to construct this class
    it is simply a wrapper for TankDrive command
    """

    def __init__(self, control: typing.Callable[[], float]) -> None:
        self.control = lambda: control() * kMaxForwardLinearVelocity**2 * 2
        self.pid = ProfiledPIDController(
            kDrivePGain,
            kDriveIGain,
            kDriveDGain,
            TrapezoidProfile.Constraints(
                kMaxForwardLinearVelocity,
                kMaxForwardLinearAcceleration,
            ),
        )

    def __call__(self) -> float:
        return self.pid.calculate(self.control())


class TankDrive(Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        left: typing.Callable[[], float],
        right: typing.Callable[[], float],
    ) -> None:
        Command.__init__(self)
        self.setName(type(self).__name__)

        self.drivetrain = DifferentialDriveKinematics(
            kSwerveModuleCenterToRobotCenterWidth * 2
        )

        self.drive = drive
        self.left = ControlledMotor(left)
        self.right = ControlledMotor(right)

        self.addRequirements(self.drive)

    def execute(self) -> None:
        l = -self.left()
        r = -self.right()

        target_pos = self.drivetrain.toChassisSpeeds(DifferentialDriveWheelSpeeds(l, r))

        self.drive.arcadeDriveWithSpeeds(
            target_pos,
            DriveSubsystem.CoordinateMode.RobotRelative,
        )
