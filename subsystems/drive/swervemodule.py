from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from util.angleoptimize import optimizeAngle

from constants.drive import kMinWheelLinearVelocity


class SwerveModuleConfigParams:
    swerveEncoderOffset: float
    swerveEncoderID: int
    driveMotorID: int
    driveMotorInverted: bool
    steerMotorID: int
    steerMotorInverted: bool
    canbus: str = ""

    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
        self,
        driveMotorID: int,
        driveMotorInverted: bool,
        steerMotorID: int,
        steerMotorInverted: bool,
        swerveEncoderID: int,
        swerveEncoderOffset: float,
        canbus: str = "",
    ) -> None:
        self.driveMotorID = driveMotorID
        self.driveMotorInverted = driveMotorInverted
        self.steerMotorID = steerMotorID
        self.steerMotorInverted = steerMotorInverted
        self.swerveEncoderID = swerveEncoderID
        self.swerveEncoderOffset = swerveEncoderOffset
        self.canbus = canbus


class SwerveModule:
    def __init__(self, name: str) -> None:
        self.name = name

    def getSwerveAngle(self) -> Rotation2d:
        raise NotImplementedError("Must be implemented by subclass")

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def getWheelLinearVelocity(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def getWheelTotalPosition(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def reset(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def optimizedAngle(self, targetAngle: Rotation2d) -> Rotation2d:
        return optimizeAngle(self.getSwerveAngle(), targetAngle)

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.getWheelTotalPosition(), self.getSwerveAngle())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.getWheelLinearVelocity(),
            self.getSwerveAngle(),
        )

    def applyState(self, state: SwerveModuleState) -> None:
        state.optimize(self.getSwerveAngle())

        self.setWheelLinearVelocityTarget(state.speed)
        if (
            abs(state.speed) >= kMinWheelLinearVelocity
        ):  # prevent unneccisary movement for what would otherwise not move the robot
            optimizedAngle = self.optimizedAngle(state.angle)
            self.setSwerveAngleTarget(optimizedAngle)
