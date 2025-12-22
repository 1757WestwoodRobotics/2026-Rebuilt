from pykit.logger import Logger
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from subsystems.drive.swervemoduleio import SwerveModuleIO

from constants.drive import kMinWheelLinearVelocity, kWheelRadius
from util.logtracer import LogTracer


class SwerveModule:
    def __init__(self, name: str, io: SwerveModuleIO) -> None:
        self.name = name
        self.io = io
        self.inputs = SwerveModuleIO.SwerveModuleIOInputs()
        self.prevPosition = SwerveModulePosition()

    def periodic(self) -> None:
        LogTracer.resetOuter("SwerveModule/" + self.name)
        self.prevPosition = self.getPosition()
        LogTracer.record("GetPosition")
        self.io.updateInputs(self.inputs)
        LogTracer.record("UpdateInputs")
        Logger.processInputs("Drive/Module" + self.name, self.inputs)
        LogTracer.record("ProcessInputs")
        LogTracer.recordTotal()

    def getSwerveAngle(self) -> Rotation2d:
        return Rotation2d(self.inputs.turn_position)

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        self.io.setSwerveAngle(swerveAngle)

    def getSwerveEncoderAngle(self) -> Rotation2d:
        return Rotation2d(self.inputs.turn_absolute_position)

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        self.io.setSwerveAngleTarget(swerveAngleTarget)

    def getWheelLinearVelocity(self) -> float:
        return self.inputs.drive_velocity * kWheelRadius

    def getWheelTotalPosition(self) -> float:
        return self.inputs.drive_position * kWheelRadius

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        self.io.setWheelLinearVelocityTarget(wheelLinearVelocityTarget / kWheelRadius)

    def reset(self) -> None:
        self.setSwerveAngle(self.getSwerveEncoderAngle())

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
            self.setSwerveAngleTarget(state.angle)
