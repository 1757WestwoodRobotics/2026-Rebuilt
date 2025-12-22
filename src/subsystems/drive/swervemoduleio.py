from dataclasses import dataclass
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d


@dataclass
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


class SwerveModuleIO:
    @autolog
    @dataclass
    class SwerveModuleIOInputs:
        driveconnected: bool = False
        steerconnected: bool = False
        encoderconnected: bool = False

        drive_position: float = 0.0  # rad
        drive_velocity: float = 0.0  # rad / sec
        drive_applied: float = 0.0  # volts
        drive_supply_current: float = 0.0  # amps
        drive_torque_current: float = 0.0  # amps

        turn_position: float = 0.0  # rad
        turn_velocity: float = 0.0  # rad / sec
        turn_applied: float = 0.0  # volts
        turn_supply_current: float = 0.0  # amps
        turn_torque_current: float = 0.0  # amps

        turn_absolute_position: float = 0.0  # rad

    def __init__(self, name: str) -> None:
        self.name = name

    def updateInputs(self, inputs: SwerveModuleIOInputs) -> None:
        """Update the swerve module I/O inputs.

        Args:
            inputs (SwerveModuleIOInputs): The swerve module I/O inputs to update.
        """

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        """Set the swerve module angle.

        Args:
            swerveAngle (Rotation2d): The desired swerve module angle.
        """

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        """Set the swerve module wheel linear velocity.

        Args:
            wheelLinearVelocity_m_per_s (float): The desired wheel linear velocity in meters per second.
        """

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        """Set the swerve module angle target.

        Args:
            swerveAngleTarget (Rotation2d): The desired swerve module angle target.
        """
