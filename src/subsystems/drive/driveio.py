from dataclasses import dataclass
from pykit.autolog import autolog


class DriveIO:
    """A dataclass for holding I/O data for the drive subsystem."""

    @autolog
    @dataclass
    class DriveIOInputs:
        connected: bool = False
        gyro_yaw_rad: float = 0.0
        gyro_yaw_rate_rad_per_sec: float = 0.0

    def updateInputs(self, inputs: DriveIOInputs) -> None:
        """Update the drive I/O inputs.

        Args:
            inputs (DriveIOInputs): The drive I/O inputs to update.
        """

    def setYaw(self, yaw_rad: float) -> None:
        """Set the gyro yaw.

        Args:
            yaw_rad (float): The yaw in radians.
        """
