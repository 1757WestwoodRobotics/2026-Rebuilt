from phoenix6.configs.pigeon2_configs import Pigeon2Configuration
from phoenix6.hardware.core.core_talon_fx import BaseStatusSignal
from phoenix6.hardware.pigeon2 import Pigeon2
from subsystems.drive.driveio import DriveIO

from constants.drive import kPigeonCANId, kCANivoreName
from constants.math import kRadiansPerDegree
from constants import kRobotUpdateFrequency
from util.phoenixutil import PhoenixUtil, tryUntilOk


class DriveIOPigeon(DriveIO):
    """A dataclass for holding I/O data for the drive subsystem using a Pigeon IMU."""

    gyroConfig: Pigeon2Configuration = Pigeon2Configuration()

    def __init__(self) -> None:
        super().__init__()

        print("Starting Pigeon")
        self.gyro = Pigeon2(kPigeonCANId, kCANivoreName)
        tryUntilOk(5, lambda: self.gyro.configurator.apply(self.gyroConfig, 0.25))
        tryUntilOk(5, lambda: self.gyro.configurator.set_yaw(0, 0.25))
        self.yaw_position = self.gyro.get_yaw()
        self.yaw_velocity = self.gyro.get_angular_velocity_z_world()

        BaseStatusSignal.set_update_frequency_for_all(
            kRobotUpdateFrequency, self.yaw_position, self.yaw_velocity
        )

        PhoenixUtil.registerSignals(kCANivoreName, self.yaw_position, self.yaw_velocity)

        self.gyro.optimize_bus_utilization()
        print("Finished Pigeon")

    def updateInputs(self, inputs: DriveIO.DriveIOInputs) -> None:
        """Update the drive I/O inputs.

        Args:
            inputs (DriveIOInputs): The drive I/O inputs to update.
        """
        inputs.connected = BaseStatusSignal.is_all_good(
            self.yaw_velocity, self.yaw_position
        )
        inputs.gyro_yaw_rad = self.yaw_position.value * kRadiansPerDegree
        inputs.gyro_yaw_rate_rad_per_sec = self.yaw_velocity.value * kRadiansPerDegree

    def setYaw(self, yaw_rad: float) -> None:
        self.gyro.set_yaw(yaw_rad / kRadiansPerDegree)
