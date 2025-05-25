from phoenix6.hardware.cancoder import CANcoder
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.sim.cancoder_sim_state import CANcoderSimState
from wpilib import DataLogManager
from wpimath.geometry import Rotation2d

import constants


class CTREEncoder:
    def __init__(self, canId: int, offset: float, canbus: str = "") -> None:
        self.encoder = CANcoder(canId, canbus)
        self.offset = offset

        # not sure if this change is functionally the same
        config = CANcoderConfiguration().with_magnet_sensor(
            MagnetSensorConfigs()
            .with_absolute_sensor_discontinuity_point(0.5)
            .with_magnet_offset(-1 * self.offset)
        )
        DataLogManager.log(f"Encoder {canId} initialized")
        self.encoder.configurator.apply(config)

    def getDeviceNumber(self) -> int:
        return self.encoder.device_id

    def getPosition(self) -> Rotation2d:
        return Rotation2d(
            self.encoder.get_position().value * constants.kRadiansPerRevolution
        )

    def getSim(self) -> CANcoderSimState:
        return self.encoder.sim_state
