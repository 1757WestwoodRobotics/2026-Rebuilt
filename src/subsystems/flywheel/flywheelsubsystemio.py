from dataclasses import dataclass

from pykit.autolog import autolog


class FlywheelSubsystemIO:

    @autolog
    @dataclass
    class FlywheelSubsystemIOInputs:
        flywheelConnected: bool = False

        flywheelSpeed: float = 0.0
        flywheelPosition: float = 0.0

        flywheelApplidVolts: float = 0.0
        flywheelSupplyAmps: float = 0.0

    def updateInputs(self, inputs: FlywheelSubsystemIOInputs):
        pass

    def set_speed(self, speed: float):
        # Code to set the shooter speed, in rad/s
        pass

    def set_volts(self, volts: float):
        # Code to set the shooter voltage
        pass

    def neutral_output(self):
        # Code to stop the shooter
        pass
