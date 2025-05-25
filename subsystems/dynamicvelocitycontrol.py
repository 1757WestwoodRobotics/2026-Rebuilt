from enum import Enum, auto
from ntcore import NetworkTableInstance
from commands2.subsystem import Subsystem

import constants
from util.simtalon import Talon


class VelocityControl(Subsystem):
    class ControlState(Enum):
        Setpoint1 = auto()
        Setpoint2 = auto()
        Off = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.setpoint1 = NetworkTableInstance.getDefault().getDoubleTopic(
            constants.kVelocitySetpoint1ControlKey
        )
        self.setpoint2 = NetworkTableInstance.getDefault().getDoubleTopic(
            constants.kVelocitySetpoint2ControlKey
        )
        self.gearratio = NetworkTableInstance.getDefault().getDoubleTopic(
            constants.kVelocityControlGearRatio
        )

        self.setpoint1.publish().set(0)
        self.setpoint2.publish().set(0)
        self.gearratio.publish().set(1)

        self.setpoint1getter = self.setpoint1.subscribe(0)
        self.setpoint2getter = self.setpoint2.subscribe(0)
        self.gearratiogetter = self.gearratio.subscribe(1)

        self.motor = Talon(
            constants.kVelocityControlCANId,
            "Velocity Control",
            constants.kVelocityControlPGain,
            constants.kVelocityControlIGain,
            constants.kVelocityControlDGain,
            False,
            constants.kCANivoreName,
            constants.kVelocityControlkV,
        )

        self.state = VelocityControl.ControlState.Off

    def periodic(self) -> None:
        if self.state == VelocityControl.ControlState.Off:
            self.motor.neutralOutput()
        elif self.state == VelocityControl.ControlState.Setpoint1:
            self.motor.set(
                Talon.ControlMode.Velocity,
                self.setpoint1getter.get() / self.gearratiogetter.get(),
            )
        elif self.state == VelocityControl.ControlState.Setpoint2:
            self.motor.set(
                Talon.ControlMode.Velocity,
                self.setpoint2getter.get() / self.gearratiogetter.get(),
            )

    def setState(self, state: ControlState) -> None:
        self.state = state
