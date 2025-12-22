from typing import Callable
from commands2 import Command, cmd
from commands2.button import Trigger
from util.controltype import ControlAxis, ControlBase, ControlButton
from util.helpfultriggerwrappers import Deadband, Invert, SignSquare

from constants.oi import kXboxJoystickDeadband

button = Callable[[], Trigger]


class OperatorInterface:
    """
    The controls that the operator(s)/driver(s) interact with
    """

    class Drive:
        reset_gyro = ControlButton(0, 3)
        defense_state = ControlButton(0, 2)
        align_angle = ControlButton(1, 3)

        class ChassisControls:
            class Translation:
                y = SignSquare(
                    Invert(
                        Deadband(
                            ControlAxis(0, 1)(),  # Joystick vertical
                            kXboxJoystickDeadband,
                        )
                    )
                )
                x = SignSquare(
                    Invert(
                        Deadband(
                            ControlAxis(0, 0)(),  # Joystick horizontal
                            kXboxJoystickDeadband,
                        )
                    )
                )

            class Rotation:
                y = Invert(
                    Deadband(
                        ControlAxis(0, 5)(),  # Joystick vertical
                        kXboxJoystickDeadband,
                    )
                )
                x = Invert(
                    Deadband(
                        ControlAxis(0, 4)(),  # Joystick horizontal
                        kXboxJoystickDeadband,
                    )
                )

    @staticmethod
    def rumbleControllers(amount: float = 1.0) -> Command:
        return cmd.startEnd(
            lambda: ControlBase.rumbleControllers(amount),
            lambda: ControlBase.rumbleControllers(0.0),
        )
