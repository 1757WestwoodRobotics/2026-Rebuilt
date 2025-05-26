import constants
from util.controltype import ControlAxis, ControlButton
from util.helpfultriggerwrappers import Deadband, Invert, SignSquare


class OperatorInterface:
    """
    The controls that the operator(s)/driver(s) interact with
    """

    class Drive:
        reset_gyro = ControlButton(0, 16)
        defense_state = ControlButton(2, 24)
        align_angle = ControlButton(1, 3)
        auto_waypoint = ControlButton(1, 4)

        class ChassisControls:
            class Translation:
                y = SignSquare(
                    Invert(
                        Deadband(
                            ControlAxis(0, 1)(),  # Joystick vertical
                            constants.kXboxJoystickDeadband,
                        )
                    )
                )
                x = SignSquare(
                    Invert(
                        Deadband(
                            ControlAxis(0, 0)(),  # Joystick horizontal
                            constants.kXboxJoystickDeadband,
                        )
                    )
                )

            class Rotation:
                y = Invert(
                    Deadband(
                        ControlAxis(1, 1)(),  # Joystick vertical
                        constants.kXboxJoystickDeadband,
                    )
                )
                x = Invert(
                    Deadband(
                        ControlAxis(1, 0)(),  # Joystick horizontal
                        constants.kXboxJoystickDeadband,
                    )
                )
