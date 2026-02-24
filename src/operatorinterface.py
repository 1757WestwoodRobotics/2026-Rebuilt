from commands2 import Command, cmd
from commands2.button import CommandXboxController
from wpilib.interfaces import GenericHID
from util.helpfultriggerwrappers import Deadband, Invert, SignSquare
from util.joystick.commandfarmcontroller import CommandFarmController

from constants.oi import kXboxJoystickDeadband
from util.joystick.commandfarmcontroller import CommandFarmController


class OperatorInterface:
    """
    The controls that the operator(s)/driver(s) interact with
    """

    driverController: CommandXboxController
    operatorController: CommandFarmController

    def __init__(self) -> None:
        self.driverController = CommandXboxController(0)
        self.operatorController = CommandFarmController(1)

        self.driverX = SignSquare(
            Invert(Deadband(self.driverController.getLeftX, kXboxJoystickDeadband))
        )
        self.driverY = SignSquare(
            Invert(Deadband(self.driverController.getLeftY, kXboxJoystickDeadband))
        )
        self.driverRotation = Invert(
            Deadband(self.driverController.getRightX, kXboxJoystickDeadband)
        )

    def rumbleControllers(self, amount: float = 1.0) -> None:
        self.driverController.setRumble(GenericHID.RumbleType.kBothRumble, amount)
        self.operatorController.setRumble(GenericHID.RumbleType.kBothRumble, amount)

    def rumbleControllersCommand(self, amount: float = 1.0) -> Command:
        return cmd.startEnd(
            lambda: self.rumbleControllers(amount),
            lambda: self.rumbleControllers(0.0),
        )
