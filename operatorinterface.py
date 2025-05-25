from math import copysign
import typing
import json

from os import path
from wpilib import Joystick, DataLogManager, Preferences

import constants
from util.convenientmath import map_range, number

AnalogInput = typing.Callable[[], float]


def Deadband(inputFn: AnalogInput, deadband: float) -> AnalogInput:
    def withDeadband() -> float:
        value = inputFn()
        if abs(value) <= deadband:
            return 0
        else:
            return value

    return withDeadband


def Invert(inputFn: AnalogInput) -> AnalogInput:
    def invert() -> float:
        return -1 * inputFn()

    return invert


def SignSquare(inputFn: AnalogInput) -> AnalogInput:
    def square() -> float:
        val = inputFn()
        return copysign(val * val, val)

    return square


def MapRange(
    inputFn: AnalogInput,
    inputMin: number,
    inputMax: number,
    outputMin: number,
    outputMax: number,
) -> AnalogInput:
    return lambda: map_range(inputFn(), inputMin, inputMax, outputMin, outputMax)


def Multiply(a: AnalogInput, b: AnalogInput) -> AnalogInput:
    return lambda: a() * b()


class HolonomicInput:
    def __init__(
        self,
        forwardsBackwards: AnalogInput,
        sideToSide: AnalogInput,
        rotationX: AnalogInput,
        rotationY: AnalogInput,
    ) -> None:
        self.forwardsBackwards = forwardsBackwards
        self.sideToSide = sideToSide
        self.rotationX = rotationX
        self.rotationY = rotationY


class Control2D:
    def __init__(self, forwardsBackwards: AnalogInput, sideToSide: AnalogInput) -> None:
        self.forwardsBackwards = forwardsBackwards
        self.sideToSide = sideToSide


# pylint:disable-next=too-many-instance-attributes
class OperatorInterface:
    """
    The controls that the operator(s)/driver(s) interact with
    """

    # pylint:disable-next=too-many-instance-attributes
    # pylint:disable-next=too-many-statements
    def __init__(self) -> None:
        with open(
            path.join(
                path.dirname(path.realpath(__file__)),
                constants.kControllerMappingFilename,
            ),
            "r",
            encoding="utf-8",
        ) as file:
            controlScheme = json.load(file)

        controllerNumbers = set(
            i[0] for i in controlScheme.values()
        )  # set ensures no duplicates
        DataLogManager.log(f"Looking for controllers: {controllerNumbers} ...")

        self.controllers = {}

        self.prefs = Preferences

        for control in controlScheme:
            binding = controlScheme[control]
            self.prefs.setInt(f"OI/{control}/controller", binding[0])
            if "Button" in binding[1].keys():
                self.prefs.setInt(f"OI/{control}/button", binding[1]["Button"])
            elif "Axis" in binding[1].keys():
                self.prefs.setInt(f"OI/{control}/axis", binding[1]["Axis"])
            elif "POV" in binding[1].keys():
                self.prefs.setInt(f"OI/{control}/angle", binding[1]["POV"][1])
                self.prefs.setInt(f"OI/{control}/pov", binding[1]["POV"][0])

        for num in controllerNumbers:
            controller = Joystick(num)
            DataLogManager.log(
                f"Found Controller {num}:{controller.getName()}\n\tAxis: {controller.getAxisCount()}\n\tButtons: {controller.getButtonCount()}\n\tPoV Hats: {controller.getPOVCount()}"
            )
            self.controllers[num] = controller

        def getButtonBindingOfName(
            name: str,
        ) -> typing.Callable[[], typing.Tuple[Joystick, int]]:
            return lambda: (
                self.controllers[self.prefs.getInt(f"OI/{name}/controller")],
                self.prefs.getInt(f"OI/{name}/button"),
            )

        def getAxisBindingOfName(name: str) -> AnalogInput:
            return lambda: self.controllers[
                self.prefs.getInt(f"OI/{name}/controller")
            ].getRawAxis(self.prefs.getInt(f"OI/{name}/axis"))

        # pylint: disable-next=unused-variable
        def getPOVBindingOfName(name: str) -> typing.Tuple[Joystick, int, int]:
            binding = controlScheme[name]
            return (
                self.controllers[binding[0]],
                binding[1]["POV"][1],
                binding[1]["POV"][0],
            )

        self.fieldRelativeCoordinateModeControl = getButtonBindingOfName(
            constants.kFieldRelativeCoordinateModeControlButtonName
        )
        self.resetGyro = getButtonBindingOfName(constants.kResetGyroButtonName)
        self.defenseStateControl = getButtonBindingOfName("defenseStateControl")

        self.alignAngle = getButtonBindingOfName("alignAngle")
        self.autoWaypoint = getButtonBindingOfName("autoWaypoint")

        self.offVelocity = getButtonBindingOfName("offVelocity")
        self.velocitySetpoint1 = getButtonBindingOfName("setpoint1velocity")
        self.velocitySetpoint2 = getButtonBindingOfName("setpoint2velocity")

        self.intakeCoral = getButtonBindingOfName("intakeCoral")
        self.intakeScoring = getButtonBindingOfName("intakeScoring")

        self.elevatorL1 = getButtonBindingOfName("elevatorL1")
        self.elevatorL2 = getButtonBindingOfName("elevatorL2")
        self.elevatorL3 = getButtonBindingOfName("elevatorL3")
        self.elevatorL4 = getButtonBindingOfName("elevatorL4")
        self.algaeLow = getPOVBindingOfName("algaeLow")
        self.algaeLow2 = getPOVBindingOfName("algaeLow2")
        self.algaeLow3 = getPOVBindingOfName("algaeLow3")
        self.algaeHigh = getPOVBindingOfName("algaeHigh")
        self.algaeHigh2 = getPOVBindingOfName("algaeHigh2")
        self.algaeHigh3 = getPOVBindingOfName("algaeHigh3")
        self.elevatorIntakePositionToggleOn = getButtonBindingOfName(
            "elevatorIntakePositionToggleOn"
        )
        self.elevatorIntakePositionToggleOff = getButtonBindingOfName(
            "elevatorIntakePositionToggleOff"
        )
        self.climberDown = getButtonBindingOfName("pullClimberDown")
        self.climberUp = getButtonBindingOfName("pullClimberUp")
        self.elevatorManualUp = getButtonBindingOfName("elevatorManualUp")
        self.elevatorManualDown = getButtonBindingOfName("elevatorManualDown")
        self.climberManualUp = getButtonBindingOfName("climberManualUp")
        self.climberManualDown = getButtonBindingOfName("climberManualDown")
        self.elevatorFudgeUp = getButtonBindingOfName("elevatorFudgeUp")
        self.elevatorFudgeDown = getButtonBindingOfName("elevatorFudgeDown")
        self.intakeFudgeScoreForward = getButtonBindingOfName("intakeFudgeScoreForward")
        self.intakeFudgeScoreBackward = getButtonBindingOfName(
            "intakeFudgeScoreBackward"
        )
        self.intakeFudgeCoralUp = getButtonBindingOfName("intakeFudgeCoralUp")
        self.intakeFudgeCoralDown = getButtonBindingOfName("intakeFudgeCoralDown")
        self.setLeftReef = getButtonBindingOfName("setLeftReef")
        self.setRightReef = getButtonBindingOfName("setRightReef")
        self.setNoSpace = getButtonBindingOfName("setNoSpace")
        self.setCoralSpace = getButtonBindingOfName("setCoralSpace")
        self.elevatorDefaultL1 = getButtonBindingOfName("elevatorDefaultL1")

        self.chassisControls = HolonomicInput(
            SignSquare(
                Invert(
                    Deadband(
                        getAxisBindingOfName(
                            constants.kChassisForwardsBackwardsAxisName
                        ),
                        constants.kXboxJoystickDeadband,
                    ),
                )
            ),
            SignSquare(
                Invert(
                    Deadband(
                        getAxisBindingOfName(constants.kChassisSideToSideAxisName),
                        constants.kXboxJoystickDeadband,
                    ),
                )
            ),
            SignSquare(
                Invert(
                    Deadband(
                        getAxisBindingOfName(constants.kChassisRotationXAxisName),
                        constants.kXboxJoystickDeadband,
                    ),
                )
            ),
            SignSquare(
                Invert(
                    Deadband(
                        getAxisBindingOfName(constants.kChassisRotationYAxisName),
                        constants.kXboxJoystickDeadband,
                    )
                )
            ),
        )
