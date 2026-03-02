from enum import Enum
from functools import partial
from commands2 import Subsystem
from commands2.command import Command
import commands2.cmd as Commands
from commands2.sysid import SysIdRoutine
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib.sysid import State
from wpimath.geometry import Rotation2d

from robotstate import RobotState
from subsystems.intake.intakesubsystemio import IntakeSubsystemIO

from constants.turret import (
    kTurretStartingAngle,
    kTurretSafetyTolerance,
)
from constants.intake import (
    kPivotRetractedPosition,
    kPivotExtendedPosition,
    kPivotDepotPosition,
    kPivotSafePosition,
    kPivotMinAngle,
    kPivotMaxAngle,
    kPivotStartAngle,
    kRollerForwardVoltage,
    kRollerReverseVoltage,
    kPivotTolerance,
)
from util.convenientmath import clampRotation
from util.logtracer import LogTracer


class PivotGoal(Enum):
    DEPLOYED = kPivotExtendedPosition
    RETRACTED = kPivotRetractedPosition
    DEPOT = kPivotDepotPosition


class RollerGoal(Enum):
    FORWARD = kRollerForwardVoltage
    NEUTRAL = 0
    REVERSE = kRollerReverseVoltage


class IntakeSubsystemGoal(Enum):
    DEPLOYED = (
        RollerGoal.FORWARD,
        PivotGoal.DEPLOYED,
    )
    EXTENDED = (
        RollerGoal.NEUTRAL,
        PivotGoal.DEPLOYED,
    )
    RETRACTED = (
        RollerGoal.NEUTRAL,
        PivotGoal.RETRACTED,
    )
    REVERSED = (
        RollerGoal.REVERSE,
        PivotGoal.DEPLOYED,
    )


@autologgable_output
class IntakeSubsystem(Subsystem):

    def __init__(self, io: IntakeSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.io = io
        self.io.setIntakeAngle(kPivotStartAngle)
        self.inputs = IntakeSubsystemIO.IntakeSubsystemIOInputs()

        self.isClosedLoop = True
        self.pivotGoal = PivotGoal.RETRACTED
        self.rollerGoal = RollerGoal.NEUTRAL
        self.pivotFudge = Rotation2d()  # fudge factor, ideally stays at 0

    def periodic(self) -> None:
        LogTracer.resetOuter("IntakeSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Intake", self.inputs)
        LogTracer.record("UpdateInputs")

        if self.isClosedLoop:
            goalAngle = (
                clampRotation(self.pivotGoal.value, kPivotMinAngle, kPivotMaxAngle)
                + self.pivotFudge
            )
            if (
                abs(
                    RobotState.turretRotation.radians() - kTurretStartingAngle.radians()
                )
                > kTurretSafetyTolerance.radians()
                and goalAngle.radians() > kPivotSafePosition.radians()
            ):
                goalAngle = kPivotSafePosition
            self.io.setIntakeTarget(
                self.rollerGoal.value,
                goalAngle,
            )
        LogTracer.record("SetIntakeTarget")

        Logger.recordOutput("Intake/Roller Goal", self.rollerGoal.name)
        Logger.recordOutput("Intake/Pivot Goal", self.pivotGoal.name)
        Logger.recordOutput("Intake/Pivot/Fudge", self.pivotFudge)
        LogTracer.recordTotal()

    def bumpPivotFudge(self, bumpAmount: Rotation2d) -> None:
        self.pivotFudge += bumpAmount

    def setSubsystemGoal(self, goal: IntakeSubsystemGoal):
        self.rollerGoal, self.pivotGoal = goal.value

    def setRollerGoal(self, goal: RollerGoal):
        self.rollerGoal = goal

    def setPivotGoal(self, goal: PivotGoal):
        self.pivotGoal = goal

    @property
    def position(self) -> Rotation2d:
        return Rotation2d(self.inputs.pivotPosition)

    @autolog_output(key="Intake/at target")
    def isAtTarget(self) -> bool:
        return (
            abs(self.inputs.pivotPosition - self.pivotGoal.value.radians())
            < kPivotTolerance.radians()
        )

    def isAtGoal(self, goal: Rotation2d) -> bool:
        return (
            abs(self.inputs.pivotPosition - goal.radians()) < kPivotTolerance.radians()
        )

    def setOpenLoop(self) -> None:
        self.isClosedLoop = False

    def setClosedLoop(self) -> None:
        self.isClosedLoop = True

    def sysIdRoutine(self, subsystem: Subsystem) -> Command:
        def logState(state: State) -> None:
            loggedStateStr = ""
            match state:
                case State.kQuasistaticForward:
                    loggedStateStr = "quasistatic-forward"
                case State.kQuasistaticReverse:
                    loggedStateStr = "quasistatic-reverse"
                case State.kDynamicForward:
                    loggedStateStr = "dynamic-forward"
                case State.kDynamicReverse:
                    loggedStateStr = "dynamic-reverse"
                case State.kNone:
                    loggedStateStr = "none"
            Logger.recordOutput("Intake/SysID State", loggedStateStr)

        characterizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(0.1, 4, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.setPivotVolts,
                (lambda _: None),
                subsystem,
            ),
        )

        return Commands.sequence(
            Commands.runOnce(self.setOpenLoop, self),
            characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(
                partial(self.isAtGoal, PivotGoal.DEPLOYED.value)
            ),
            characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(
                partial(self.isAtGoal, PivotGoal.RETRACTED.value)
            ),
            characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(
                partial(self.isAtGoal, PivotGoal.DEPLOYED.value)
            ),
            characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(
                partial(self.isAtGoal, PivotGoal.RETRACTED.value)
            ),
            Commands.runOnce(self.setClosedLoop, self),
        )
