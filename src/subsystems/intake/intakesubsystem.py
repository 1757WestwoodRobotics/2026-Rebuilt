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
    kPivotOscillatedPosition,
    kPivotMinAngle,
    kPivotMaxAngle,
    kPivotRollersAllowedToMoveAngle,
    kPivotStartAngle,
    kPivotMaxVelocity,
    kPivotMaxAcceleration,
    kRollerForwardVoltage,
    kRollerReverseVoltage,
    kPivotTolerance,
)
from util.convenientmath import clampRotation
from util.logtracer import LogTracer
from util.logtunablenumber import LoggedTunableNumber


class PivotGoal(Enum):
    DEPLOYED = kPivotExtendedPosition
    RETRACTED = kPivotRetractedPosition
    DEPOT = kPivotDepotPosition
    OSCILLATE = kPivotOscillatedPosition  # this is a special goal for oscillating between the deployed and a slight upwards tilt


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
    OSCILLATE = (
        RollerGoal.FORWARD,
        PivotGoal.OSCILLATE,
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

        self._oscillationGoal = PivotGoal.OSCILLATE

        self.maxVel = LoggedTunableNumber("Intake/pivot maxVel", kPivotMaxVelocity)
        self.maxAccel = LoggedTunableNumber(
            "Intake/pivot maxAccel", kPivotMaxAcceleration
        )

        self.maxVel.onChange(self.io.setMaxVel)
        self.maxAccel.onChange(self.io.setMaxAccel)

    def periodic(self) -> None:
        LogTracer.resetOuter("IntakeSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Intake", self.inputs)
        LogTracer.record("UpdateInputs")

        pivotGoal = self.pivotGoal.value
        if self.isClosedLoop:
            goalAngle = (
                clampRotation(pivotGoal, kPivotMinAngle, kPivotMaxAngle)
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

            rollerGoal = self.rollerGoal.value
            if self.position.radians() > kPivotRollersAllowedToMoveAngle.radians():
                rollerGoal = RollerGoal.NEUTRAL.value

            self.io.setIntakeTarget(
                rollerGoal,
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
        return self.inputs.pivotPosition

    @autolog_output(key="Intake/at target")
    def isAtTarget(self) -> bool:
        return (
            abs(self.inputs.pivotPosition.radians() - self.pivotGoal.value.radians())
            < kPivotTolerance.radians()
        )

    def isAtGoal(self, goal: Rotation2d) -> bool:
        return (
            abs(self.inputs.pivotPosition.radians() - goal.radians())
            < kPivotTolerance.radians()
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
