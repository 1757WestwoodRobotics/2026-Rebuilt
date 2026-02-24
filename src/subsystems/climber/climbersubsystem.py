from commands2 import Subsystem, cmd
from commands2.command import Command
from commands2.sysid import SysIdRoutine
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib.sysid import State

from subsystems.climber.climbersubsystemio import ClimberSubsystemIO

from constants.climber import (
    kClimberPositionTolerance,
    kClimberMinHeight,
    kClimberMaxHeight,
)


@autologgable_output
class ClimberSubsystem(Subsystem):
    def __init__(self, io: ClimberSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.io = io
        self.inputs = ClimberSubsystemIO.ClimberSubsystemIOInputs()
        self.isClosedLoop = True
        self.climberGoal = 0.0

    def periodic(self) -> None:
        """Run ongoing subsystem periodic process."""
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Climber", self.inputs)

        if self.isClosedLoop:
            self.io.set_climber_position(self.climberGoal)
        Logger.recordOutput("Climber/goal", self.climberGoal)
        Logger.recordOutput("Climber/ClosedLoop", self.isClosedLoop)

    def setClosedLoop(self, closedLoop: bool) -> None:
        self.isClosedLoop = closedLoop

    def setClimberRawVolts(self, volts: float) -> None:
        """Apply a specific amount of volts to motor."""
        Logger.recordOutput("Climber/RawVolts", volts)
        self.io.set_climber_volts(volts)

    def setClimberGoal(self, goal: float) -> None:
        self.climberGoal = goal

    def bumpClimberGoal(self, bumpAmount: float) -> None:
        self.climberGoal += bumpAmount

    def isAtGoal(self, goal: float) -> bool:
        return abs(self.inputs.climberPosition - goal) <= kClimberPositionTolerance

    @autolog_output(key="Climber/at target")
    def atTarget(self) -> bool:
        return self.isAtGoal(self.climberGoal)

    def isAtMin(self) -> bool:
        return (
            self.inputs.climberPosition <= kClimberPositionTolerance + kClimberMinHeight
        )

    def isAtMax(self) -> bool:
        return (
            self.inputs.climberPosition >= kClimberMaxHeight - kClimberPositionTolerance
        )

    def sysIdRoutine(self, subsystem: Subsystem) -> Command:
        """Model the behavior of the system (for better control) by sweeping through the max and min angles."""

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
            Logger.recordOutput("Climber/SysID State", loggedStateStr)

        charactarizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(0.5, 6, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.set_climber_volts,
                (lambda _: None),
                subsystem,
            ),
        )

        return cmd.sequence(
            cmd.runOnce(lambda: self.setClosedLoop(False), self),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(
                self.isAtMax
            ),
            charactarizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(
                self.isAtMin
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(
                self.isAtMax
            ),
            charactarizationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(
                self.isAtMin
            ),
            cmd.runOnce(lambda: self.setClosedLoop(True), self),
        )
