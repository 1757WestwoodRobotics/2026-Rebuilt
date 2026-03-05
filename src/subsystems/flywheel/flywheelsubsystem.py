from enum import Enum, auto
from commands2 import Command, Subsystem, cmd
from commands2.sysid import SysIdRoutine
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib import DriverStation
from wpilib.sysid import State

from subsystems.flywheel.flywheelsubsystemio import FlywheelSubsystemIO
from util.logtracer import LogTracer

from constants.flywheel import kFlywheelTolerance


class FlywheelSubsystemState(Enum):
    IDLE = auto()
    FIRING = auto()


@autologgable_output
class FlywheelSubsystem(Subsystem):
    """
    Flywheel subsystem class.
    Controls just the motor used for the flywheel
    """

    def __init__(self, io: FlywheelSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.io = io
        self.inputs = FlywheelSubsystemIO.FlywheelSubsystemIOInputs()

        self.state = FlywheelSubsystemState.IDLE
        self.isClosedLoop = True
        self.goal = 0.0  # rad/s

    def periodic(self) -> None:
        LogTracer.resetOuter("FlywheelSubsystem Periodic")
        self.io.updateInputs(self.inputs)
        Logger.processInputs("Flywheel", self.inputs)
        LogTracer.record("UpdateInputs")

        if DriverStation.isDisabled() or self.state == FlywheelSubsystemState.IDLE:
            self.stop()
        elif self.state == FlywheelSubsystemState.FIRING:
            if self.isClosedLoop:
                self.io.set_speed(self.goal)

        LogTracer.record("Closed Loop Control")

        Logger.recordOutput("Flywheel/goal", self.goal)
        Logger.recordOutput("Flywheel/ClosedLoop", self.isClosedLoop)
        Logger.recordOutput("Flywheel/State", self.state.name)
        LogTracer.recordTotal()

    def setClosedLoop(self, closedLoop: bool) -> None:
        """
        Sets whether the flywheel is in closed loop control or not
        """
        self.isClosedLoop = closedLoop
        if not closedLoop:
            self.state = (
                FlywheelSubsystemState.FIRING
            )  # need to set to firing to prevent periodic from stopping the flywheel

    def stop(self) -> None:
        """
        Stops applying power to the flywheel
        """
        self.io.neutral_output()

    def setVolts(self, volts: float) -> None:
        """
        Sets the flywheel voltage in open loop control
        """
        Logger.recordOutput("Flywheel/RawVolts", volts)
        self.state = (
            FlywheelSubsystemState.FIRING
        )  # need to set to firing to prevent periodic from stopping the flywheel
        self.setClosedLoop(False)
        self.io.set_volts(volts)

    def setGoal(self, goal: float) -> None:
        """
        Sets the shooter goal in rad/s
        """
        self.state = FlywheelSubsystemState.FIRING
        self.setClosedLoop(True)
        self.goal = goal

    def flywheelIdle(self) -> None:
        """
        Stops applying power to the flywheel
        """
        self.state = FlywheelSubsystemState.IDLE

    @autolog_output(key="Flywheel/at goal")
    def isAtGoal(self) -> bool:
        """
        Returns whether the flywheel is at the goal speed
        """
        return abs(self.inputs.flywheelSpeed - self.goal) < kFlywheelTolerance

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
            Logger.recordOutput("Flywheel/SysID State", loggedStateStr)

        characterizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(1, 8, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.set_volts,
                (lambda _: None),
                subsystem,
            ),
        )

        return cmd.sequence(
            cmd.runOnce(lambda: self.setClosedLoop(False), self),
            characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward),
            characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            cmd.runOnce(lambda: self.setClosedLoop(True), self),
        )
