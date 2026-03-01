import numpy as np
from commands2 import Command, Subsystem, cmd
from commands2.sysid import SysIdRoutine
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib.sysid import State
from wpimath.geometry import Rotation2d

from subsystems.turret.turretsubsystemio import TurretSubsystemIO
from util.convenientmath import clampRotation
from util.logtracer import LogTracer

from constants.turret import (
    kTurretMinAngle,
    kTurretMaxAngle,
    kTurretTolerance,
    kTurretStartingAngle,
)


@autologgable_output
class TurretSubsystem(Subsystem):
    def __init__(self, io: TurretSubsystemIO) -> None:
        Subsystem.__init__(self)
        self.setName(type(self).__name__)
        self.io = io
        self.io.set_turret_position(kTurretStartingAngle)
        self.inputs = (
            TurretSubsystemIO.TurretSubsystemIOInputs()
        )  # initialize IO inputs
        self.isClosedLoop = True
        self.turretGoal = Rotation2d()

    def periodic(self) -> None:
        """Run ongoing subsystem periodic process."""
        LogTracer.resetOuter("TurretSubsystem periodic")
        self.io.updateInputs(self.inputs)  # update state of motor
        Logger.processInputs("Turret", self.inputs)
        LogTracer.record("UpdateInputs")

        if self.isClosedLoop:
            self.io.set_turret_angle(
                clampRotation(
                    self.turretGoal, kTurretMinAngle, kTurretMaxAngle
                )  # if isClosedLoop, move the motor to the turretGoal angle (if in allowable range)
            )
        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Turret/goal", self.turretGoal)
        Logger.recordOutput("Turret/ClosedLoop", self.isClosedLoop)
        LogTracer.recordTotal()

    def setClosedLoop(self, closedLoop: bool) -> None:
        self.isClosedLoop = closedLoop

    def setTurretRawVolts(self, volts: float) -> None:
        """Apply a specific amount of volts to motor."""
        Logger.recordOutput("Turret/RawVolts", volts)
        self.io.set_turret_volts(
            volts
        )  # call the appropriate overridden method with the IO class

    def setTurretGoal(self, goal: Rotation2d) -> None:
        self.turretGoal = goal

    @autolog_output(key="Turret/at min")
    def isAtMin(self) -> bool:
        return (
            self.inputs.turretPosition.radians()
            <= kTurretMinAngle.radians() + kTurretTolerance.radians()
        )

    @autolog_output(key="Turret/at max")
    def isAtMax(self) -> bool:
        return (
            self.inputs.turretPosition.radians()
            >= kTurretMaxAngle.radians() - kTurretTolerance.radians()
        )

    def isAtorBeyondGoal(self, goal: Rotation2d) -> bool:
        """Determine whether turret is at or beyond a goal (within a small tolerance)."""
        if np.sign(goal.radians()) > 0:
            return (
                self.inputs.turretPosition.radians()
                >= (goal - kTurretTolerance).radians()
            )
        else:
            return (
                self.inputs.turretPosition.radians()
                <= (goal + kTurretTolerance).radians()
            )

    def isAtGoal(self, goal: Rotation2d) -> bool:
        """Determine whether turret is at goal (within a small tolerance)."""
        return (
            abs(self.inputs.turretPosition.radians() - goal.radians())
            < kTurretTolerance.radians()
        )

    @autolog_output(key="Turret/at target")
    def atTarget(self) -> bool:
        return self.isAtGoal(self.turretGoal)

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
            Logger.recordOutput("Turret/SysID State", loggedStateStr)

        characterizationRoutine = SysIdRoutine(
            SysIdRoutine.Config(0.1, 4, 10, logState),
            SysIdRoutine.Mechanism(
                self.io.set_turret_volts,
                (lambda _: None),
                subsystem,
            ),
        )

        return cmd.sequence(
            cmd.runOnce(lambda: self.setClosedLoop(False), self),
            characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(
                self.isAtMax
            ),
            characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(
                self.isAtMin
            ),
            characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(
                self.isAtMax
            ),
            characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(
                self.isAtMin
            ),
            cmd.runOnce(lambda: self.setClosedLoop(True), self),
        )
